// ROS interface for scan-to-map localization.
// Resolves T_map_base_init through the TF chain at the exact scan time,
// then publishes accepted map poses with the original scan timestamp.
// Owns bounded scan waiting, TF lookup, startup compatibility, status and start_tracking.
// Does not publish TF or manage Scan Context relocalization.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "ml_icp_localization/map_loader.hpp"
#include "ml_icp_localization/tracker.hpp"

using namespace ml_icp_loc;

// 节点参数(集中声明与读取,单一维护点)
struct NodeParams
{
    std::string input_cloud_topic = "/cloud_registered_body"; // 点云订阅源
    std::string map_frame = "map";         // 全局系(icp_pose/icp_path/map 的 frame)
    std::string odom_frame = "odom";       // EKF odom 的 frame_id
    std::string base_frame = "base_link";  // body 系(EKF child_frame_id / 点云 frame)
    double scan_timeout = 2.0;            // Input freshness in ROS (sensor) time.
    double tf_wait_timeout = 1.0;         // Maximum pending-scan wait in steady-clock seconds.
    double pose_covariance = 0.3;          // icp_pose 位置协方差底噪 σ(m)
    double orientation_covariance = 0.05;  // icp_pose 姿态协方差底噪 σ(rad)
    // 初始位姿注入 [x,y,z,roll,pitch,yaw](rad),空 = 不注入。单 EKF 测试链路
    // (无 Manager)下,首次获取 map->odom TF 前以此为 map 基准合成配准初值
    // (不进 EKF;EKF 只吃 icp_pose + odom 增量,首条 icp_pose 即冷启动测量)。
    // Manager 链路留空。旧手工模式仅在从未出现全局 TF 时允许使用此初值。
    std::vector<double> init_pose;
    // 动态协方差(2026-09-12 定稿公式):
    //   σ_pos = max(pose_covariance, covariance_fitness_scale x fitness)
    //   σ_ori = max(orientation_covariance, covariance_fitness_scale_ori x fitness)
    double covariance_fitness_scale = 3.0;
    double covariance_fitness_scale_ori = 0.5;
    Tracker::Config tracker;               // 配准/失效计数参数
    std::string map_directory = "/";
    std::string pose_file = "isam_poses.txt";
    bool pub_map = true;
    bool wait_for_initialization = false; // Manager releases startup through start_tracking.
    bool enter_lost_on_failure = true; // Disable only for continuous offline diagnostics.
};

class MlIcpLocalizationNode
{
public:
    MlIcpLocalizationNode(rclcpp::Node::SharedPtr node,
                          const NodeParams& params,
                          const std::vector<Eigen::Matrix4d>& poses)
        : node_(node), params_(params),
          tracker_(params.map_directory, poses, params.tracker),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_, node_) // 显式传 node:与节点同 executor/时钟域
                                          // (sim time 下内部自建 node 不走 /clock)
    {
        sub_cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            params_.input_cloud_topic, rclcpp::QoS(10).best_effort(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { OnCloud(m); });
        // Manager 验证 reloc 并发布 /ml_localization/reloc_pose 后调用:
        // 清零失效计数与 LOST,恢复配准
        srv_start_tracking_ = node_->create_service<std_srvs::srv::Trigger>(
            "ml_localization/start_tracking",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                (void)req;
                OnStartTracking(res);
            });

        pub_icp_path_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/ml_localization/icp_path", 10);
        pub_icp_pose_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ml_localization/icp_pose", 10);
        pub_status_ = node_->create_publisher<std_msgs::msg::String>(
            "/ml_localization/status", 10);

        track_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(int(1000.0 / params_.tracker.icp_rate)),
            [this]() { TrackTick(); });

        if (auto map = tracker_.GlobalMap()) {
            RCLCPP_INFO(node_->get_logger(), "resident map loaded: %zu points; XY window %.1f m, update margin %.1f m",
                        map->size(), params_.tracker.submap_size, params_.tracker.submap_update_margin);
        }
        if (params_.pub_map) PublishMap(params_.map_directory + "/map.pcd");
    }

    // 全局地图发布(1m 仅可视化);window 模式复用常驻配准地图,避免重复读盘。
    // transient_local + KeepLast(1),
    // 发一次即可——后订阅者(rviz 晚开)从缓存拿最后一帧,不再需要 5s 重发
    void PublishMap(const std::string& map_pcd_path)
    {
        auto map_all = tracker_.GlobalMap();
        auto loaded = pcl::make_shared<pcl::PointCloud<PointType>>();
        pcl::PointCloud<PointType>::Ptr map_vis(new pcl::PointCloud<PointType>());
        if (!map_all && pcl::io::loadPCDFile<PointType>(map_pcd_path, *loaded) == -1) {
            RCLCPP_WARN(node_->get_logger(), "map.pcd not found at %s, skip map publish",
                        map_pcd_path.c_str());
            return;
        }
        if (!map_all) map_all = loaded;
        pcl::VoxelGrid<PointType> vg_map;
        vg_map.setLeafSize(1.0f, 1.0f, 1.0f); // 可视化用 1m 降采样
        vg_map.setInputCloud(map_all);
        vg_map.filter(*map_vis);
        pub_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ml_localization/map", rclcpp::QoS(1).transient_local());
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*map_vis, map_msg);
        map_msg.header.stamp = node_->now();
        map_msg.header.frame_id = params_.map_frame;
        pub_map_->publish(map_msg);
        RCLCPP_INFO(node_->get_logger(),
                    "map.pcd published (%zu pts, transient_local) to /ml_localization/map",
                    map_vis->size());
    }

private:
    static double YawDeg(const Eigen::Matrix4d& T)
    {
        return std::atan2(T(1, 0), T(0, 0)) * 180.0 / M_PI;
    }

    void OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr m)
    {
        // frame 校验:scan 必须是 base_frame 系(body 系)
        if (m->header.frame_id != params_.base_frame) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[icp_loc] cloud frame '%s' != base_frame '%s', dropped",
                m->header.frame_id.c_str(), params_.base_frame.c_str());
            return;
        }
        if (rclcpp::Time(m->header.stamp).nanoseconds() <= 0) return;
        std::lock_guard<std::mutex> lock(mtx_buf_);
        scan_stamp_ = rclcpp::Time(m->header.stamp);
        scan_latest_ = std::move(m);
    }

    // Manager 调 /ml_localization/start_tracking:清零失效计数与 LOST,恢复配准。
    // Manager 调用时序:验证 reloc → 初始化 Global EKF
    // → 验证 Global EKF 输出已与初值一致 → 调本服务。
    void OnStartTracking(std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (params_.wait_for_initialization && !initialization_complete_) {
            initialization_complete_ = true;
            // Control boundary in ROS time, not a synthetic measurement stamp.
            initialization_release_stamp_ = node_->now().seconds();
        }
        tracker_.Reset();
        have_prediction_ = false; // 复位后首帧重新查询 TF,不沿用旧值
        // Drop a pending scan, but never reset the global-TF latch or consumed stamp.
        pending_scan_.reset();
        waiting_for_tf_ = false;
        // has_received_global_tf_ must never be cleared by reset.
        res->success = true;
        res->message = "tracking started";
        RCLCPP_INFO(node_->get_logger(),
                    "[icp_loc] start_tracking: fail counts and LOST cleared, resume");
    }

    // T_map_base maps a body point directly into map. TF composes and
    // interpolates map <- odom <- base_link at ONE sensor timestamp.
    // No timeout here: blocking the executor would delay input processing.
    bool LookupInitialPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_map_base,
                           bool& used_seed)
    {
        used_seed = false;
        // This availability-only query protects the legacy seed latch. Its
        // latest transform is never used as a pose prediction.
        if (!params_.init_pose.empty() && !has_received_global_tf_ &&
            tf_buffer_.canTransform(params_.map_frame, params_.base_frame, tf2::TimePointZero))
            has_received_global_tf_ = true;
        try {
            const auto transform = tf_buffer_.lookupTransform(
                params_.map_frame, params_.base_frame,
                tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));
            const auto& rotation = transform.transform.rotation;
            Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
            const auto& position = transform.transform.translation;
            if (!std::isfinite(q.squaredNorm()) || std::fabs(q.squaredNorm() - 1.0) > 0.01 ||
                !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                    "[icp_loc] invalid map<-base TF, skipping");
                return false;
            }
            T_map_base = Eigen::Matrix4d::Identity();
            T_map_base.block<3,3>(0,0) = q.normalized().toRotationMatrix();
            T_map_base.block<3,1>(0,3) = Eigen::Vector3d(position.x, position.y, position.z);
            has_received_global_tf_ = true;
            return true;
        } catch (const tf2::TransformException& error) {
            // Preserve the old manual bootstrap without an odometry subscription.
            // The local TF must cover this scan, and no global TF may have appeared.
            if (!has_received_global_tf_ && !params_.init_pose.empty() &&
                tf_buffer_.canTransform(params_.odom_frame, params_.base_frame,
                    tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())))) {
                T_map_base = PoseVectorToMatrix(params_.init_pose);
                used_seed = true;
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                    "[icp_loc] legacy init_pose seed; waiting for global TF");
                return T_map_base.allFinite();
            }
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[icp_loc] waiting for map<-base TF at scan_stamp=%.9f: %s",
                stamp.seconds(), error.what());
            return false;
        }
    }

    // 2Hz 心跳状态发布(Manager 状态机与 watchdog 消费):
    //   INIT     - 无 map->odom TF(冷启动/Global EKF 尚无测量)
    //   TRACKING - 最近一轮配准通过
    //   DEGRADED - 单帧失效(尚未达到 LOST 阈值)
    //   LOST     - 连续失效达阈值;仅 pause_on_lost=true 时暂停等待 start_tracking
    void PublishStatus()
    {
        auto st = tracker_.GetStatus();
        std_msgs::msg::String msg;
        const double scan_age = scan_latest_ ? node_->now().seconds() - scan_stamp_.seconds() : -1.0;
        const bool input_stale = !scan_latest_ || scan_age > params_.scan_timeout || scan_age < -params_.scan_timeout;
        if (!have_prediction_) msg.data = "INIT";
        else if (input_stale) msg.data = "DEGRADED";
        else if (st.lost) msg.data = "LOST";
        else if (!st.last_result.accepted) msg.data = "DEGRADED";
        else msg.data = "TRACKING";
        msg.data += " fitness=" + std::to_string(st.last_result.fitness) +
                    " fail=" + std::to_string(st.fail_count);
        if (!st.last_result.rejection_reason.empty())
            msg.data += " reason=" + st.last_result.rejection_reason;
        msg.data += " backend=" + params_.tracker.registration_backend +
                    " registration_ms=" + std::to_string(st.last_result.registration_ms) +
                    " scan_stamp=" + std::to_string(last_attempt_scan_stamp_) +
                    " converged=" + std::to_string(st.last_result.converged) +
                    " inlier_1m=" + std::to_string(st.last_result.inlier_fraction) +
                    " correction_m=" + std::to_string(st.last_result.correction_m) +
                    " correction_yaw_deg=" + std::to_string(st.last_result.correction_yaw_deg) +
                    " auto_lost=" + std::to_string(params_.enter_lost_on_failure) +
                    " paused=" + std::to_string(st.lost && params_.tracker.pause_on_lost);
        msg.data += " waiting_for_initialization=" +
                    std::to_string(params_.wait_for_initialization && !initialization_complete_);
        msg.data += " waiting_for_tf=" + std::to_string(waiting_for_tf_) +
                    " tf_dropped=" + std::to_string(tf_dropped_);
        msg.data += " input_stale=" + std::to_string(input_stale) +
                    " input_age=" + std::to_string(scan_age);
        pub_status_->publish(msg);
    }

    // icp_rate Hz 配准
    void TrackTick()
    {
        if (params_.wait_for_initialization && !initialization_complete_) {
            PublishStatus();
            return;
        }
        if (tracker_.IsLost() && params_.tracker.pause_on_lost) {
            PublishStatus();
            return; // Explicit pause policy for Manager-controlled recovery.
        }

        if (!pending_scan_) {
            std::lock_guard<std::mutex> lock(mtx_buf_);
            if (!scan_latest_ || scan_latest_->data.empty() ||
                scan_stamp_.nanoseconds() == last_consumed_scan_stamp_ns_) {
                PublishStatus();
                return;
            }
            pending_scan_ = scan_latest_;
            pending_stamp_ = scan_stamp_;
            pending_since_ = std::chrono::steady_clock::now();
        }
        const auto scan_stamp = pending_stamp_;
        // Time zero asks TF for the latest transform, not the sensor-time pose.
        if (scan_stamp.nanoseconds() <= 0 ||
            (params_.wait_for_initialization && scan_stamp.seconds() <= initialization_release_stamp_)) {
            last_consumed_scan_stamp_ns_ = scan_stamp.nanoseconds();
            pending_scan_.reset();
            waiting_for_tf_ = false;
            PublishStatus();
            return;
        }
        const double wait_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - pending_since_).count();
        if (wait_seconds > params_.tf_wait_timeout) {
            ++tf_dropped_;
            last_consumed_scan_stamp_ns_ = scan_stamp.nanoseconds();
            pending_scan_.reset();
            waiting_for_tf_ = false;
            have_prediction_ = false;
            RCLCPP_WARN(node_->get_logger(),
                "[icp_loc] drop scan_stamp=%.9f after %.3f s waiting for sensor-time TF; fail count unchanged",
                scan_stamp.seconds(), wait_seconds);
            PublishStatus();
            return;
        }
        Eigen::Matrix4d T_map_base_init;
        bool used_seed = false;
        if (!LookupInitialPose(scan_stamp, T_map_base_init, used_seed)) {
            waiting_for_tf_ = true;
            have_prediction_ = false;
            PublishStatus();
            return; // Keep this scan eligible while callbacks receive later TF.
        }
        waiting_for_tf_ = false;
        have_prediction_ = !used_seed;
        auto scan = pending_scan_;
        pending_scan_.reset();
        last_consumed_scan_stamp_ns_ = scan_stamp.nanoseconds();
        last_attempt_scan_stamp_ = scan_stamp.seconds();
        const auto track_start = std::chrono::steady_clock::now();
        const auto previous_status = tracker_.GetStatus();
        // Preprocess only the selected scan after its sensor-time TF is ready.
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*scan, *cloud);
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(0.4f, 0.4f, 0.4f);
        voxel.setInputCloud(cloud);
        voxel.filter(*filtered);
        const auto result = tracker_.TrackOnce(filtered, T_map_base_init);
        const double track_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - track_start).count();
        const auto st = tracker_.GetStatus();
        const std::string failure_limit = params_.enter_lost_on_failure
            ? std::to_string(params_.tracker.failure_count_before_relocalization) : "disabled";
        // At 2 Hz, log every attempted registration so failures and recoveries
        // can be matched to the exact recorded sensor scan.
        RCLCPP_INFO(node_->get_logger(),
            "[icp_loc] %s scan_stamp=%.9f tf_wait=%.3f s scan_age_ros=%.3f s"
            " | converged=%d fitness=%.4f inlier_1m=%.3f"
            " correction=%.3f m yaw_delta=%.2f deg"
            " | registration=%.1f ms track_total=%.1f ms fail=%d/%s reason=%s"
            " | pred_map=(%.3f,%.3f,%.3f) yaw=%.2f result_map=(%.3f,%.3f,%.3f) yaw=%.2f"
            " | submap=%s updated=%d points=%zu build_ms=%.2f center=(%.2f,%.2f)",
            result.accepted ? "ACCEPT" : "REJECT", scan_stamp.seconds(), wait_seconds,
            (node_->now() - scan_stamp).seconds(), result.converged,
            result.fitness, result.inlier_fraction, result.correction_m, result.correction_yaw_deg,
            result.registration_ms, track_ms, st.fail_count,
            failure_limit.c_str(),
            result.accepted ? "none" : result.rejection_reason.c_str(),
            T_map_base_init(0,3), T_map_base_init(1,3), T_map_base_init(2,3), YawDeg(T_map_base_init),
            result.T_map_base(0,3), result.T_map_base(1,3), result.T_map_base(2,3), YawDeg(result.T_map_base),
            params_.tracker.submap_strategy.c_str(), result.submap_updated, result.submap_points,
            result.submap_ms, result.window_center_x, result.window_center_y);
        if (result.accepted) {
            if (previous_status.lost) {
                RCLCPP_INFO(node_->get_logger(),
                    "[icp_loc] recovered LOST -> TRACKING scan_stamp=%.9f after %d rejected scans",
                    scan_stamp.seconds(), previous_status.fail_count);
            }
            PublishIcpPath(result.T_map_base, scan_stamp);
            PublishIcpPose(result.T_map_base, scan_stamp, result.fitness);
        }
        // Quality state and pause policy are independent. Log only the transition.
        if (params_.enter_lost_on_failure && !tracker_.IsLost() && tracker_.NeedsRelocalization()) {
            tracker_.MarkLost();
            RCLCPP_WARN(node_->get_logger(),
                "[icp_loc] entering LOST after %d rejected scans; %s",
                st.fail_count, params_.tracker.pause_on_lost
                    ? "paused, waiting for start_tracking"
                    : "continuing with fresh scans and sensor-time TF predictions");
        }
        PublishStatus();
    }

    // init_pose 参数 [x,y,z,roll,pitch,yaw] → 4x4 齐次变换(map 系)
    static Eigen::Matrix4d PoseVectorToMatrix(const std::vector<double>& v)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rx(v[3], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry(v[4], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz(v[5], Eigen::Vector3d::UnitZ());
        T.block<3,3>(0,0) = (rz * ry * rx).toRotationMatrix();
        T(0,3) = v[0];
        T(1,3) = v[1];
        T(2,3) = v[2];
        return T;
    }

    // map 系绝对位姿发布(Global EKF 的测量源):
    // PoseWithCovarianceStamped,stamp = scan 测量时刻(非 now()),
    // 动态协方差:σ_pos = max(pose_covariance, covariance_fitness_scale x fitness),
    //            σ_ori = max(orientation_covariance, covariance_fitness_scale_ori x fitness)
    // (fitness 差 → 协方差大 → Global EKF 权重自动降低;有底噪,永不置零)。
    // 仅配准成功时调用(失败帧不发)
    void PublishIcpPose(const Eigen::Matrix4d& T_map_base, const rclcpp::Time& stamp,
                        double fitness)
    {
        Eigen::Quaterniond q(T_map_base.block<3,3>(0,0));
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = params_.map_frame;
        msg.pose.pose.position.x = T_map_base(0,3);
        msg.pose.pose.position.y = T_map_base(1,3);
        msg.pose.pose.position.z = T_map_base(2,3);
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        const double p_sigma = std::max(params_.pose_covariance,
                                        params_.covariance_fitness_scale * fitness);
        const double o_sigma = std::max(params_.orientation_covariance,
                                        params_.covariance_fitness_scale_ori * fitness);
        const double pv = p_sigma * p_sigma;
        const double ov = o_sigma * o_sigma;
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = pv;   // x
        msg.pose.covariance[7] = pv;   // y
        msg.pose.covariance[14] = pv;  // z
        msg.pose.covariance[21] = ov;  // roll
        msg.pose.covariance[28] = ov;  // pitch
        msg.pose.covariance[35] = ov;  // yaw
        pub_icp_pose_->publish(msg);
    }

    // 真实 GICP 结果轨迹(调试/评估用)
    void PublishIcpPath(const Eigen::Matrix4d& T_map_base_gicp, const rclcpp::Time& stamp)
    {
        Eigen::Quaterniond q(T_map_base_gicp.block<3,3>(0,0));
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = params_.map_frame;
        ps.pose.position.x = T_map_base_gicp(0,3);
        ps.pose.position.y = T_map_base_gicp(1,3);
        ps.pose.position.z = T_map_base_gicp(2,3);
        ps.pose.orientation.w = q.w();
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        icp_path_.header.stamp = ps.header.stamp;
        icp_path_.header.frame_id = params_.map_frame;
        icp_path_.poses.push_back(ps);
        // 上限防长跑无限增长:超限按块剪除(摊薄 erase 开销)
        if (icp_path_.poses.size() > kIcpPathMaxPoses) {
            icp_path_.poses.erase(icp_path_.poses.begin(),
                                  icp_path_.poses.begin() + kIcpPathTrimChunk);
        }
        pub_icp_path_->publish(icp_path_);
    }

    rclcpp::Node::SharedPtr node_;
    NodeParams params_;
    Tracker tracker_; // 配准参数已含于 params_.tracker(tracker 内部持副本)

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_tracking_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_icp_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_icp_pose_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    double last_attempt_scan_stamp_ = -1.0; // Sensor time of the last attempt; -1 before first scan.
    int64_t last_consumed_scan_stamp_ns_ = -1; // Matched or expired; no duplicate attempts.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    nav_msgs::msg::Path icp_path_;
    static constexpr size_t kIcpPathMaxPoses = 5000; // 轨迹话题上限(2Hz 下 ~42min)
    static constexpr size_t kIcpPathTrimChunk = 500; // 超限每次剪除量
    rclcpp::TimerBase::SharedPtr track_timer_;

    // Global EKF map->odom TF 的查询入口(只读;唯一对外 TF 来源)
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool initialization_complete_ = false;
    double initialization_release_stamp_ = -1.0;
    bool have_prediction_ = false;
    bool has_received_global_tf_ = false; // Lifetime latch for legacy manual startup only.
    bool waiting_for_tf_ = false;
    std::size_t tf_dropped_ = 0;
    sensor_msgs::msg::PointCloud2::SharedPtr pending_scan_;
    rclcpp::Time pending_stamp_;
    std::chrono::steady_clock::time_point pending_since_;

    std::mutex mtx_buf_;
    rclcpp::Time scan_stamp_;                 // 最近一次 scan 的测量时刻
    sensor_msgs::msg::PointCloud2::SharedPtr scan_latest_;
};

// 节点参数集中声明与读取:declare 默认值直接取结构体字段默认值,单一维护点
NodeParams LoadNodeParams(const rclcpp::Node::SharedPtr& node)
{
    NodeParams p;
    node->declare_parameter<std::string>("input_cloud_topic", p.input_cloud_topic);
    p.input_cloud_topic = node->get_parameter("input_cloud_topic").as_string();
    node->declare_parameter<std::string>("map_frame", p.map_frame);
    p.map_frame = node->get_parameter("map_frame").as_string();
    node->declare_parameter<std::string>("odom_frame", p.odom_frame);
    p.odom_frame = node->get_parameter("odom_frame").as_string();
    node->declare_parameter<std::string>("base_frame", p.base_frame);
    p.base_frame = node->get_parameter("base_frame").as_string();
    p.scan_timeout = node->declare_parameter<double>("scan_timeout", p.scan_timeout);
    if (!std::isfinite(p.scan_timeout) || p.scan_timeout <= 0.0)
        throw std::invalid_argument("scan_timeout must be finite and positive");
    p.tf_wait_timeout = node->declare_parameter<double>("tf_wait_timeout", p.tf_wait_timeout);
    if (!std::isfinite(p.tf_wait_timeout) || p.tf_wait_timeout <= 0.0)
        throw std::invalid_argument("tf_wait_timeout must be finite and positive");
    node->declare_parameter<double>("pose_covariance", p.pose_covariance);
    p.pose_covariance = node->get_parameter("pose_covariance").as_double();
    node->declare_parameter<double>("orientation_covariance", p.orientation_covariance);
    p.orientation_covariance = node->get_parameter("orientation_covariance").as_double();
    node->declare_parameter<double>("covariance_fitness_scale", p.covariance_fitness_scale);
    p.covariance_fitness_scale = node->get_parameter("covariance_fitness_scale").as_double();
    node->declare_parameter<double>("covariance_fitness_scale_ori",
                                    p.covariance_fitness_scale_ori);
    p.covariance_fitness_scale_ori =
        node->get_parameter("covariance_fitness_scale_ori").as_double();
    p.wait_for_initialization = node->declare_parameter<bool>("wait_for_initialization", p.wait_for_initialization);
    node->declare_parameter<std::vector<double>>("init_pose", p.init_pose);
    p.init_pose = node->get_parameter("init_pose").as_double_array();
    if (!p.init_pose.empty() && p.init_pose.size() != 6) {
        RCLCPP_ERROR(node->get_logger(),
                     "init_pose must be empty or 6 elements [x,y,z,roll,pitch,yaw], "
                     "got %zu, ignored",
                     p.init_pose.size());
        p.init_pose.clear();
    }
    node->declare_parameter<std::string>("registration_backend", p.tracker.registration_backend);
    p.tracker.registration_backend = node->get_parameter("registration_backend").as_string();
    node->declare_parameter<int>("registration_threads", p.tracker.registration_threads);
    p.tracker.registration_threads = node->get_parameter("registration_threads").as_int();
    node->declare_parameter<bool>("enter_lost_on_failure", p.enter_lost_on_failure);
    p.enter_lost_on_failure = node->get_parameter("enter_lost_on_failure").as_bool();
    node->declare_parameter<bool>("pause_on_lost", p.tracker.pause_on_lost);
    p.tracker.pause_on_lost = node->get_parameter("pause_on_lost").as_bool();
    node->declare_parameter<std::string>("submap_strategy", p.tracker.submap_strategy);
    p.tracker.submap_strategy = node->get_parameter("submap_strategy").as_string();
    node->declare_parameter<double>("submap_size", p.tracker.submap_size);
    p.tracker.submap_size = node->get_parameter("submap_size").as_double();
    node->declare_parameter<double>("submap_update_margin", p.tracker.submap_update_margin);
    p.tracker.submap_update_margin = node->get_parameter("submap_update_margin").as_double();
    // Tracker 参数
    node->declare_parameter<double>("icp_rate", p.tracker.icp_rate);
    p.tracker.icp_rate = node->get_parameter("icp_rate").as_double();
    node->declare_parameter<double>("correspondence_distance", p.tracker.correspondence_distance);
    p.tracker.correspondence_distance = node->get_parameter("correspondence_distance").as_double();
    node->declare_parameter<double>("fitness_threshold", p.tracker.fitness_threshold);
    p.tracker.fitness_threshold = node->get_parameter("fitness_threshold").as_double();
    node->declare_parameter<double>("inlier_fraction_threshold",
                                    p.tracker.inlier_fraction_threshold);
    p.tracker.inlier_fraction_threshold =
        node->get_parameter("inlier_fraction_threshold").as_double();
    node->declare_parameter<double>("max_translation_jump", p.tracker.max_translation_jump);
    p.tracker.max_translation_jump = node->get_parameter("max_translation_jump").as_double();
    node->declare_parameter<double>("max_yaw_jump", p.tracker.max_yaw_jump);
    p.tracker.max_yaw_jump = node->get_parameter("max_yaw_jump").as_double();
    node->declare_parameter<int>("submap_halfwidth", p.tracker.submap_halfwidth);
    p.tracker.submap_halfwidth = node->get_parameter("submap_halfwidth").as_int();
    node->declare_parameter<double>("icp_ds", p.tracker.icp_ds);
    p.tracker.icp_ds = node->get_parameter("icp_ds").as_double();
    node->declare_parameter<int>("failure_count_before_relocalization",
                                 p.tracker.failure_count_before_relocalization);
    p.tracker.failure_count_before_relocalization =
        node->get_parameter("failure_count_before_relocalization").as_int();
    // 地图与发布
    node->declare_parameter<std::string>("map_directory", p.map_directory);
    p.map_directory = node->get_parameter("map_directory").as_string();
    // 默认与打包地图一致(ml_bringup/map 只有 isam_poses/odom_poses)
    node->declare_parameter<std::string>("pose_file", p.pose_file);
    p.pose_file = node->get_parameter("pose_file").as_string();
    node->declare_parameter<bool>("pub_map", p.pub_map);
    p.pub_map = node->get_parameter("pub_map").as_bool();
    return p;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ml_icp_localization_node");

    NodeParams params = LoadNodeParams(node);

    MapPackage map_pkg;
    std::string load_reason;
    if (params.tracker.submap_strategy != "window" &&
        !LoadMapPackage(params.map_directory, params.pose_file, map_pkg, load_reason)) {
        RCLCPP_ERROR(node->get_logger(), "%s", load_reason.c_str());
        rclcpp::shutdown();
        return 1;
    }
    if (params.tracker.submap_strategy != "window")
    std::cout << "poses: " << map_pkg.poses.size()
              << ", scan files: " << map_pkg.scan_count << std::endl;
    std::cout << "starting tracker ("
              << params.tracker.icp_rate << " Hz, init from map<-base TF at scan time)." << std::endl;

    if ((params.tracker.registration_backend != "pcl_gicp" &&
         params.tracker.registration_backend != "fast_gicp") || params.tracker.registration_threads < 1) {
        RCLCPP_ERROR(node->get_logger(), "Invalid registration_backend or registration_threads");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "registration backend=%s, threads=%d (FastGICP only)",
                params.tracker.registration_backend.c_str(), params.tracker.registration_threads);
    RCLCPP_INFO(node->get_logger(), "automatic LOST on failures: %s",
                params.enter_lost_on_failure ? "enabled" : "disabled (diagnostic run)");
    RCLCPP_INFO(node->get_logger(), "LOST policy: %s",
                params.tracker.pause_on_lost ? "pause until start_tracking" : "continue matching fresh scans");
    try {
        RCLCPP_INFO(node->get_logger(), "submap strategy=%s size=%.1f margin=%.1f voxel=%.2f",
                    params.tracker.submap_strategy.c_str(), params.tracker.submap_size,
                    params.tracker.submap_update_margin, params.tracker.icp_ds);
        MlIcpLocalizationNode loc_node(node, params, map_pkg.poses);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "localization initialization/runtime failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
