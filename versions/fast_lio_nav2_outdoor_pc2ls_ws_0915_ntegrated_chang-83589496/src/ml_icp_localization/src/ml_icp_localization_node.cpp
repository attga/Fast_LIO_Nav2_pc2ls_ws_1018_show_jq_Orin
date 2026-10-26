// ROS interface for scan-to-map localization.
// Resolves T_map_base_init through the TF chain at the exact scan time,
// then publishes accepted map poses with the original scan timestamp.
// Owns bounded scan waiting, TF lookup, initialization handoff, status and start_tracking.
// Does not publish TF or manage Scan Context relocalization.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
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
#include "node_parameters.hpp"

using namespace ml_icp_loc;

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
        // Manager 确认 Global EKF 初始化后调用；仅放行配准，不携带位姿。
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
        pub_status_diagnostic_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/ml_localization/status_diagnostic", 10);

        track_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(int(1000.0 / params_.icp_rate)),
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
        // 丢弃待处理扫描，保留已消费时间戳，避免复位后重复匹配。
        pending_scan_.reset();
        waiting_for_tf_ = false;
        res->success = true;
        res->message = "tracking started";
        RCLCPP_INFO(node_->get_logger(),
                    "[icp_loc] start_tracking: fail counts and LOST cleared, resume");
    }

    // T_map_base maps a body point directly into map. TF composes and
    // interpolates map <- odom <- base_link at ONE sensor timestamp.
    // No timeout here: blocking the executor would delay input processing.
    bool LookupInitialPose(const rclcpp::Time& stamp, Eigen::Matrix4d& T_map_base)
    {
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
            return true;
        } catch (const tf2::TransformException& error) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[icp_loc] waiting for map<-base TF at scan_stamp=%.9f: %s",
                stamp.seconds(), error.what());
            return false;
        }
    }

    // 2Hz 心跳状态发布(Manager 状态机与 watchdog 消费):
    //   INIT     - 尚未初始化或当前无法取得扫描时刻的 TF 初值
    //   TRACKING - 最近一轮配准通过
    //   DEGRADED - 输入/处理过期、输出非法或配准未通过(尚未达到 LOST 阈值)
    //   LOST     - 连续失效达阈值;仅 pause_on_lost=true 时暂停等待 start_tracking
    void PublishStatus()
    {
        auto st = tracker_.GetStatus();
        std_msgs::msg::String msg;
        const double scan_age = scan_latest_ ? node_->now().seconds() - scan_stamp_.seconds() : -1.0;
        const bool input_stale = !scan_latest_ || scan_age > params_.scan_timeout || scan_age < -params_.scan_timeout;
        // 点云到达不代表已完成处理；持续缺少历史 TF 时，旧配准不能表示当前可用。
        const double processing_age = node_->now().seconds() - last_attempt_scan_stamp_;
        const bool processing_stale = last_attempt_scan_stamp_ <= 0.0 ||
            processing_age > params_.scan_timeout || processing_age < -params_.scan_timeout;
        // 状态与诊断等级只计算一次，两个输出接口共用同一结果。
        const char* state = "DEGRADED";
        auto level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        if (!have_prediction_) {
            state = "INIT";
            level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        } else if (!input_stale && !processing_stale) {
            if (st.lost) state = "LOST";
            else if (!output_invalid_ && st.last_result.accepted) {
                state = "TRACKING";
                level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            }
        }
        msg.data = state;
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
        msg.data += " output_invalid=" + std::to_string(output_invalid_);
        msg.data += " processing_stale=" + std::to_string(processing_stale);
        pub_status_->publish(msg);

        // 保留旧 String 接口，同时提供稳定字段供 Manager/监控工具使用。
        diagnostic_msgs::msg::DiagnosticStatus diagnostic;
        diagnostic.name = "ml_icp_localization";
        diagnostic.message = msg.data;
        diagnostic.level = level;
        const auto add = [&diagnostic](const std::string& key, const std::string& value) {
            diagnostic.values.push_back(diagnostic_msgs::msg::KeyValue{});
            diagnostic.values.back().key = key;
            diagnostic.values.back().value = value;
        };
        add("state", state);
        add("waiting_for_tf", std::to_string(waiting_for_tf_));
        add("tf_dropped", std::to_string(tf_dropped_));
        add("input_stale", std::to_string(input_stale));
        add("processing_stale", std::to_string(processing_stale));
        add("output_invalid", std::to_string(output_invalid_));
        add("fail_count", std::to_string(st.fail_count));
        add("fitness", std::to_string(st.last_result.fitness));
        add("converged", std::to_string(st.last_result.converged));
        add("scan_stamp", std::to_string(last_attempt_scan_stamp_));
        pub_status_diagnostic_->publish(diagnostic);
    }

    // 消费表示该扫描已配准或已丢弃；与 start_tracking 仅清除待处理槽不同。
    sensor_msgs::msg::PointCloud2::SharedPtr ConsumePendingScan()
    {
        last_consumed_scan_stamp_ns_ = pending_stamp_.nanoseconds();
        auto scan = std::move(pending_scan_);
        return scan;
    }

    // 固定一帧等待其传感器时刻 TF；超时丢弃后，下轮再取最新输入。
    // 此处只更新调度/TF 状态，诊断由主回调统一发布。
    sensor_msgs::msg::PointCloud2::SharedPtr TakeScanForTracking(
        rclcpp::Time& scan_stamp, Eigen::Matrix4d& T_map_base_init, double& wait_seconds)
    {
        if (!pending_scan_) {
            std::lock_guard<std::mutex> lock(mtx_buf_);
            if (!scan_latest_ || scan_latest_->data.empty() ||
                scan_stamp_.nanoseconds() == last_consumed_scan_stamp_ns_) {
                return {};
            }
            pending_scan_ = scan_latest_;
            pending_stamp_ = scan_stamp_;
            pending_since_ = std::chrono::steady_clock::now();
        }
        scan_stamp = pending_stamp_;
        // Time zero asks TF for the latest transform, not the sensor-time pose.
        if (scan_stamp.nanoseconds() <= 0 ||
            (params_.wait_for_initialization && scan_stamp.seconds() <= initialization_release_stamp_)) {
            ConsumePendingScan();
            waiting_for_tf_ = false;
            return {};
        }
        wait_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - pending_since_).count();
        if (wait_seconds > params_.tf_wait_timeout) {
            ++tf_dropped_;
            ConsumePendingScan();
            // 这一帧 TF 超时只代表输入不可用，不代表已有预测或配准质量失效。
            // 保留 have_prediction_，并继续标记 waiting_for_tf，避免 Manager 将 INIT 误判为 LOST。
            waiting_for_tf_ = true;
            RCLCPP_WARN(node_->get_logger(),
                "[icp_loc] drop scan_stamp=%.9f after %.3f s waiting for sensor-time TF; fail count unchanged",
                scan_stamp.seconds(), wait_seconds);
            return {};
        }
        if (!LookupInitialPose(scan_stamp, T_map_base_init)) {
            waiting_for_tf_ = true;
            have_prediction_ = false;
            return {}; // 保留这一帧，等待后续 TF；不能提前消费。
        }
        waiting_for_tf_ = false;
        have_prediction_ = true;
        return ConsumePendingScan();
    }

    // icp_rate Hz：准备扫描 → 预处理 → 配准/质量判断 → 发布测量和诊断。
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

        rclcpp::Time scan_stamp;
        Eigen::Matrix4d T_map_base_init;
        double wait_seconds = 0.0;
        auto scan = TakeScanForTracking(scan_stamp, T_map_base_init, wait_seconds);
        if (!scan) {
            PublishStatus();
            return;
        }
        last_attempt_scan_stamp_ = scan_stamp.seconds();
        const auto track_start = std::chrono::steady_clock::now();
        const auto previous_status = tracker_.GetStatus();
        // Preprocess only the selected scan after its sensor-time TF is ready.
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*scan, *cloud);
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> voxel;
        const float leaf = static_cast<float>(params_.tracker.icp_ds);
        voxel.setLeafSize(leaf, leaf, leaf);
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
            if (PublishIcpPose(result.T_map_base, scan_stamp, result.fitness))
                PublishIcpPath(result.T_map_base, scan_stamp);
        }
        // 使用本次配准后的状态快照判断 LOST，仅在状态迁移时记录日志。
        if (params_.enter_lost_on_failure && !st.lost &&
            st.fail_count >= params_.tracker.failure_count_before_relocalization) {
            tracker_.MarkLost();
            RCLCPP_WARN(node_->get_logger(),
                "[icp_loc] entering LOST after %d rejected scans; %s",
                st.fail_count, params_.tracker.pause_on_lost
                    ? "paused, waiting for start_tracking"
                    : "continuing with fresh scans and sensor-time TF predictions");
        }
        PublishStatus();
    }

    // 仅转换 T_map_base；发布函数各自保留时间戳、坐标系和协方差处理。
    static geometry_msgs::msg::Pose MatrixToPose(const Eigen::Matrix4d& T_map_base)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = T_map_base(0,3);
        pose.position.y = T_map_base(1,3);
        pose.position.z = T_map_base(2,3);
        const Eigen::Quaterniond q(T_map_base.block<3,3>(0,0));
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        return pose;
    }

    // map 系绝对位姿发布(Global EKF 的测量源):
    // PoseWithCovarianceStamped,stamp = scan 测量时刻(非 now()),
    // 动态协方差:σ_pos = max(pose_covariance, covariance_fitness_scale x fitness),
    //            σ_ori = max(orientation_covariance, covariance_fitness_scale_ori x fitness)
    // (fitness 差 → 协方差大 → Global EKF 权重自动降低;有底噪,永不置零)。
    // 仅配准成功时调用(失败帧不发)
    bool PublishIcpPose(const Eigen::Matrix4d& T_map_base, const rclcpp::Time& stamp,
                        double fitness)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = params_.map_frame;
        msg.pose.pose = MatrixToPose(T_map_base);
        const double p_sigma = std::max(params_.pose_covariance,
                                        params_.covariance_fitness_scale * fitness);
        const double o_sigma = std::max(params_.orientation_covariance,
                                        params_.covariance_fitness_scale_ori * fitness);
        const double pv = p_sigma * p_sigma;
        const double ov = o_sigma * o_sigma;
        const auto& q = msg.pose.pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        // 检查实际输出；Manager 不在 ICP 到 Global EKF 的测量链路上。
        if (stamp.nanoseconds() <= 0 || msg.header.frame_id.empty() ||
            !T_map_base.allFinite() || !std::isfinite(norm) || std::fabs(norm - 1.0) > 0.01 ||
            !std::isfinite(fitness) || fitness < 0.0 ||
            !std::isfinite(pv) || pv <= 0.0 || !std::isfinite(ov) || ov <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(), "[icp_loc] invalid output pose, stamp or covariance; measurement not published");
            output_invalid_ = true;
            return false;
        }
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = pv;   // x
        msg.pose.covariance[7] = pv;   // y
        msg.pose.covariance[14] = pv;  // z
        msg.pose.covariance[21] = ov;  // roll
        msg.pose.covariance[28] = ov;  // pitch
        msg.pose.covariance[35] = ov;  // yaw
        pub_icp_pose_->publish(msg);
        output_invalid_ = false;
        return true;
    }

    // 真实 GICP 结果轨迹(调试/评估用)
    void PublishIcpPath(const Eigen::Matrix4d& T_map_base_gicp, const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = params_.map_frame;
        ps.pose = MatrixToPose(T_map_base_gicp);
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
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status_diagnostic_;
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
    bool waiting_for_tf_ = false;
    bool output_invalid_ = false; // 独立于配准失败计数；合法测量发布后清除。
    std::size_t tf_dropped_ = 0;
    // 待处理槽固定 TF 等待对象；输入槽可继续被新消息覆盖，积压有界。
    sensor_msgs::msg::PointCloud2::SharedPtr pending_scan_;
    rclcpp::Time pending_stamp_;
    std::chrono::steady_clock::time_point pending_since_;

    std::mutex mtx_buf_;
    rclcpp::Time scan_stamp_;                 // 最近一次 scan 的测量时刻
    sensor_msgs::msg::PointCloud2::SharedPtr scan_latest_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ml_icp_localization_node");

    try {
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
                  << params.icp_rate << " Hz, init from map<-base TF at scan time)." << std::endl;

        RCLCPP_INFO(node->get_logger(), "registration backend=%s, threads=%d (FastGICP only)",
                    params.tracker.registration_backend.c_str(), params.tracker.registration_threads);
        RCLCPP_INFO(node->get_logger(), "automatic LOST on failures: %s",
                    params.enter_lost_on_failure ? "enabled" : "disabled (diagnostic run)");
        RCLCPP_INFO(node->get_logger(), "LOST policy: %s",
                    params.tracker.pause_on_lost ? "pause until start_tracking" : "continue matching fresh scans");
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
