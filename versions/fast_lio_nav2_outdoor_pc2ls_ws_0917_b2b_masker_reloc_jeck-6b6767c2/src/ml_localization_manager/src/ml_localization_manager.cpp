// ROS adapter for the Localization Manager.
// Preserves validated SC scan stamps/covariances, confirms EKF agreement before
// requesting ICP start_tracking, and waits for fresh TRACKING before declaring recovery.
// Local EKF provides /local_odometry/filter for startup pose time alignment.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <robot_localization/srv/set_pose.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "ml_localization_manager/manager.hpp"

using namespace ml_loc_mgr;

// 节点参数(集中声明与读取,单一维护点)
struct ManagerParams
{
    std::string map_frame = "map";        // reloc/icp_pose 校验与发布 frame
    std::string odom_frame = "odom";      // EKF odom 的 frame_id
    std::string base_frame = "base_link"; // EKF child_frame_id
    std::vector<double> init_pose;        // 空=等 SC reloc;[x,y,z,roll,pitch,yaw]
    std::string global_odom_topic = "/global_odometry/filter";
    bool planar_recovery = false;
    std::string injection_method = "measurement"; // Legacy mode remains the default.
    std::string set_pose_service = "/global/set_pose";
    double recovery_position_tolerance = 0.5;
    double recovery_yaw_tolerance_deg = 5.0;
    LocalizationManager::Config mgr;      // 状态机参数(见 manager.hpp)
};

class MlLocalizationManagerNode
{
public:
    MlLocalizationManagerNode(rclcpp::Node::SharedPtr node,
                              const ManagerParams& params)
        : node_(node), params_(params), mgr_(params.mgr)
    {
        sub_status_ = node_->create_subscription<std_msgs::msg::String>(
            "/ml_localization/status", 10,
            [this](std_msgs::msg::String::SharedPtr m) { OnStatus(m); });
        if (!params_.mgr.startup_only) sub_icp_pose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ml_localization/icp_pose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { OnIcpPose(m); });
        sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/local_odometry/filter", rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { OnOdom(m); });
        sub_reloc_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/relocalized_pose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { OnReloc(m); });

        sub_global_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            params_.global_odom_topic, rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { OnGlobalOdom(m); });

        if (params_.injection_method == "measurement") pub_reloc_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ml_localization/reloc_pose", 10);
        pub_state_ = node_->create_publisher<std_msgs::msg::String>(
            "/ml_localization/manager_state", 10);

        client_reloc_ = node_->create_client<std_srvs::srv::Trigger>("/trigger_relocalization");
        client_start_tracking_ = node_->create_client<std_srvs::srv::Trigger>("/ml_localization/start_tracking");
        if (params_.injection_method == "set_pose")
            client_set_pose_ = node_->create_client<robot_localization::srv::SetPose>(params_.set_pose_service);

        tick_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1), [this]() { Tick(); });
        RCLCPP_INFO(node_->get_logger(), "[manager] started (state %s)",
                    LocalizationManager::StateString(mgr_.GetState()));
    }

private:
    static double YawDeg(const Eigen::Matrix4d& T)
    {
        return std::atan2(T(1, 0), T(0, 0)) * 180.0 / M_PI;
    }

    static Eigen::Matrix4d OdomToMatrix(const nav_msgs::msg::Odometry::SharedPtr& m)
    {
        Eigen::Quaterniond q(m->pose.pose.orientation.w, m->pose.pose.orientation.x,
                             m->pose.pose.orientation.y, m->pose.pose.orientation.z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T(0,3) = m->pose.pose.position.x;
        T(1,3) = m->pose.pose.position.y;
        T(2,3) = m->pose.pose.position.z;
        return T;
    }

    static Eigen::Matrix4d PoseToMatrix(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& m)
    {
        Eigen::Quaterniond q(m->pose.pose.orientation.w, m->pose.pose.orientation.x,
                             m->pose.pose.orientation.y, m->pose.pose.orientation.z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T(0,3) = m->pose.pose.position.x;
        T(1,3) = m->pose.pose.position.y;
        T(2,3) = m->pose.pose.position.z;
        return T;
    }

    // init_pose 参数 [x,y,z,roll,pitch,yaw] → 4x4 齐次变换
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

    double NowSec() const { return node_->now().seconds(); }

    void OnOdom(nav_msgs::msg::Odometry::SharedPtr m)
    {
        if (initialization_error_ || (params_.mgr.startup_only && mgr_.StartupCompleted())) return;
        // frame 校验:EKF 输出必须是 odom_frame -> base_frame
        if (m->header.frame_id != params_.odom_frame ||
            m->child_frame_id != params_.base_frame) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[manager] odom frame '%s'->'%s' != expected '%s'->'%s', dropped",
                m->header.frame_id.c_str(), m->child_frame_id.c_str(),
                params_.odom_frame.c_str(), params_.base_frame.c_str());
            return;
        }
        if (!ValidPose(m->pose.pose)) return;
        if (latest_odom_ && rclcpp::Time(m->header.stamp) <= rclcpp::Time(latest_odom_->header.stamp)) return;
        latest_odom_ = m;
        mgr_.PushOdom(rclcpp::Time(m->header.stamp).seconds(), OdomToMatrix(m));
        // init_pose 参数注入:第一条合法 EKF odom 到达时注入一次
        // (reloc 先到则不覆盖,由 InjectInitPose 内部判定)
        if (!init_pose_tried_ && !params_.init_pose.empty()) {
            init_pose_tried_ = true; // 只尝试一次
            Eigen::Matrix4d T_init = PoseVectorToMatrix(params_.init_pose);
            std::string reason;
            if (mgr_.InjectInitPose(T_init, rclcpp::Time(m->header.stamp).seconds(), reason)) {
                RCLCPP_INFO(node_->get_logger(),
                            "[manager] init_pose injected: (%.3f, %.3f, %.3f) yaw %.2f deg",
                            T_init(0,3), T_init(1,3), T_init(2,3), YawDeg(T_init));
                PublishRelocObservation(T_init, m->header.stamp);
                BeginRecovery(m->header.stamp);
                PublishState();
            } else {
                RCLCPP_INFO(node_->get_logger(), "[manager] init_pose skipped: %s",
                            reason.c_str());
            }
        }
    }

    void OnStatus(std_msgs::msg::String::SharedPtr m)
    {
        if (initialization_error_) return; // A late heartbeat cannot undo a failed handoff.
        LocalizationManager::StatusInfo si;
        si.level = LocalizationManager::ParseLevel(m->data);
        if (params_.mgr.startup_only && mgr_.StartupCompleted() &&
            m->data.find(" input_stale=1") != std::string::npos)
            si.level = LocalizationManager::StatusInfo::Level::kLost;
        si.fail_count = LocalizationManager::ParseFailCount(m->data);
        si.raw = m->data;
        const auto stamp_pos = m->data.find(" scan_stamp=");
        if (stamp_pos != std::string::npos) {
            try { si.scan_stamp = std::stod(m->data.substr(stamp_pos + 12)); }
            catch (const std::exception&) { si.scan_stamp = -1.0; }
        }
        if (mgr_.OnStatus(si, NowSec())) {
            RCLCPP_INFO(node_->get_logger(), "[manager] state -> %s (ICP: %s)",
                        LocalizationManager::StateString(mgr_.GetState()), m->data.c_str());
            PublishState();
        }
    }

    void OnIcpPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m)
    {
        if (m->header.frame_id != params_.map_frame) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[manager] icp_pose frame '%s' != map_frame '%s', dropped",
                m->header.frame_id.c_str(), params_.map_frame.c_str());
            return;
        }
        mgr_.PushIcpPose(rclcpp::Time(m->header.stamp).seconds(), PoseToMatrix(m));
    }

    void OnReloc(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m)
    {
        if (initialization_error_) return;
        if (m->header.frame_id != params_.map_frame) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[manager] reloc pose frame '%s' != map_frame '%s', dropped",
                m->header.frame_id.c_str(), params_.map_frame.c_str());
            return;
        }
        if (!ValidPose(m->pose.pose)) {
            RCLCPP_WARN(node_->get_logger(), "[manager] invalid reloc pose rejected");
            return;
        }
        for (size_t i = 0; i < m->pose.covariance.size(); ++i) {
            if (!std::isfinite(m->pose.covariance[i]) ||
                (i % 7 == 0 && m->pose.covariance[i] <= 0.0)) {
                RCLCPP_WARN(node_->get_logger(), "[manager] invalid reloc covariance rejected");
                return;
            }
        }
        Eigen::Matrix4d T_map = PoseToMatrix(m);
        if (params_.injection_method == "set_pose" && params_.planar_recovery) {
            const double yaw = std::atan2(T_map(1,0), T_map(0,0));
            T_map.block<3,3>(0,0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            T_map(2,3) = 0.0;
        }
        std::string reason;
        if (mgr_.OnRelocPose(T_map, rclcpp::Time(m->header.stamp).seconds(), NowSec(), reason)) {
            RCLCPP_INFO(node_->get_logger(),
                        "[manager] reloc ACCEPTED: (%.3f, %.3f, %.3f) yaw %.2f deg "
                        "validated; waiting for EKF initialization/confirmation",
                        T_map(0,3), T_map(1,3), T_map(2,3), YawDeg(T_map));
            BeginRecovery(m->header.stamp);
            if (params_.injection_method == "set_pose") {
                initialization_covariance_ = m->pose.covariance;
                TrySetPose();
            } else {
                PublishRelocObservation(T_map, m->header.stamp, &m->pose.covariance);
            }
            PublishState();
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[manager] reloc REJECTED: %s", reason.c_str());
        }
    }

    // 1Hz:状态机驱动(reloc 请求节流 + 超时/重试/LOST)
    void Tick()
    {
        const double now = NowSec();
        if (params_.injection_method == "set_pose") {
            if (initialization_error_ || !latest_odom_ || now <= 0.0) return;
            if (set_pose_pending_ && now - set_pose_sent_at_ > 3.0) {
                client_set_pose_->remove_pending_request(set_pose_request_id_);
                set_pose_pending_ = false;
                InitializationError("set_pose response timeout; result unknown, refusing duplicate initialization");
                return;
            }
            if (mgr_.GetState() == LocalizationManager::State::kRecovering) {
                if (now - recovery_started_at_ > params_.mgr.recovery_timeout) {
                    InitializationError("EKF/ICP confirmation timed out; inspect before restarting");
                    return;
                }
                TrySetPose();
            }
        }
        if (reloc_pending_ && now - reloc_sent_at_ > 3.0) {
            client_reloc_->remove_pending_request(reloc_request_id_);
            reloc_pending_ = false;
            RCLCPP_WARN(node_->get_logger(), "[manager] SC trigger response timed out");
        }
        if (start_tracking_pending_ && now - start_tracking_sent_at_ > 3.0) {
            client_start_tracking_->remove_pending_request(start_tracking_request_id_);
            start_tracking_pending_ = false;
            if (params_.injection_method == "set_pose") {
                InitializationError("ICP start_tracking response timeout; no automatic retry");
                return;
            }
            RCLCPP_WARN(node_->get_logger(), "[manager] ICP start_tracking response timed out");
        }
        LocalizationManager::Action a = mgr_.Tick(now);
        if (a.request_reloc) RequestReloc();
        if (mgr_.ConsumeStateChanged()) PublishState();
    }

    static bool ValidPose(const geometry_msgs::msg::Pose& pose)
    {
        const auto& p = pose.position;
        const auto& q = pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
               std::isfinite(norm) && std::fabs(norm - 1.0) < 0.01;
    }

    void BeginRecovery(const builtin_interfaces::msg::Time& stamp)
    {
        recovery_stamp_ = rclcpp::Time(stamp).seconds();
        last_confirmation_stamp_ = -1.0;
        confirmation_count_ = 0;
        start_tracking_confirmed_ = false;
        recovery_started_at_ = NowSec();
        set_pose_confirmed_ = false;
        mgr_.ConsumeAccepted();
    }

    void OnGlobalOdom(nav_msgs::msg::Odometry::SharedPtr m)
    {
        if (initialization_error_ ||
            (params_.injection_method == "set_pose" && !set_pose_confirmed_)) return;
        if (mgr_.GetState() != LocalizationManager::State::kRecovering ||
            start_tracking_confirmed_ || start_tracking_pending_ || m->header.frame_id != params_.map_frame ||
            m->child_frame_id != params_.base_frame || !ValidPose(m->pose.pose)) return;
        const double stamp = rclcpp::Time(m->header.stamp).seconds();
        if (stamp < recovery_stamp_ || stamp <= last_confirmation_stamp_ ||
            NowSec() - stamp > params_.mgr.reloc_sync_tolerance) return;
        Eigen::Matrix4d predicted;
        if (!mgr_.PredictAcceptedPose(stamp, predicted)) return;
        last_confirmation_stamp_ = stamp;
        const Eigen::Matrix4d filtered = OdomToMatrix(m);
        const Eigen::Vector3d delta = filtered.block<3,1>(0,3) - predicted.block<3,1>(0,3);
        const double position_error = params_.planar_recovery ? delta.head<2>().norm() : delta.norm();
        double yaw_error = std::fabs(YawDeg(filtered) - YawDeg(predicted));
        yaw_error = std::min(yaw_error, 360.0 - yaw_error);
        if (position_error > params_.recovery_position_tolerance ||
            yaw_error > params_.recovery_yaw_tolerance_deg) {
            confirmation_count_ = 0;
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[manager] waiting for EKF: position_error=%.3f m yaw_error=%.2f deg",
                position_error, yaw_error);
            return;
        }
        if (++confirmation_count_ < 2) return;
        RequestStartTracking();
    }

    void InitializationError(const char* reason)
    {
        initialization_error_ = true;
        RCLCPP_ERROR(node_->get_logger(), "[manager] initialization stopped: %s", reason);
        PublishState();
    }

    void TrySetPose()
    {
        if (!client_set_pose_ || set_pose_pending_ || set_pose_confirmed_ || initialization_error_ ||
            mgr_.GetState() != LocalizationManager::State::kRecovering || !latest_odom_ ||
            !client_set_pose_->service_is_ready()) return;
        const auto stamp = latest_odom_->header.stamp;
        const double time = rclcpp::Time(stamp).seconds();
        if (NowSec() - time > params_.mgr.reloc_sync_tolerance) return;
        Eigen::Matrix4d T_map_base;
        if (!mgr_.PredictAcceptedPose(time, T_map_base)) return;
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        auto& message = request->pose;
        message.header.frame_id = params_.map_frame;
        message.header.stamp = stamp; // Pose was transported using actual local motion.
        message.pose.pose.position.x = T_map_base(0,3);
        message.pose.pose.position.y = T_map_base(1,3);
        message.pose.pose.position.z = T_map_base(2,3);
        Eigen::Quaterniond q(T_map_base.block<3,3>(0,0));
        message.pose.pose.orientation.x = q.x();
        message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z();
        message.pose.pose.orientation.w = q.w();
        message.pose.covariance = initialization_covariance_;
        set_pose_pending_ = true;
        set_pose_sent_at_ = NowSec();
        recovery_stamp_ = time; // Confirmation must postdate the actual reset.
        auto pending = client_set_pose_->async_send_request(request,
            [this](rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture future) {
                set_pose_pending_ = false;
                if (initialization_error_) return;
                try { future.get(); }
                catch (const std::exception& error) { InitializationError(error.what()); return; }
                set_pose_confirmed_ = true;
                RCLCPP_INFO(node_->get_logger(), "[manager] set_pose acknowledged; waiting for EKF agreement");
            });
        set_pose_request_id_ = pending.request_id;
        RCLCPP_INFO(node_->get_logger(),
            "[manager] set_pose sent: stamp=%.9f position=(%.3f,%.3f,%.3f) yaw=%.2f",
            time, T_map_base(0,3), T_map_base(1,3), T_map_base(2,3), YawDeg(T_map_base));
    }

    void RequestReloc()
    {
        if (reloc_pending_ || initialization_error_) return;
        if (!client_reloc_->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "[manager] SC service unavailable; will retry");
            return;
        }
        mgr_.MarkRelocRequested();
        reloc_pending_ = true;
        reloc_sent_at_ = NowSec();
        auto request = client_reloc_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                reloc_pending_ = false;
                const auto response = future.get();
                if (!response->success) mgr_.CancelRelocRequest();
                RCLCPP_INFO(node_->get_logger(), "[manager] SC trigger response: success=%d %s",
                            response->success, response->message.c_str());
            });
        reloc_request_id_ = request.request_id;
        RCLCPP_INFO(node_->get_logger(), "[manager] SC trigger sent");
    }

    void RequestStartTracking()
    {
        if (!client_start_tracking_->service_is_ready()) return;
        start_tracking_pending_ = true;
        start_tracking_sent_at_ = NowSec();
        auto request = client_start_tracking_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                start_tracking_pending_ = false;
                const auto response = future.get();
                if (initialization_error_) return;
                if (response->success && mgr_.GetState() == LocalizationManager::State::kRecovering) {
                    start_tracking_confirmed_ = true;
                    mgr_.ConfirmStartTracking(NowSec());
                }
                RCLCPP_INFO(node_->get_logger(),
                    "[manager] ICP start_tracking response: success=%d; waiting for fresh TRACKING",
                    response->success);
            });
        start_tracking_request_id_ = request.request_id;
    }

    // reloc/init 位姿 → /ml_localization/reloc_pose(Global EKF 测量源):
    // Preserve SC sensor stamp and covariance. Only optional initial poses use
    // the configured covariance defaults, stamped with the first local odometry.
    void PublishRelocObservation(const Eigen::Matrix4d& T_map,
                                 const builtin_interfaces::msg::Time& stamp,
                                 const std::array<double, 36>* covariance = nullptr)
    {
        Eigen::Quaterniond q(T_map.block<3,3>(0,0));
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = params_.map_frame;
        msg.pose.pose.position.x = T_map(0,3);
        msg.pose.pose.position.y = T_map(1,3);
        msg.pose.pose.position.z = T_map(2,3);
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        const double pv = params_.mgr.reloc_covariance_pos * params_.mgr.reloc_covariance_pos;
        const double ov = params_.mgr.reloc_covariance_ori * params_.mgr.reloc_covariance_ori;
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = pv;   // x
        msg.pose.covariance[7] = pv;   // y
        msg.pose.covariance[14] = pv;  // z
        msg.pose.covariance[21] = ov;  // roll
        msg.pose.covariance[28] = ov;  // pitch
        msg.pose.covariance[35] = ov;  // yaw
        if (covariance) msg.pose.covariance = *covariance;
        pub_reloc_->publish(msg);
    }

    void PublishState()
    {
        std_msgs::msg::String msg;
        msg.data = std::string(initialization_error_ ? "INITIALIZATION_FAILED" : LocalizationManager::StateString(mgr_.GetState())) +
                   " fail=" + std::to_string(mgr_.LastStatus().fail_count) +
                   " startup_complete=" + std::to_string(mgr_.StartupCompleted()) +
                   " initialization_error=" + std::to_string(initialization_error_);
        pub_state_->publish(msg);
    }

    rclcpp::Node::SharedPtr node_;
    ManagerParams params_;
    LocalizationManager mgr_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_icp_pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_global_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_reloc_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_reloc_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_reloc_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_start_tracking_;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr client_set_pose_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    std::array<double, 36> initialization_covariance_{};
    bool set_pose_pending_ = false, set_pose_confirmed_ = false, initialization_error_ = false;
    double set_pose_sent_at_ = 0.0, recovery_started_at_ = 0.0;
    int64_t set_pose_request_id_ = 0;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    double recovery_stamp_ = 0.0;
    double last_confirmation_stamp_ = -1.0;
    int confirmation_count_ = 0;
    bool start_tracking_confirmed_ = false;
    bool reloc_pending_ = false, start_tracking_pending_ = false;
    double reloc_sent_at_ = 0.0, start_tracking_sent_at_ = 0.0;
    int64_t reloc_request_id_ = 0, start_tracking_request_id_ = 0;
    bool init_pose_tried_ = false;
};

// 节点参数集中声明与读取:declare 默认值直接取结构体字段默认值,单一维护点
ManagerParams LoadManagerParams(const rclcpp::Node::SharedPtr& node)
{
    ManagerParams p;
    node->declare_parameter<std::string>("map_frame", p.map_frame);
    p.map_frame = node->get_parameter("map_frame").as_string();
    node->declare_parameter<std::string>("odom_frame", p.odom_frame);
    p.odom_frame = node->get_parameter("odom_frame").as_string();
    node->declare_parameter<std::string>("base_frame", p.base_frame);
    p.base_frame = node->get_parameter("base_frame").as_string();
    node->declare_parameter<std::vector<double>>("init_pose", p.init_pose);
    p.init_pose = node->get_parameter("init_pose").as_double_array();
    if (!p.init_pose.empty() && p.init_pose.size() != 6) {
        RCLCPP_ERROR(node->get_logger(),
                     "init_pose must be empty or 6 elements [x,y,z,roll,pitch,yaw], "
                     "got %zu, ignored",
                     p.init_pose.size());
        p.init_pose.clear();
    }
    node->declare_parameter<double>("reloc_sync_tolerance", p.mgr.reloc_sync_tolerance);
    p.mgr.reloc_sync_tolerance = node->get_parameter("reloc_sync_tolerance").as_double();
    node->declare_parameter<double>("max_reloc_jump", p.mgr.max_reloc_jump);
    p.mgr.max_reloc_jump = node->get_parameter("max_reloc_jump").as_double();
    node->declare_parameter<double>("reloc_request_interval", p.mgr.reloc_request_interval);
    p.mgr.reloc_request_interval = node->get_parameter("reloc_request_interval").as_double();
    node->declare_parameter<double>("reloc_timeout", p.mgr.reloc_timeout);
    p.mgr.reloc_timeout = node->get_parameter("reloc_timeout").as_double();
    node->declare_parameter<int>("reloc_max_attempts", p.mgr.reloc_max_attempts);
    p.mgr.reloc_max_attempts = node->get_parameter("reloc_max_attempts").as_int();
    node->declare_parameter<double>("reloc_grace_period", p.mgr.reloc_grace_period);
    p.mgr.reloc_grace_period = node->get_parameter("reloc_grace_period").as_double();
    node->declare_parameter<double>("reloc_covariance_pos", p.mgr.reloc_covariance_pos);
    p.mgr.reloc_covariance_pos = node->get_parameter("reloc_covariance_pos").as_double();
    node->declare_parameter<double>("reloc_covariance_ori", p.mgr.reloc_covariance_ori);
    p.mgr.reloc_covariance_ori = node->get_parameter("reloc_covariance_ori").as_double();
    p.injection_method = node->declare_parameter<std::string>("injection_method", p.injection_method);
    p.set_pose_service = node->declare_parameter<std::string>("set_pose_service", p.set_pose_service);
    p.mgr.startup_only = node->declare_parameter<bool>("startup_only", p.mgr.startup_only);
    if (p.injection_method != "measurement" && p.injection_method != "set_pose")
        throw std::invalid_argument("injection_method must be measurement or set_pose");
    if (p.injection_method == "set_pose" && (!p.mgr.startup_only || !p.init_pose.empty()))
        throw std::invalid_argument("set_pose mode currently requires startup_only=true and no init_pose");
    p.global_odom_topic = node->declare_parameter<std::string>("global_odom_topic", p.global_odom_topic);
    p.planar_recovery = node->declare_parameter<bool>("planar_recovery", p.planar_recovery);
    p.recovery_position_tolerance = node->declare_parameter<double>("recovery_position_tolerance", p.recovery_position_tolerance);
    p.recovery_yaw_tolerance_deg = node->declare_parameter<double>("recovery_yaw_tolerance_deg", p.recovery_yaw_tolerance_deg);
    p.mgr.recovery_timeout = node->declare_parameter<double>("recovery_timeout", p.mgr.recovery_timeout);
    p.mgr.max_reloc_age = node->declare_parameter<double>("max_reloc_age", p.mgr.max_reloc_age);
    return p;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ml_localization_manager");

    ManagerParams params = LoadManagerParams(node);

    MlLocalizationManagerNode mgr_node(node, params);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
