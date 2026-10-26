// ROS 适配层：解析消息、执行初始化服务交接并发布状态。
// 初始化需要 EKF 输出确认和放行后的新扫描；运行健康由核心统一判断。

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <robot_localization/srv/set_pose.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <array>
#include <algorithm>

#include "ml_localization_manager/manager.hpp"

using namespace ml_loc_mgr;
using InitializationStage = LocalizationManager::InitializationStage;

// 节点参数(集中声明与读取,单一维护点)
struct ManagerParams
{
    std::string map_frame = "map";        // 重定位与 Global 位姿的地图坐标系
    std::string odom_frame = "odom";      // EKF odom 的 frame_id
    std::string base_frame = "base_link"; // EKF child_frame_id
    std::string global_odom_topic = "/global_odometry/filter";
    bool planar_recovery = false;
    std::string set_pose_service = "/global/set_pose";
    double recovery_position_tolerance = 0.5;
    double recovery_yaw_tolerance_deg = 5.0;
    double recovery_timeout = 15.0; // 初始化交接总超时，由适配层负责。
    double icp_status_timeout = 3.0; // ROS 时间；正常仿真暂停不累计心跳超时。
    LocalizationManager::Config mgr;      // 状态机参数(见 manager.hpp)
};

class MlLocalizationManagerNode
{
public:
    MlLocalizationManagerNode(rclcpp::Node::SharedPtr node,
                              const ManagerParams& params)
        : node_(node), params_(params), mgr_(params.mgr)
    {
        // 结构化状态为主；保留旧 String 订阅，兼容尚未升级的 ICP 节点。
        sub_status_diagnostic_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
            "/ml_localization/status_diagnostic", 10,
            [this](diagnostic_msgs::msg::DiagnosticStatus::SharedPtr m) { OnDiagnosticStatus(m); });
        sub_status_ = node_->create_subscription<std_msgs::msg::String>(
            "/ml_localization/status", 10,
            [this](std_msgs::msg::String::SharedPtr m) { OnStatus(m); });
        sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/local_odometry/filter", rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { OnOdom(m); });
        sub_reloc_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/relocalized_pose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) { OnReloc(m); });

        sub_global_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            params_.global_odom_topic, rclcpp::QoS(10).best_effort(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { OnGlobalOdom(m); });

        pub_state_ = node_->create_publisher<std_msgs::msg::String>(
            "/ml_localization/manager_state", 10);

        client_reloc_ = node_->create_client<std_srvs::srv::Trigger>("/trigger_relocalization");
        client_start_tracking_ = node_->create_client<std_srvs::srv::Trigger>("/ml_localization/start_tracking");
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

    // 里程计与重定位消息共用位姿转换，保留原有坐标和四元数处理。
    static Eigen::Matrix4d PoseToMatrix(const geometry_msgs::msg::Pose& pose)
    {
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T(0,3) = pose.position.x;
        T(1,3) = pose.position.y;
        T(2,3) = pose.position.z;
        return T;
    }

    // 仅转换位姿；调用者负责消息的坐标系、传感器时间戳和协方差。
    static geometry_msgs::msg::Pose MatrixToPose(const Eigen::Matrix4d& T)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = T(0,3);
        pose.position.y = T(1,3);
        pose.position.z = T(2,3);
        const Eigen::Quaterniond q(T.block<3,3>(0,0));
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    }

    double NowSec() const { return node_->now().seconds(); }

    void OnOdom(nav_msgs::msg::Odometry::SharedPtr m)
    {
        if (mgr_.InitializationFailed() || mgr_.StartupCompleted()) return;
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
        mgr_.PushOdom(rclcpp::Time(m->header.stamp).seconds(), PoseToMatrix(m->pose.pose));

    }

    // 只把已识别的状态当作心跳；兼容字符串不覆盖已接管的结构化输入。
    void ApplyIcpStatus(const LocalizationManager::StatusInfo& si)
    {
        if (si.level == LocalizationManager::StatusInfo::Level::kUnknown) return;
        const bool changed = mgr_.OnStatus(si, NowSec());
        if (changed) {
            RCLCPP_INFO(node_->get_logger(), "[manager] state -> %s (ICP status)",
                        LocalizationManager::StateString(mgr_.GetState()));
        }
        PublishState();
    }

    void OnDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::SharedPtr m)
    {
        if (mgr_.InitializationFailed()) return;
        auto value = [&m](const std::string& key) -> std::string {
            for (const auto& item : m->values) if (item.key == key) return item.value;
            return {};
        };
        LocalizationManager::StatusInfo si;
        const std::string state = value("state");
        si.level = LocalizationManager::ParseLevel(state.empty() ? m->message : state);
        if (si.level == LocalizationManager::StatusInfo::Level::kUnknown) return;
        structured_status_seen_ = true;
        // 启动完成后，输入或处理超时均表示当前不可用；不改变配准失败计数。
        if (mgr_.StartupCompleted() &&
            (value("input_stale") == "1" || value("processing_stale") == "1"))
            si.level = LocalizationManager::StatusInfo::Level::kLost;
        try { si.fail_count = std::stoi(value("fail_count")); }
        catch (const std::exception&) { si.fail_count = 0; }
        si.waiting_for_tf = value("waiting_for_tf") == "1";
        try { si.scan_stamp = std::stod(value("scan_stamp")); }
        catch (const std::exception&) { si.scan_stamp = -1.0; }
        ApplyIcpStatus(si);
    }

    void OnStatus(std_msgs::msg::String::SharedPtr m)
    {
        if (mgr_.InitializationFailed()) return; // A late heartbeat cannot undo a failed handoff.
        if (structured_status_seen_) return; // 结构化状态接管后，旧字符串只作兼容回退。
        LocalizationManager::StatusInfo si;
        si.level = LocalizationManager::ParseLevel(m->data);
        if (si.level == LocalizationManager::StatusInfo::Level::kUnknown) return;
        if (mgr_.StartupCompleted() &&
            (m->data.find(" input_stale=1") != std::string::npos ||
             m->data.find(" processing_stale=1") != std::string::npos))
            si.level = LocalizationManager::StatusInfo::Level::kLost;
        si.fail_count = LocalizationManager::ParseFailCount(m->data);
        si.waiting_for_tf = m->data.find(" waiting_for_tf=1") != std::string::npos;
        const auto stamp_pos = m->data.find(" scan_stamp=");
        if (stamp_pos != std::string::npos) {
            try { si.scan_stamp = std::stod(m->data.substr(stamp_pos + 12)); }
            catch (const std::exception&) { si.scan_stamp = -1.0; }
        }
        ApplyIcpStatus(si);
    }

    void OnReloc(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m)
    {
        if (mgr_.InitializationFailed()) return;
        if (m->header.frame_id != params_.map_frame) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[manager] reloc pose frame '%s' != map_frame '%s', dropped",
                m->header.frame_id.c_str(), params_.map_frame.c_str());
            return;
        }
        if (rclcpp::Time(m->header.stamp).nanoseconds() <= 0 ||
            !ValidPose(m->pose.pose) || !ValidCovariance(m->pose.covariance)) {
            RCLCPP_WARN(node_->get_logger(), "[manager] invalid reloc timestamp, pose or covariance rejected");
            return;
        }
        Eigen::Matrix4d T_map = PoseToMatrix(m->pose.pose);
        if (params_.planar_recovery) {
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
            initialization_covariance_ = m->pose.covariance;
            TrySetPose();
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
        if (mgr_.InitializationFailed() || !latest_odom_ || now <= 0.0) return;
        if (mgr_.Stage() == InitializationStage::kWaitingForSetPose && now - set_pose_sent_at_ > 3.0) {
            client_set_pose_->remove_pending_request(set_pose_request_id_);
            InitializationError("set_pose response timeout; result unknown, refusing duplicate initialization");
            return;
        }
        if (mgr_.GetState() == LocalizationManager::State::kRecovering) {
            if (now - recovery_started_at_ > params_.recovery_timeout) {
                InitializationError("EKF/ICP confirmation timed out; inspect before restarting");
                return;
            }
            TrySetPose();
        }
        if (reloc_pending_ && now - reloc_sent_at_ > 3.0) {
            client_reloc_->remove_pending_request(reloc_request_id_);
            reloc_pending_ = false;
            RCLCPP_WARN(node_->get_logger(), "[manager] SC trigger response timed out");
        }
        if (mgr_.Stage() == InitializationStage::kWaitingForStartTracking && now - start_tracking_sent_at_ > 3.0) {
            client_start_tracking_->remove_pending_request(start_tracking_request_id_);
            InitializationError("ICP start_tracking response timeout; no automatic retry");
            return;
        }
        // 完成启动后才监控心跳；启动交接仍使用原有超时策略。
        if (mgr_.CheckHealth(now, params_.icp_status_timeout)) {
            RCLCPP_WARN(node_->get_logger(), "[manager] ICP heartbeat timed out; monitoring without reinitialization");
        }
        LocalizationManager::Action a = mgr_.Tick(now);
        if (a.request_reloc) RequestReloc();
        PublishState();
    }

    static bool ValidCovariance(const std::array<double, 36>& covariance)
    {
        constexpr std::array<size_t, 6> diagonal{{0, 7, 14, 21, 28, 35}};
        for (double value : covariance) if (!std::isfinite(value)) return false;
        for (size_t index : diagonal) if (covariance[index] <= 0.0) return false;
        return true;
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
        recovery_started_at_ = NowSec();
    }

    void OnGlobalOdom(nav_msgs::msg::Odometry::SharedPtr m)
    {
        if (mgr_.Stage() != InitializationStage::kWaitingForGlobal) return;
        if (m->header.frame_id != params_.map_frame ||
            m->child_frame_id != params_.base_frame || !ValidPose(m->pose.pose)) return;
        const double stamp = rclcpp::Time(m->header.stamp).seconds();
        if (stamp < recovery_stamp_ || stamp <= last_confirmation_stamp_ ||
            NowSec() - stamp > params_.mgr.reloc_sync_tolerance) return;
        Eigen::Matrix4d predicted;
        if (!mgr_.PredictAcceptedPose(stamp, predicted)) return;
        last_confirmation_stamp_ = stamp;
        const Eigen::Matrix4d filtered = PoseToMatrix(m->pose.pose);
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
        mgr_.FailInitialization(reason);
        RCLCPP_ERROR(node_->get_logger(), "[manager] initialization stopped: %s", reason);
        PublishState();
    }

    void TrySetPose()
    {
        if (mgr_.Stage() != InitializationStage::kReadyToSetPose || !latest_odom_ ||
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
        message.pose.pose = MatrixToPose(T_map_base);
        message.pose.covariance = initialization_covariance_;
        if (!mgr_.MarkSetPoseSent()) return;
        set_pose_sent_at_ = NowSec();
        recovery_stamp_ = time; // Confirmation must postdate the actual reset.
        auto pending = client_set_pose_->async_send_request(request,
            [this](rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture future) {
                if (mgr_.InitializationFailed()) return;
                try { future.get(); }
                catch (const std::exception& error) { InitializationError(error.what()); return; }
                if (!mgr_.ConfirmSetPose()) return;
                RCLCPP_INFO(node_->get_logger(), "[manager] set_pose acknowledged; waiting for EKF agreement");
            });
        set_pose_request_id_ = pending.request_id;
        RCLCPP_INFO(node_->get_logger(),
            "[manager] set_pose sent: stamp=%.9f position=(%.3f,%.3f,%.3f) yaw=%.2f",
            time, T_map_base(0,3), T_map_base(1,3), T_map_base(2,3), YawDeg(T_map_base));
    }

    void RequestReloc()
    {
        if (reloc_pending_ || mgr_.InitializationFailed()) return;
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
        if (!mgr_.MarkStartTrackingSent()) return;
        start_tracking_sent_at_ = NowSec();
        auto request = client_start_tracking_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                const auto response = future.get();
                if (mgr_.InitializationFailed()) return;
                if (response->success) mgr_.ConfirmStartTracking(NowSec());
                else mgr_.RejectStartTracking();
                RCLCPP_INFO(node_->get_logger(),
                    "[manager] ICP start_tracking response: success=%d; waiting for fresh TRACKING",
                    response->success);
            });
        start_tracking_request_id_ = request.request_id;
    }

    void PublishState()
    {
        // 所有发布入口在此消费变化标记，避免回调与定时器重复通知。
        // 初始化失败和健康标记变化均由核心通知。
        const bool changed = mgr_.ConsumeStateChanged();
        if (!changed) return;
        std_msgs::msg::String msg;
        msg.data = std::string(mgr_.InitializationFailed() ? "INITIALIZATION_FAILED" : LocalizationManager::StateString(mgr_.GetState())) +
                   " fail=" + std::to_string(mgr_.LastStatus().fail_count) +
                   " startup_complete=" + std::to_string(mgr_.StartupCompleted()) +
                   " initialization_error=" + std::to_string(mgr_.InitializationFailed()) +
                   " icp_heartbeat_stale=" + std::to_string(mgr_.HeartbeatStale()) +
                   " reason=" + mgr_.StateReason();
        pub_state_->publish(msg);
    }

    rclcpp::Node::SharedPtr node_;
    ManagerParams params_;
    LocalizationManager mgr_;

    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr sub_status_diagnostic_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    bool structured_status_seen_ = false;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_global_odom_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_reloc_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_reloc_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_start_tracking_;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr client_set_pose_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    std::array<double, 36> initialization_covariance_{};
    double set_pose_sent_at_ = 0.0, recovery_started_at_ = 0.0;
    int64_t set_pose_request_id_ = 0;
    rclcpp::TimerBase::SharedPtr tick_timer_;
    double recovery_stamp_ = 0.0;
    double last_confirmation_stamp_ = -1.0;
    int confirmation_count_ = 0;
    bool reloc_pending_ = false;
    double reloc_sent_at_ = 0.0, start_tracking_sent_at_ = 0.0;
    int64_t reloc_request_id_ = 0, start_tracking_request_id_ = 0;
};

// 节点参数集中声明与读取:declare 默认值直接取结构体字段默认值,单一维护点
ManagerParams LoadManagerParams(const rclcpp::Node::SharedPtr& node)
{
    // 未声明参数可能被 ROS 静默忽略，先检查参数覆盖表以阻止旧配置误用。
    const auto& overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    for (const auto* name : {"init_pose", "max_reloc_jump", "reloc_grace_period",
                             "reloc_covariance_pos", "reloc_covariance_ori"}) {
        if (overrides.count(name))
            throw std::invalid_argument(std::string("removed legacy parameter: ") + name);
    }
    ManagerParams p;
    p.map_frame = node->declare_parameter<std::string>("map_frame", p.map_frame);
    p.odom_frame = node->declare_parameter<std::string>("odom_frame", p.odom_frame);
    p.base_frame = node->declare_parameter<std::string>("base_frame", p.base_frame);
    p.mgr.reloc_sync_tolerance = node->declare_parameter<double>("reloc_sync_tolerance", p.mgr.reloc_sync_tolerance);
    p.mgr.reloc_request_interval = node->declare_parameter<double>("reloc_request_interval", p.mgr.reloc_request_interval);
    p.mgr.reloc_timeout = node->declare_parameter<double>("reloc_timeout", p.mgr.reloc_timeout);
    p.mgr.reloc_max_attempts = node->declare_parameter<int>("reloc_max_attempts", p.mgr.reloc_max_attempts);
    // 接受正式 YAML 的固定模式标记；旧模式明确报错，不保留运行分支。
    if (node->declare_parameter<std::string>("injection_method", "set_pose") != "set_pose")
        throw std::invalid_argument("injection_method only supports set_pose; measurement mode was removed");
    if (!node->declare_parameter<bool>("startup_only", true))
        throw std::invalid_argument("startup_only must be true; runtime reinitialization is not supported");
    p.set_pose_service = node->declare_parameter<std::string>("set_pose_service", p.set_pose_service);
    p.global_odom_topic = node->declare_parameter<std::string>("global_odom_topic", p.global_odom_topic);
    p.planar_recovery = node->declare_parameter<bool>("planar_recovery", p.planar_recovery);
    p.recovery_position_tolerance = node->declare_parameter<double>("recovery_position_tolerance", p.recovery_position_tolerance);
    p.recovery_yaw_tolerance_deg = node->declare_parameter<double>("recovery_yaw_tolerance_deg", p.recovery_yaw_tolerance_deg);
    p.recovery_timeout = node->declare_parameter<double>("recovery_timeout", p.recovery_timeout);
    p.mgr.max_reloc_age = node->declare_parameter<double>("max_reloc_age", p.mgr.max_reloc_age);
    p.icp_status_timeout = node->declare_parameter<double>("icp_status_timeout", p.icp_status_timeout);
    if (!std::isfinite(p.icp_status_timeout) || p.icp_status_timeout <= 0.0)
        throw std::invalid_argument("icp_status_timeout must be finite and positive");
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
