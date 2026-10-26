#pragma once
// Localization Manager: triggers the separate SC node and validates map observations.
// INIT/LOST -> RELOCALIZING -> RECOVERING -> NORMAL_TRACKING.
// RECOVERING waits for EKF agreement, a successful ICP start_tracking response, and fresh
// ICP TRACKING. Timeouts return to relocalization; DEGRADED alone is not recovery.
// Local odometry transports the accepted pose for one-time EKF initialization
// and agreement checks; this class never maintains another map->odom.

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>

namespace ml_loc_mgr
{

class LocalizationManager
{
public:
    enum class State { kNormalTracking, kRelocalizing, kRecovering, kLost };

    struct Config
    {
        bool startup_only = false; // After first confirmed handoff, monitor without triggering SC.
        // Sensor-time alignment tolerance for reloc and local odometry.
        double reloc_sync_tolerance = 0.5;
        // reloc 与最近一次 icp_pose 的跳变闸门(m):SC 误匹配的粗过滤
        double max_reloc_jump = 100.0;
        // 请求重定位周期(s):relocalizer 需 ~3 帧攒 confirm,过频死锁(566d397)
        double reloc_request_interval = 10.0;
        double reloc_timeout = 30.0;         // 单次尝试超时窗口(s)
        int reloc_max_attempts = 5;          // 连续超时上限 → LOST(仍持续重试)
        // 位姿注入后 INIT 宽限期(s):注入 reloc/init 位姿后,Global EKF 需
        // 一次测量周期才发布 map->odom TF,期间 ICP 仍报 INIT,不能误判成
        // 冷启动再触发一轮 reloc
        double reloc_grace_period = 5.0;
        double recovery_timeout = 15.0;
        double max_reloc_age = 1.5; // Must fit the EKF history window (2 s).
        // reloc_pose 输出协方差(固定;relocalizer 端同款 0.5/0.05)
        double reloc_covariance_pos = 0.5;
        double reloc_covariance_ori = 0.05;
    };

    // ICP 状态快照(ROS 适配层从 /ml_localization/status 解析后传入)
    struct StatusInfo
    {
        enum class Level { kInit, kLost, kDegraded, kTracking, kUnknown };
        Level level = Level::kUnknown;
        int fail_count = 0;
        double scan_stamp = -1.0;
        std::string raw; // 原始字符串(日志/调试)
    };

    // Tick() 输出的动作(ROS 适配层执行)
    struct Action
    {
        bool request_reloc = false;
        std::string reason;
    };

    explicit LocalizationManager(const Config& cfg) : cfg_(cfg) {}

    // ---- 输入 ----

    // EKF odom 历史(reloc 时间同步用);600 条 ≈ 30Hz x 20s
    void PushOdom(double t, const Eigen::Matrix4d& T_odom)
    {
        odom_hist_.push_back({t, T_odom});
        while (odom_hist_.size() > kOdomHistMax) odom_hist_.pop_front();
    }

    // 最近一次 icp_pose(reloc 跳变闸门参照)
    void PushIcpPose(double t, const Eigen::Matrix4d& T_map_base)
    {
        last_icp_pose_ = T_map_base;
        last_icp_pose_time_ = t;
        have_icp_pose_ = true;
    }

    // ICP 状态(每条约 2Hz)。返回是否发生状态迁移(发布 manager_state 用)
    bool OnStatus(const StatusInfo& si, double now)
    {
        last_status_ = si;
        bool changed = false;
        if (state_ == State::kRecovering) {
            if (start_tracking_confirmed_ && si.level == StatusInfo::Level::kTracking &&
                si.scan_stamp > start_tracking_confirmed_at_) {
                startup_completed_ = true;
                SetState(State::kNormalTracking, "ICP confirmed recovery");
                return true;
            }
            return false; // Old LOST/DEGRADED heartbeats do not restart recovery.
        }
        if (cfg_.startup_only && startup_completed_) {
            if (si.level == StatusInfo::Level::kTracking && state_ != State::kNormalTracking) {
                SetState(State::kNormalTracking, "ICP self-recovered"); return true;
            }
            if ((si.level == StatusInfo::Level::kLost || si.level == StatusInfo::Level::kInit) &&
                state_ != State::kLost) {
                SetState(State::kLost, "startup-only: ICP continues without SC"); return true;
            }
            return false;
        }
        if (si.level == StatusInfo::Level::kTracking) {
            if (state_ != State::kNormalTracking) {
                SetState(State::kNormalTracking, "ICP recovered");
                changed = true;
            }
            return changed;
        }
        if (si.level == StatusInfo::Level::kDegraded) return false;
        if (si.level == StatusInfo::Level::kLost) {
            if (state_ == State::kNormalTracking) {
                EnterRelocalizing(now, "ICP LOST");
                changed = true;
            }
            return changed;
        }
        if (si.level == StatusInfo::Level::kInit) {
            // 刚注入过位姿的宽限期内忽略 INIT(ICP 在等 Global EKF 的 TF)
            if (pose_injected_at_ > 0.0 && now - pose_injected_at_ < cfg_.reloc_grace_period)
                return changed;
            if (state_ == State::kNormalTracking) {
                EnterRelocalizing(now, "no map->odom TF (INIT)");
                changed = true;
            }
        }
        return changed;
    }

    // reloc 位姿(ROS 适配层收到 /relocalized_pose 后调用)。
    // 三道闸门:状态门(主动跟踪时忽略自发 reloc)/ 时间同步门 / 跳变门。
    // 通过后进入 RECOVERING，等待 EKF 与 ICP 确认。
    // 拒绝原因写入 reject_reason。
    bool OnRelocPose(const Eigen::Matrix4d& T_map, double t, double now, std::string& reject_reason)
    {
        reject_reason.clear();
        if (cfg_.startup_only && startup_completed_) {
            reject_reason = "startup initialization already completed"; return false;
        }
        if ((state_ != State::kRelocalizing && state_ != State::kLost) ||
            !request_issued_) {
            reject_reason = "no active relocalization request";
            return false;
        }
        if (!T_map.allFinite() || !std::isfinite(t) || t < last_request_at_ ||
            now - t > cfg_.max_reloc_age || t - now > cfg_.reloc_sync_tolerance) {
            reject_reason = "invalid, stale or pre-request reloc pose";
            return false;
        }
        // 2. 时间同步门:reloc 时刻的最近 odom 快照 dt 必须入窗
        if (odom_hist_.empty()) {
            reject_reason = "no odom history";
            return false;
        }
        double dt = 0.0;
        NearestOdomAt(t, dt);
        if (std::fabs(dt) > cfg_.reloc_sync_tolerance) {
            reject_reason = "odom sync dt " + std::to_string(dt) + " s > " +
                            std::to_string(cfg_.reloc_sync_tolerance) + " s";
            return false;
        }
        // 3. 跳变门:与最近 icp_pose 距离超限视为误匹配
        if (have_icp_pose_) {
            double jump = (T_map.block<3,1>(0,3) - last_icp_pose_.block<3,1>(0,3)).norm();
            if (jump > cfg_.max_reloc_jump) {
                reject_reason = "jump " + std::to_string(jump) + " m > " +
                                std::to_string(cfg_.max_reloc_jump) + " m";
                return false;
            }
        }
        accepted_pose_ = T_map;
        accepted_stamp_ = t;
        have_accepted_ = true;
        pose_injected_once_ = true;
        pose_injected_at_ = now;
        start_tracking_confirmed_ = false;
        SetState(State::kRecovering, "reloc validated; waiting for EKF and ICP");
        return true;
    }

    // init_pose 参数注入(ROS 适配层在第一条合法 odom 到达时调用一次)。
    // 已注入过位姿(reloc 先到或本函数已执行)则不覆盖。
    bool InjectInitPose(const Eigen::Matrix4d& T_map, double now, std::string& reject_reason)
    {
        reject_reason.clear();
        if (pose_injected_once_) {
            reject_reason = "pose already injected (reloc first), skipped";
            return false;
        }
        accepted_pose_ = T_map;
        accepted_stamp_ = now;
        have_accepted_ = true;
        pose_injected_once_ = true;
        pose_injected_at_ = now;
        start_tracking_confirmed_ = false;
        SetState(State::kRecovering, "init pose injected; waiting for EKF and ICP");
        return true;
    }

    // ---- 周期驱动(1Hz 由 ROS 适配层调用) ----
    Action Tick(double now)
    {
        Action a;
        if (cfg_.startup_only && startup_completed_) return a;
        if (state_ == State::kNormalTracking) return a;
        if (state_ == State::kRecovering) {
            if (now - pose_injected_at_ < cfg_.recovery_timeout) return a;
            EnterRelocalizing(now, "EKF/ICP recovery confirmation timed out");
        }
        // RELOCALIZING / LOST:按周期请求 SC 重定位
        if (now - last_request_at_ >= cfg_.reloc_request_interval) {
            a.request_reloc = true;
            a.reason = StateString(state_);
            last_request_at_ = now;
            request_issued_ = false; // Adapter confirms an actual service request.
        }
        // RELOCALIZING 超时:进入下一尝试窗口,连续超限转 LOST(仍持续重试)
        if (state_ == State::kRelocalizing &&
            now - reloc_started_at_ >= cfg_.reloc_timeout) {
            attempts_++;
            if (attempts_ >= cfg_.reloc_max_attempts) {
                SetState(State::kLost, "reloc timed out " +
                         std::to_string(attempts_) + " times, keep retrying");
            } else {
                reloc_started_at_ = now; // 下一轮窗口
            }
        }
        return a;
    }

    // ---- 输出 ----

    // 验证通过的位姿(适配层发布 /ml_localization/reloc_pose 后调用 consume)
    // Transport the validated map pose using local motion, without owning map->odom.
    bool PredictAcceptedPose(double t, Eigen::Matrix4d& predicted) const
    {
        if (odom_hist_.empty() || t < accepted_stamp_) return false;
        double dt0, dt1;
        const auto origin = NearestOdomAt(accepted_stamp_, dt0);
        const auto current = NearestOdomAt(t, dt1);
        if (std::fabs(dt0) > cfg_.reloc_sync_tolerance ||
            std::fabs(dt1) > cfg_.reloc_sync_tolerance) return false;
        predicted = accepted_pose_ * origin.inverse() * current;
        return predicted.allFinite();
    }

    void MarkRelocRequested() { request_issued_ = true; }
    void CancelRelocRequest() { request_issued_ = false; }

    void ConfirmStartTracking(double stamp) { start_tracking_confirmed_ = true; start_tracking_confirmed_at_ = stamp; }

    bool StartupCompleted() const { return startup_completed_; }
    bool HasAccepted() const { return have_accepted_; }
    const Eigen::Matrix4d& AcceptedPose() const { return accepted_pose_; }
    void ConsumeAccepted() { have_accepted_ = false; }

    State GetState() const { return state_; }
    const StatusInfo& LastStatus() const { return last_status_; }
    bool ConsumeStateChanged() { bool c = state_changed_; state_changed_ = false; return c; }

    static const char* StateString(State s)
    {
        switch (s) {
            case State::kNormalTracking: return "NORMAL_TRACKING";
            case State::kRelocalizing: return "RELOCALIZING";
            case State::kRecovering: return "RECOVERING";
            case State::kLost: return "LOST";
        }
        return "UNKNOWN";
    }

    // 状态字符串 → 枚举(适配层解析 /ml_localization/status 首 token 用)
    static StatusInfo::Level ParseLevel(const std::string& s)
    {
        if (s.rfind("INIT", 0) == 0) return StatusInfo::Level::kInit;
        if (s.rfind("LOST", 0) == 0) return StatusInfo::Level::kLost;
        if (s.rfind("DEGRADED", 0) == 0) return StatusInfo::Level::kDegraded;
        if (s.rfind("TRACKING", 0) == 0) return StatusInfo::Level::kTracking;
        return StatusInfo::Level::kUnknown;
    }

    // " fail=N" 解析(状态串 " fail=3" 形式;reason 里可能有 '=',故以 " fail=" 定位)
    static int ParseFailCount(const std::string& s)
    {
        const std::string k = " fail=";
        size_t p = s.find(k);
        if (p == std::string::npos) return 0;
        try { return std::stoi(s.substr(p + k.size())); }
        catch (...) { return 0; }
    }

private:
    void EnterRelocalizing(double now, const std::string& reason)
    {
        state_ = State::kRelocalizing;
        reloc_started_at_ = now;
        last_request_at_ = now - cfg_.reloc_request_interval;
        request_issued_ = false;
        attempts_ = 0;
        state_changed_ = true;
        state_reason_ = reason;
    }

    void SetState(State s, const std::string& reason)
    {
        state_ = s;
        state_changed_ = true;
        state_reason_ = reason;
    }

    // odom_hist_ 中找时间戳最接近 t 的快照。
    // dt_out:带符号时间差 ts - t(>0 = odom 比 t 新,<0 = 过期)
    Eigen::Matrix4d NearestOdomAt(double t, double& dt_out) const
    {
        Eigen::Matrix4d best_T = Eigen::Matrix4d::Identity();
        double best = 1e18;
        dt_out = 0.0;
        for (const auto& [ts, T] : odom_hist_) {
            double d = std::fabs(ts - t);
            if (d < best) { best = d; best_T = T; dt_out = ts - t; }
        }
        return best_T;
    }

    Config cfg_;
    State state_ = State::kNormalTracking;
    bool state_changed_ = false;
    bool startup_completed_ = false;
    std::string state_reason_;

    static constexpr size_t kOdomHistMax = 600; // EKF 30Hz ≈ 20s
    std::deque<std::pair<double, Eigen::Matrix4d>> odom_hist_;

    StatusInfo last_status_;
    Eigen::Matrix4d last_icp_pose_ = Eigen::Matrix4d::Identity();
    double last_icp_pose_time_ = 0.0;
    bool have_icp_pose_ = false;

    double accepted_stamp_ = 0.0;
    Eigen::Matrix4d accepted_pose_ = Eigen::Matrix4d::Identity();
    bool have_accepted_ = false;

    // 已注入过位姿(reloc 或 init):防自发 reloc / init_pose 二次覆盖
    bool pose_injected_once_ = false;
    double pose_injected_at_ = 0.0; // 最近一次注入时刻(INIT 宽限判定)

    double reloc_started_at_ = 0.0; // 当前 RELOCALIZING 尝试窗口起点
    bool request_issued_ = false;
    bool start_tracking_confirmed_ = false;
    double start_tracking_confirmed_at_ = -1.0;
    int attempts_ = 0;              // 连续超时次数
    double last_request_at_ = 0.0;  // 上次请求 reloc 时刻(节流)
};

} // namespace ml_loc_mgr
