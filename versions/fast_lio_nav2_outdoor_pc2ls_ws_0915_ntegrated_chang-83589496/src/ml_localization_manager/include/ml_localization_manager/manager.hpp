#pragma once
// 定位管理核心：请求独立 SC 重定位，校验结果并等待 EKF/ICP 交接确认。
// RECOVERING 只有收到放行后的新扫描 TRACKING 才完成启动。
// 交接超时由 ROS 适配层锁定为初始化失败；完成启动后只监控，不再次初始化。
// Local 位姿仅用于初始化结果的时间推进与确认，不在此维护第二套 map→odom。

#include <Eigen/Dense>
#include "ml_localization_manager/health_monitor.hpp"
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

    // 初始化进度只在核心持有；服务资源和计时仍由 ROS 适配层管理。
    enum class InitializationStage {
        kWaitingForReloc, kReadyToSetPose, kWaitingForSetPose,
        kWaitingForGlobal, kWaitingForStartTracking, kWaitingForScan,
        kComplete, kFailed
    };

    struct Config
    {
        // Sensor-time alignment tolerance for reloc and local odometry.
        double reloc_sync_tolerance = 0.5;
        // 请求重定位周期(s):relocalizer 需 ~3 帧攒 confirm,过频死锁(566d397)
        double reloc_request_interval = 10.0;
        double reloc_timeout = 30.0;         // 单次尝试超时窗口(s)
        int reloc_max_attempts = 5;          // 连续超时上限 → LOST(仍持续重试)
        double max_reloc_age = 1.5; // Must fit the EKF history window (2 s).
    };

    using StatusInfo = IcpStatus;

    // Tick() 输出的动作(ROS 适配层执行)
    struct Action
    {
        bool request_reloc = false;
    };

    explicit LocalizationManager(const Config& cfg) : cfg_(cfg) {}

    // ---- 输入 ----

    // EKF odom 历史(reloc 时间同步用);600 条 ≈ 30Hz x 20s
    void PushOdom(double t, const Eigen::Matrix4d& T_odom)
    {
        odom_hist_.push_back({t, T_odom});
        while (odom_hist_.size() > kOdomHistMax) odom_hist_.pop_front();
    }

    // ICP 状态(每条约 2Hz)。返回是否发生状态迁移(发布 manager_state 用)
    bool OnStatus(const StatusInfo& si, double now)
    {
        if (InitializationFailed() || si.level == StatusInfo::Level::kUnknown) return false;
        const bool was_stale = health_.HeartbeatStale();
        const auto observation = health_.Observe(si, now);
        if (was_stale) state_changed_ = true; // 主状态不变时仍通知健康标记更新。
        bool changed = false;
        if (state_ == State::kRecovering) {
            if (stage_ == InitializationStage::kWaitingForScan && si.level == StatusInfo::Level::kTracking &&
                si.scan_stamp > start_tracking_confirmed_at_) {
                stage_ = InitializationStage::kComplete;
                SetState(State::kNormalTracking, "ICP confirmed recovery");
                return true;
            }
            return false; // Old LOST/DEGRADED heartbeats do not restart recovery.
        }
        if (StartupCompleted()) {
            if (observation == HealthMonitor::Observation::kTracking && state_ != State::kNormalTracking) {
                SetState(State::kNormalTracking, "ICP self-recovered"); return true;
            }
            if (observation == HealthMonitor::Observation::kLost && state_ != State::kLost) {
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
            if (state_ == State::kNormalTracking) {
                EnterRelocalizing(now, "no map->odom TF (INIT)");
                changed = true;
            }
        }
        return changed;
    }

    // reloc 位姿(ROS 适配层收到 /relocalized_pose 后调用)。
    // 检查主动请求、测量时间和 Local 位姿同步；完成启动后忽略重定位结果。
    // 通过后进入 RECOVERING，等待 EKF 与 ICP 确认。
    // 拒绝原因写入 reject_reason。
    bool OnRelocPose(const Eigen::Matrix4d& T_map, double t, double now, std::string& reject_reason)
    {
        reject_reason.clear();
        if (InitializationFailed()) {
            reject_reason = "initialization failed; restart required"; return false;
        }
        if (StartupCompleted()) {
            reject_reason = "startup initialization already completed"; return false;
        }
        if ((state_ != State::kRelocalizing && state_ != State::kLost) ||
            !request_issued_) {
            reject_reason = "no active relocalization request";
            return false;
        }
        const bool is_startup = (state_ == State::kRelocalizing && !StartupCompleted());
        const double startup_request_leeway = is_startup ? 3.0 : 0.0;
        const double effective_max_age = is_startup ? 3.5 : cfg_.max_reloc_age;

        if (!T_map.allFinite() || !std::isfinite(t) || t < (last_request_at_ - startup_request_leeway) ||
            now - t > effective_max_age || t - now > cfg_.reloc_sync_tolerance) {
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
        EnterRecovering(T_map, t, "reloc validated; waiting for EKF and ICP");
        return true;
    }

    // ---- 周期驱动(1Hz 由 ROS 适配层调用) ----
    Action Tick(double now)
    {
        Action a;
        if (StartupCompleted() || InitializationFailed()) return a;
        if (state_ == State::kNormalTracking) return a;
        if (state_ == State::kRecovering) return a; // 交接由适配层确认或锁定失败。
        // RELOCALIZING / LOST:按周期请求 SC 重定位
        if (now - last_request_at_ >= cfg_.reloc_request_interval) {
            a.request_reloc = true;
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

    // 使用局部运动推进验证通过的位姿，供 EKF 初始化与输出确认使用。
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

    InitializationStage Stage() const { return stage_; }
    bool StartupCompleted() const { return stage_ == InitializationStage::kComplete; }
    bool InitializationFailed() const { return stage_ == InitializationStage::kFailed; }

    bool MarkSetPoseSent() {
        return Advance(InitializationStage::kReadyToSetPose, InitializationStage::kWaitingForSetPose);
    }
    bool ConfirmSetPose() {
        return Advance(InitializationStage::kWaitingForSetPose, InitializationStage::kWaitingForGlobal);
    }
    bool MarkStartTrackingSent() {
        return Advance(InitializationStage::kWaitingForGlobal, InitializationStage::kWaitingForStartTracking);
    }
    bool RejectStartTracking() {
        // 明确拒绝时保留原策略：等待后续 Global 样本再次确认并尝试放行。
        return Advance(InitializationStage::kWaitingForStartTracking, InitializationStage::kWaitingForGlobal);
    }
    bool ConfirmStartTracking(double stamp) {
        if (!Advance(InitializationStage::kWaitingForStartTracking, InitializationStage::kWaitingForScan))
            return false;
        start_tracking_confirmed_at_ = stamp;
        return true;
    }
    void FailInitialization(const std::string& reason) {
        if (StartupCompleted() || InitializationFailed()) return;
        stage_ = InitializationStage::kFailed;
        state_reason_ = reason;
        state_changed_ = true;
    }

    const std::string& StateReason() const { return state_reason_; }
    State GetState() const { return state_; }
    const StatusInfo& LastStatus() const { return health_.LastStatus(); }
    bool HeartbeatStale() const { return health_.HeartbeatStale(); }

    bool CheckHealth(double now, double timeout)
    {
        if (!StartupCompleted() || !health_.CheckTimeout(now, timeout)) return false;
        if (state_ != State::kLost)
            SetState(State::kLost, "startup-only: ICP continues without SC");
        state_changed_ = true;
        return true;
    }
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
    bool Advance(InitializationStage from, InitializationStage to)
    {
        if (stage_ != from) return false;
        stage_ = to;
        return true;
    }

    // 只记录测量时间用于位姿推进；交接超时由 ROS 适配层持有。
    void EnterRecovering(const Eigen::Matrix4d& T_map_base, double measurement_stamp,
                         const std::string& reason)
    {
        accepted_pose_ = T_map_base;
        accepted_stamp_ = measurement_stamp;
        stage_ = InitializationStage::kReadyToSetPose;
        SetState(State::kRecovering, reason);
    }

    void EnterRelocalizing(double now, const std::string& reason)
    {
        reloc_started_at_ = now;
        last_request_at_ = now - cfg_.reloc_request_interval;
        request_issued_ = false;
        attempts_ = 0;
        SetState(State::kRelocalizing, reason);
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
    InitializationStage stage_ = InitializationStage::kWaitingForReloc;
    std::string state_reason_;

    static constexpr size_t kOdomHistMax = 600; // EKF 30Hz ≈ 20s
    std::deque<std::pair<double, Eigen::Matrix4d>> odom_hist_;

    HealthMonitor health_;

    double accepted_stamp_ = 0.0;
    Eigen::Matrix4d accepted_pose_ = Eigen::Matrix4d::Identity();

    double reloc_started_at_ = 0.0; // 当前 RELOCALIZING 尝试窗口起点
    bool request_issued_ = false;
    double start_tracking_confirmed_at_ = -1.0;
    int attempts_ = 0;              // 连续超时次数
    double last_request_at_ = 0.0;  // 上次请求 reloc 时刻(节流)
};

} // namespace ml_loc_mgr
