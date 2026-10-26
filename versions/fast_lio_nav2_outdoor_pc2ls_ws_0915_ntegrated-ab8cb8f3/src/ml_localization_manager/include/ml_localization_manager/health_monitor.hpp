#pragma once

namespace ml_loc_mgr
{

// ICP 状态快照；输入/处理超时由适配层转为不可用，不改配准失败计数。
struct IcpStatus
{
    enum class Level { kInit, kLost, kDegraded, kTracking, kUnknown };
    Level level = Level::kUnknown;
    int fail_count = 0;
    double scan_stamp = -1.0;
    bool waiting_for_tf = false;
};

// 仅判断运行健康，不触发 SC、不操作 EKF，也不拥有初始化阶段。
// 接收时间和超时检查均使用调用方提供的 ROS 时间，仿真暂停不累计超时。
class HealthMonitor
{
public:
    enum class Observation { kKeep, kTracking, kLost };

    Observation Observe(const IcpStatus& status, double now)
    {
        if (status.level == IcpStatus::Level::kUnknown) return Observation::kKeep;
        last_status_ = status;
        last_received_at_ = now;
        heartbeat_stale_ = false;
        if (status.level == IcpStatus::Level::kTracking) return Observation::kTracking;
        if (status.level == IcpStatus::Level::kInit && status.waiting_for_tf)
            return Observation::kKeep;
        if (status.level == IcpStatus::Level::kLost || status.level == IcpStatus::Level::kInit)
            return Observation::kLost;
        return Observation::kKeep;
    }

    // 仅在完成初始化后调用；返回本次是否首次检测到超时。
    bool CheckTimeout(double now, double timeout)
    {
        if (heartbeat_stale_ || last_received_at_ < 0.0 || now - last_received_at_ <= timeout)
            return false;
        heartbeat_stale_ = true;
        last_status_.level = IcpStatus::Level::kLost;
        return true;
    }

    bool HeartbeatStale() const { return heartbeat_stale_; }
    const IcpStatus& LastStatus() const { return last_status_; }

private:
    IcpStatus last_status_;
    double last_received_at_ = -1.0;
    bool heartbeat_stale_ = false;
};

} // namespace ml_loc_mgr
