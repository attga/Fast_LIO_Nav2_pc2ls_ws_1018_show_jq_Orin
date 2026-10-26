#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

// Estimate the planar base_link twist from the same continuous odom pose used
// for TF. GTSAM's velocity state can change abruptly when a scan correction
// resets IMU propagation, even though the published odom pose stays smooth.
class Nav2VelocityEstimator {
 public:
    struct Limits {
        double window = 0.25;              // seconds of pose history for linear regression
        double time_constant = 0.08;       // seconds, additional velocity low-pass
        double max_linear_speed = 1.2;    // m/s
        double max_angular_speed = 2.0;   // rad/s
        double max_linear_accel = 1.0;    // m/s^2
        double max_angular_accel = 3.0;   // rad/s^2
        double linear_deadband = 0.02;    // m/s
        double angular_deadband = 0.03;   // rad/s
    };

    explicit Nav2VelocityEstimator(const Limits& limits) : limits_(limits) {}

    // Returns [vx, vy, wz] in base_link. Invalid or discontinuous time resets
    // history so bag replay and sensor restarts cannot create a velocity spike.
    Eigen::Vector3d update(double stamp, const Eigen::Isometry3d& odom_to_base,
                           std::optional<double> wheel_linear_x = std::nullopt) {
        if (!std::isfinite(stamp) || !odom_to_base.matrix().allFinite()) {
            reset();
            return velocity_;
        }
        if (!samples_.empty()) {
            const double dt = stamp - samples_.back().stamp;
            if (dt <= 0.0 || dt > 0.5) {
                reset();
            }
        }

        const double yaw = std::atan2(odom_to_base.linear()(1, 0), odom_to_base.linear()(0, 0));
        double unwrapped_yaw = yaw;
        if (!samples_.empty()) {
            double delta = yaw - last_yaw_;
            delta = std::atan2(std::sin(delta), std::cos(delta));
            unwrapped_yaw = samples_.back().yaw + delta;
        }
        const double dt = samples_.empty() ? 0.0 : stamp - samples_.back().stamp;
        samples_.push_back({stamp, odom_to_base.translation().x(),
                            odom_to_base.translation().y(), unwrapped_yaw});
        last_yaw_ = yaw;
        while (samples_.size() > 2 && samples_[1].stamp <= stamp - limits_.window) {
            samples_.pop_front();
        }
        if (samples_.size() < 2 || stamp - samples_.front().stamp < 0.1) {
            return velocity_;
        }

        // Regression over the high-rate pose history suppresses scan-boundary
        // centimetre-scale corrections without differentiating adjacent samples.
        double mean_t = 0.0, mean_x = 0.0, mean_y = 0.0, mean_yaw = 0.0;
        for (const auto& s : samples_) {
            mean_t += s.stamp - stamp;
            mean_x += s.x;
            mean_y += s.y;
            mean_yaw += s.yaw;
        }
        const double n = static_cast<double>(samples_.size());
        mean_t /= n;
        mean_x /= n;
        mean_y /= n;
        mean_yaw /= n;
        double tt = 0.0, tx = 0.0, ty = 0.0, tw = 0.0;
        for (const auto& s : samples_) {
            const double centered_t = s.stamp - stamp - mean_t;
            tt += centered_t * centered_t;
            tx += centered_t * (s.x - mean_x);
            ty += centered_t * (s.y - mean_y);
            tw += centered_t * (s.yaw - mean_yaw);
        }
        if (tt <= 1e-9) return velocity_;

        const double world_vx = tx / tt;
        const double world_vy = ty / tt;
        Eigen::Vector3d target(std::cos(yaw) * world_vx + std::sin(yaw) * world_vy,
                               -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy,
                               tw / tt);
        // On a nonholonomic base, a fresh wheel speed is a better longitudinal
        // feedback source than centimetre-scale scan correction derivatives.
        if (wheel_linear_x && std::isfinite(*wheel_linear_x)) {
            target.x() = *wheel_linear_x;
            target.y() = 0.0;
        }
        if (!target.allFinite()) {
            reset();
            return velocity_;
        }
        const double speed = target.head<2>().norm();
        if (speed > limits_.max_linear_speed)
            target.head<2>() *= limits_.max_linear_speed / speed;
        target.z() = std::clamp(target.z(), -limits_.max_angular_speed, limits_.max_angular_speed);

        const double alpha = dt / (limits_.time_constant + dt);
        target = velocity_ + alpha * (target - velocity_);
        Eigen::Vector2d dv = target.head<2>() - velocity_.head<2>();
        const double max_dv = limits_.max_linear_accel * dt;
        if (dv.norm() > max_dv) dv *= max_dv / dv.norm();
        velocity_.head<2>() += dv;
        velocity_.z() += std::clamp(target.z() - velocity_.z(),
                                    -limits_.max_angular_accel * dt,
                                    limits_.max_angular_accel * dt);
        // Continuous (soft) deadband preserves the acceleration bound at the
        // threshold. A hard zero-to-0.02 m/s switch creates a 4 m/s^2 step
        // at 200 Hz despite the internal 1 m/s^2 acceleration limiter.
        Eigen::Vector3d output = velocity_;
        auto softDeadband = [](double value, double deadband) {
            return std::copysign(std::max(0.0, std::abs(value) - deadband), value);
        };
        output.x() = softDeadband(output.x(), limits_.linear_deadband);
        output.y() = softDeadband(output.y(), limits_.linear_deadband);
        output.z() = softDeadband(output.z(), limits_.angular_deadband);
        return output;
    }

    void reset() {
        samples_.clear();
        velocity_.setZero();
        last_yaw_ = 0.0;
    }

 private:
    struct Sample {
        double stamp;
        double x;
        double y;
        double yaw;
    };
    Limits limits_;
    std::deque<Sample> samples_;
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    double last_yaw_ = 0.0;
};
