#pragma once

#include <cmath>
#include <stdexcept>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fast_lio {

// R_base_imu: mounting yaw of the whole MID360 (including its internal IMU).
// This is separate from the factory LiDAR-to-IMU extrinsics. The origin is
// unchanged: this class does not introduce a new IMU-to-chassis lever arm.
class YawMount {
public:
    void setYaw(double radians) {
        if (!std::isfinite(radians)) {
            throw std::invalid_argument("publish.imu_to_base_yaw must be finite (radians)");
        }
        rotation_ = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    Eigen::Vector3d imuPointToBase(const Eigen::Vector3d& point) const {
        return rotation_ * point;
    }

    Eigen::Matrix3d worldFromBase(const Eigen::Matrix3d& world_from_imu) const {
        // T_world_base = T_world_imu * inverse(T_base_imu).
        return world_from_imu * rotation_.transpose();
    }

private:
    Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
};

}  // namespace fast_lio
