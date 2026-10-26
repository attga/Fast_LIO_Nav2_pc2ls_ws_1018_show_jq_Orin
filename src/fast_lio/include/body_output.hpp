#pragma once
#include <Eigen/Geometry>

namespace fast_lio {
using PoseCovariance = Eigen::Matrix<double, 6, 6>;

// T_A_B 将 B 系点变换到 A 系；只转换发布接口，不改变前端状态。
inline Eigen::Isometry3d BasePose(const Eigen::Matrix3d& R_odom_imu,
                                const Eigen::Vector3d& p_odom_imu,
                                const Eigen::Isometry3d& T_base_imu) {
  Eigen::Isometry3d T_odom_imu = Eigen::Isometry3d::Identity();
  T_odom_imu.linear() = R_odom_imu;
  T_odom_imu.translation() = p_odom_imu;
  return T_odom_imu * T_base_imu.inverse();
}

// IKFoM 的姿态误差为 IMU 系右扰动，位置误差位于 odom。
// ROS 输出使用 odom 系固定轴旋转误差；安装偏移使旋转误差也影响底盘位置。
inline PoseCovariance BasePoseJacobian(const Eigen::Matrix3d& R_odom_imu,
                                       const Eigen::Isometry3d& T_base_imu) {
  const Eigen::Vector3d arm = T_base_imu.inverse().translation();
  Eigen::Matrix3d skew;
  skew << 0.0, -arm.z(), arm.y(),
          arm.z(), 0.0, -arm.x(),
          -arm.y(), arm.x(), 0.0;
  PoseCovariance J = PoseCovariance::Identity();
  J.block<3, 3>(0, 3) = -R_odom_imu * skew;
  J.block<3, 3>(3, 3) = R_odom_imu;
  return J;
}
}  // namespace fast_lio
