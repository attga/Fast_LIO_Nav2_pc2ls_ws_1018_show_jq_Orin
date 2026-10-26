#pragma once
#include "ml_relocalization/types.hpp"
#include <Eigen/LU>
#include <algorithm>
#include <cstdint>
#include <cmath>

namespace ml_relocalization {
// 先验证 ROS 时间字段，避免将零时间当作有效测量或构造非法 rclcpp::Time。
inline bool ValidScanStamp(int32_t sec, uint32_t nanosec) {
  return sec >= 0 && nanosec < 1000000000u && (sec > 0 || nanosec > 0);
}

// 返回拒绝原因；任务时间与采集时间均为 ROS 时间，接收时间不能替代采集时间。
inline const char* ScanTimeRejection(double stamp, double now, double requested,
                                     double max_age, double future_tolerance) {
  if (!std::isfinite(stamp) || !std::isfinite(now) || stamp <= 0.0) return "invalid_time";
  if (stamp < requested) return "pre_request_scan";
  if (now - stamp > max_age) return "stale_scan";
  if (stamp - now > future_tolerance) return "future_scan";
  return nullptr;
}

// 不依赖 is_dense 标记；只检查几何 XYZ，不改变强度值或有效点顺序。
inline std::size_t RemoveNonfinitePoints(Cloud& cloud) {
  const auto size = cloud.size();
  cloud.points.erase(std::remove_if(cloud.points.begin(), cloud.points.end(), [](const Point& p) {
    return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
  }), cloud.points.end());
  if (cloud.size() != size) {
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
  }
  return size - cloud.size();
}

inline bool ValidMapPose(const Eigen::Matrix4d& pose) {
  if (!pose.allFinite()) return false;
  constexpr double tolerance = 1e-3; // 容纳文本导出的舍入，不接受缩放、反射或明显非正交旋转。
  const auto rotation = pose.block<3,3>(0,0);
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <= tolerance &&
         std::abs(rotation.determinant() - 1.0) <= tolerance &&
         (pose.row(3) - Eigen::RowVector4d(0,0,0,1)).cwiseAbs().maxCoeff() <= tolerance;
}
} // namespace ml_relocalization
