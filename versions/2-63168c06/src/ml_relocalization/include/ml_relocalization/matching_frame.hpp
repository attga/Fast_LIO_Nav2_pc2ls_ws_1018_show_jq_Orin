#pragma once
#include "ml_relocalization/types.hpp"
#include "ml_relocalization/input_validation.hpp"
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <stdexcept>

namespace ml_relocalization {
// 每帧保存一份安装关系；地图 body 和当前 matching_frame 均指传感器参考系。
// T_A_B 将 B 系点转换到 A 系，不重新解释历史地图为新底盘坐标。
class MatchingFrame {
public:
  explicit MatchingFrame(const Eigen::Matrix4d& T_base_matching = Eigen::Matrix4d::Identity()) {
    if (!ValidMapPose(T_base_matching))
      throw std::invalid_argument("invalid base<-matching installation transform");
    T_matching_base_ = T_base_matching.inverse();
  }

  void convertScan(Cloud& scan_base) const {
    pcl::transformPointCloud(scan_base, scan_base, T_matching_base_);
  }

  Eigen::Matrix4d basePose(const Eigen::Matrix4d& T_map_matching) const {
    return T_map_matching * T_matching_base_;
  }

private:
  Eigen::Matrix4d T_matching_base_;
};
}  // namespace ml_relocalization
