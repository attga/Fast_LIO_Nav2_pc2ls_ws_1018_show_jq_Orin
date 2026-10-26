#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ml_scan_context {
using KeyframeId = std::uint64_t;
using Points = std::vector<Eigen::Vector3d>;
using Descriptor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// 此配置仅用于增强极坐标分支；旧SC分箱规则仍由SCManager维护。
struct PolarConfig {
  int rings = 20;
  int sectors = 60;
  double radius = 80.0;
  int minimum_points = 5;
};

struct Alignment {
  double distance = std::numeric_limits<double>::infinity();
  int column_shift = 0;
};

struct Candidate {
  KeyframeId keyframe_id;
  double descriptor_distance;
  // p_keyframe = T_keyframe_query * p_query；仅作几何配准初值。
  Eigen::Isometry3d T_keyframe_query = Eigen::Isometry3d::Identity();
};

struct QueryOptions {
  // 单次查询工作线程数；不修改进程全局OpenMP设置，也不绑定CPU核心。
  int threads = 1;
  std::size_t top_k = 5;
  double maximum_distance = 0.5;
  // 空列表表示不限制；调用者决定时间邻域、区域等筛选策略。
  std::vector<KeyframeId> allowed_keyframes;
};

struct DatabaseConfig {
  PolarConfig polar;
  std::vector<Eigen::Vector2d> viewpoints{
    {0, 0}, {0, -2}, {0, 2}, {-2, -2}, {2, -2}, {-2, 2}, {2, 2}};
  // 标识调用者的坐标系/裁剪/体素/高度处理约定，保存加载时严格核对。
  std::string preprocessing_id;
};
}  // namespace ml_scan_context
