#include "ml_relocalization/keyframe_map.hpp"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ml_relocalization {
KeyframeMap::KeyframeMap(const Config& config) : directory_(config.map_directory) {
  const auto path = directory_ + "/" + config.pose_file;
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open pose file: " + path);
  std::string line;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) continue;
    std::istringstream values(line);
    Eigen::Matrix4d T_map_body = Eigen::Matrix4d::Identity();
    // KITTI stores a row-major 3x4 matrix, including translation.
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col)
        if (!(values >> T_map_body(row, col)))
          throw std::runtime_error("bad pose line " + std::to_string(line_number));
    poses_.push_back(T_map_body);
  }
  if (poses_.empty()) throw std::runtime_error("empty pose database: " + path);
  for (std::size_t index = 0; index < poses_.size(); ++index)
    if (!std::ifstream(scanPath(index)).good())
      throw std::runtime_error("missing keyframe: " + scanPath(index));
}

std::string KeyframeMap::scanPath(int index) const {
  std::ostringstream path;
  path << directory_ << "/Scans/" << std::setw(6) << std::setfill('0') << index << ".pcd";
  return path.str();
}

Cloud::Ptr KeyframeMap::cloud(int index) {
  const auto found = cache_.find(index);
  if (found != cache_.end()) return found->second;
  // Preserve the original bounded lazy cache policy during extraction.
  if (cache_.size() > 50) cache_.clear();
  auto points = std::make_shared<Cloud>();
  if (pcl::io::loadPCDFile<Point>(scanPath(index), *points) == -1) return nullptr;
  cache_[index] = points;
  return points;
}

Cloud::Ptr KeyframeMap::submap(int center, int halfwidth) {
  auto result = std::make_shared<Cloud>();
  const int first = std::max(0, center - halfwidth);
  const int last = std::min(static_cast<int>(poses_.size()) - 1, center + halfwidth);
  for (int index = first; index <= last; ++index) {
    auto points = cloud(index);
    if (!points) continue;
    Cloud transformed;
    pcl::transformPointCloud(*points, transformed, poses_[index]);
    *result += transformed;
  }
  return result;
}
}  // namespace ml_relocalization
