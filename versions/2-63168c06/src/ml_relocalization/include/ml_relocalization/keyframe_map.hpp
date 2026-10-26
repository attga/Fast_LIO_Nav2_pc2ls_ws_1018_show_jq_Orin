#pragma once

#include "ml_relocalization/types.hpp"
#include <map>
#include <vector>

namespace ml_relocalization {
// Owned exclusively by the matching worker; no ROS or localization state.
class KeyframeMap {
public:
  explicit KeyframeMap(const Config& config);
  std::size_t size() const { return poses_.size(); }
  const Eigen::Matrix4d& pose(int index) const { return poses_.at(index); }
  Cloud::Ptr submap(int center, int halfwidth);
private:
  std::string scanPath(int index) const;
  Cloud::Ptr cloud(int index);
  std::string directory_;
  std::vector<Eigen::Matrix4d> poses_;
  std::map<int, Cloud::Ptr> cache_;
};
}  // namespace ml_relocalization
