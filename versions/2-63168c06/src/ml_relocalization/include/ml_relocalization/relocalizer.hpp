#pragma once
#include "ml_relocalization/types.hpp"
#include <memory>

namespace ml_relocalization {
// 单工作线程算法。配准点云保持原有滤波；新描述子另接同帧原始点云。
// 不持有里程计、TF、EKF、触发或系统恢复状态。
class Relocalizer {
public:
  explicit Relocalizer(const Config& config);
  ~Relocalizer();
  // scan_body沿用原NDT输入；polar必须另传未做描述子下采样的同帧点云。
  MatchResult match(const Cloud::Ptr& scan_body, const Cloud::Ptr& descriptor_scan = nullptr);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace ml_relocalization
