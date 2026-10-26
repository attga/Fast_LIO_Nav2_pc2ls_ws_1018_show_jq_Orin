#pragma once
#include "ml_relocalization/types.hpp"
#include <memory>

namespace ml_relocalization {
// Single-worker algorithm. Input is already SC-downsampled body cloud.
// No odometry, TF, EKF, trigger or system recovery state lives here.
class Relocalizer {
public:
  explicit Relocalizer(const Config& config);
  ~Relocalizer();
  MatchResult match(const Cloud::Ptr& scan_body);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace ml_relocalization
