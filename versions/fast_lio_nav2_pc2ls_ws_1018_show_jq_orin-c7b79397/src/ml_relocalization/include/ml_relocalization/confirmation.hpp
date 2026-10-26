#pragma once
#include "ml_relocalization/types.hpp"
#include <cmath>

namespace ml_relocalization {
// Counts consistent geometrically valid results, even across rejected attempts.
// The node serializes this small session state with trigger and publication.
class Confirmation {
public:
  explicit Confirmation(const Config& config) : config_(config) {}
  void reset() { count_ = 0; }
  int observe(const Eigen::Matrix4d& T_map_body) {
    const float yaw = std::atan2(T_map_body(1,0), T_map_body(0,0)) * 180.0 / M_PI;
    const double previous_yaw = std::atan2(previous_(1,0), previous_(0,0)) * 180.0 / M_PI;
    double yaw_difference = std::fabs(yaw - previous_yaw);
    if (yaw_difference > 180.0) yaw_difference = 360.0 - yaw_difference;
    const double distance = (T_map_body.block<3,1>(0,3) - previous_.block<3,1>(0,3)).norm();
    const bool consistent = count_ > 0 && distance < config_.confirm_position_thresh &&
                            yaw_difference < config_.confirm_yaw_thresh_deg;
    count_ = consistent ? count_ + 1 : 1;
    previous_ = T_map_body;
    return count_;
  }
private:
  Config config_;
  int count_ = 0;
  Eigen::Matrix4d previous_ = Eigen::Matrix4d::Identity();
};
}  // namespace ml_relocalization
