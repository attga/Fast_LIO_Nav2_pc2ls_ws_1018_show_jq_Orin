#pragma once
#include "ml_relocalization/types.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace ml_relocalization {
// 输入仅包含通过几何门控的结果；空集合由调用者保留原拒绝原因。
inline MatchResult SelectPolarCandidate(const std::vector<MatchResult>& accepted, const Config& config) {
  if (accepted.empty()) { MatchResult result; result.reason = "polar_no_valid_candidate"; return result; }
  auto result = *std::max_element(accepted.begin(), accepted.end(), [](const auto& a, const auto& b) {
    return a.likelihood < b.likelihood;
  });
  for (std::size_t i = 0; i < accepted.size(); ++i) {
    for (std::size_t j = i + 1; j < accepted.size(); ++j) {
      const auto& a = accepted[i].T_map_body;
      const auto& b = accepted[j].T_map_body;
      const double yaw_a = std::atan2(a(1,0), a(0,0));
      const double yaw_b = std::atan2(b(1,0), b(0,0));
      const double yaw = std::abs(std::remainder(yaw_a - yaw_b, 2.0*M_PI))*180.0/M_PI;
      if ((a.block<3,1>(0,3) - b.block<3,1>(0,3)).norm() >= config.confirm_position_thresh ||
          yaw >= config.confirm_yaw_thresh_deg) {
        result.valid = false;
        result.reason = "polar_ambiguous_candidates";
      }
    }
  }
  return result;
}
}
