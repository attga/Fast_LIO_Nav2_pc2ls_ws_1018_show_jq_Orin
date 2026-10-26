#pragma once
#include "ml_relocalization/types.hpp"

namespace ml_relocalization {
// Source and target are in map. ICP correction left-multiplies the hypothesis.
void refine(const Config& config, const Cloud::Ptr& source, const Cloud::Ptr& target,
            const Eigen::Matrix4d& T_map_body_hypothesis, MatchResult& result);
}  // namespace ml_relocalization
