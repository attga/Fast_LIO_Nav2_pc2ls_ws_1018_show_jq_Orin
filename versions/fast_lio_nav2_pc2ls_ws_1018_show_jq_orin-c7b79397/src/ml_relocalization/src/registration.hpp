#pragma once
#include "ml_relocalization/types.hpp"

namespace ml_relocalization {
// 输入扫描在 matching_frame，目标在 map；初值/输出均为 T_map_body。
void ValidateRegistrationConfig(const Config& config);
void refine(const Config& config, const Cloud::Ptr& source, const Cloud::Ptr& target,
            const Eigen::Matrix4d& T_map_body_hypothesis, MatchResult& result);
}  // namespace ml_relocalization
