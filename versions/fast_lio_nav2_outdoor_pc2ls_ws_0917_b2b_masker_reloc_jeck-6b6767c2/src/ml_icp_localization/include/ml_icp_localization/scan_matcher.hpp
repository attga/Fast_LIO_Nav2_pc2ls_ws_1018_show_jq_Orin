#pragma once
#include "ml_icp_localization/point_types.hpp"
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace ml_icp_loc
{
// Registration backend and its target caches. Inputs and aligned output are in
// map coordinates; T_target_source is the correction applied to the prediction.
// Owns no robot pose, quality policy or tracking state. Caller serializes access.
class ScanMatcher
{
public:
    struct Result {
        Eigen::Matrix4d T_target_source = Eigen::Matrix4d::Identity();
        bool converged = false;
        double fitness = 0.0;
        double inlier_fraction = 0.0;
        double registration_ms = 0.0;
    };

    ScanMatcher(const std::string& backend, int threads, double correspondence_distance,
                bool cache_target, const PointCloud::ConstPtr& map = nullptr);
    ~ScanMatcher();
    Result Align(const PointCloud::Ptr& source, const PointCloud::Ptr& target,
                 const std::vector<size_t>& map_indices);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace ml_icp_loc
