#include "ml_ndt_localization/tracker.hpp"
#include <pcl/common/point_tests.h>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace ml_ndt_loc
{

Tracker::Tracker(const std::string& map_dir, const Config& cfg)
    : cfg_(cfg), submap_cache_(std::make_unique<SubmapCache>(map_dir + "/map.pcd",
        cfg.voxel_size, cfg.submap_size, cfg.submap_update_margin)),
      matcher_(std::make_unique<NdtMatcher>(cfg.ndt)) {}

Tracker::RegistrationResult Tracker::TrackOnce(const pcl::PointCloud<PointType>::Ptr& scan_body,
    const Eigen::Matrix4d& T_map_base_init)
{
    std::lock_guard<std::mutex> lock(mtx_);
    RegistrationResult result;
    if (lost_ && cfg_.pause_on_lost) {
        result.rejection_reason = "tracker_lost_waiting_reset";
        last_result_ = result;
        return result; // Paused calls do not count as failed registrations.
    }
    if (!scan_body || scan_body->empty())
        return RecordFailure("scan_empty");

    const Eigen::Matrix4d& T_init = T_map_base_init;

    if (!T_init.allFinite()) return RecordFailure("prediction_nonfinite");
    const auto rotation = T_init.block<3,3>(0,0);
    if (!(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-5) ||
        std::abs(rotation.determinant() - 1.0) > 1e-5 ||
        !T_init.row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-8))
        return RecordFailure("prediction_not_rigid");
    for (const auto& p : *scan_body)
        if (!pcl::isFinite(p)) return RecordFailure("source_nonfinite");
    const auto submap_start = std::chrono::steady_clock::now();
    const auto selection = submap_cache_->Select(T_init.block<2,1>(0,3));
    auto submap = selection.cloud;
    result.submap_updated = selection.updated;
    result.window_center_x = selection.center.x();
    result.window_center_y = selection.center.y();
    result.submap_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - submap_start).count();
    result.submap_points = submap->size();
    if (submap->size() < 1000) {
        result.rejection_reason = "submap_small";
        return RecordResult(result);
    }

    if (scan_body->size() < 100) return RecordFailure("source_too_small");
    const auto match = matcher_->Align(scan_body, submap, T_init);
    const Eigen::Matrix4d T_current = match.T_map_base;
    result.registration_ms = match.registration_ms;
    result.target_build_ms = match.target_build_ms;
    result.covariance_xy = match.covariance_xy;
    result.covariance_valid = match.covariance_valid;
    result.covariance_ms = match.covariance_ms;
    result.likelihood = match.likelihood;
    result.transformation_probability = match.transformation_probability;
    result.iterations = match.iterations;
    result.T_map_base = T_current;
    result.converged = match.converged;
    result.inlier_fraction = match.inlier_fraction;
    result.correction_m = (T_current.block<3,1>(0,3) - T_init.block<3,1>(0,3)).norm();
    double yaw_delta = std::fabs(YawOf(T_current) - YawOf(T_init));
    result.correction_yaw_deg = std::min(yaw_delta, 2 * M_PI - yaw_delta) * 180.0 / M_PI;
    EvaluateQuality(result);
    return RecordResult(result);
}

pcl::PointCloud<PointType>::ConstPtr Tracker::GlobalMap() const
{
    return submap_cache_->Map();
}

Tracker::Status Tracker::GetStatus()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return {last_result_, fail_count_, lost_};
}

bool Tracker::IsLost()
{
    std::lock_guard<std::mutex> lock(mtx_);
    return lost_;
}

void Tracker::MarkLost()
{
    std::lock_guard<std::mutex> lock(mtx_);
    lost_ = true;
}

void Tracker::Reset()
{
    std::lock_guard<std::mutex> lock(mtx_);
    fail_count_ = 0;
    lost_ = false;
    last_result_.accepted = false;
    last_result_.rejection_reason.clear();
}

Tracker::RegistrationResult Tracker::RecordResult(const RegistrationResult& result)
{
    fail_count_ = result.accepted ? 0 : fail_count_ + 1;
    if (result.accepted) lost_ = false;
    last_result_ = result;
    return result;
}

Tracker::RegistrationResult Tracker::RecordFailure(const std::string& reason)
{
    RegistrationResult result;
    result.rejection_reason = reason;
    return RecordResult(result);
}

void Tracker::EvaluateQuality(RegistrationResult& result) const
{
    // 单一判定链：保留门限顺序，日志报告首个失败条件。
    result.accepted = false;
    result.rejection_reason.clear();
    if (!result.converged)
        result.rejection_reason = "ndt_iteration_limit_or_invalid_result";
    else if (!result.covariance_valid)
        result.rejection_reason = "covariance_invalid";
    else if (!std::isfinite(result.likelihood))
        result.rejection_reason = "likelihood_nonfinite";
    else if (!(result.likelihood >= cfg_.likelihood_threshold))
        result.rejection_reason = "likelihood=" + std::to_string(result.likelihood);
    else if (!(result.inlier_fraction >= cfg_.inlier_fraction_threshold))
        result.rejection_reason = "inlier_frac=" + std::to_string(result.inlier_fraction);
    else if (!(result.correction_m <= cfg_.max_translation_jump))
        result.rejection_reason = "translation_jump=" + std::to_string(result.correction_m);
    else if (!(result.correction_yaw_deg <= cfg_.max_yaw_jump))
        result.rejection_reason = "yaw_jump=" + std::to_string(result.correction_yaw_deg);
    else if (!result.T_map_base.allFinite())
        result.rejection_reason = "pose_nonfinite";
    else
        result.accepted = true;
    // 使用正向比较的否定式，使 NaN 与不满足门限一样被拒绝。
}

double Tracker::YawOf(const Eigen::Matrix4d& T)
{
    return std::atan2(T(1, 0), T(0, 0));
}

} // namespace ml_ndt_loc
