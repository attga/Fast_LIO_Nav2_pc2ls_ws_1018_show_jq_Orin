#include "ml_icp_localization/tracker.hpp"
#include <pcl/common/transforms.h>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace ml_icp_loc
{

Tracker::Tracker(const std::string& map_dir,
                 const std::vector<Eigen::Matrix4d>& poses,
                 const Config& cfg)
    : poses_(poses), cfg_(cfg), cache_(map_dir)
{
    if (cfg_.submap_strategy == "window") {
        submap_cache_ = std::make_unique<SubmapCache>(map_dir + "/map.pcd", cfg_.icp_ds,
            cfg_.submap_size, cfg_.submap_update_margin);
    } else if (cfg_.submap_strategy != "keyframes") {
        throw std::invalid_argument("submap_strategy must be keyframes or window");
    }
    matcher_ = std::make_unique<ScanMatcher>(cfg_.registration_backend,
        cfg_.registration_threads, cfg_.correspondence_distance,
        bool(submap_cache_), GlobalMap());
}

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
    const auto submap_start = std::chrono::steady_clock::now();
    pcl::PointCloud<PointType>::Ptr submap;
    if (submap_cache_) {
        auto selection = submap_cache_->Select(T_init.block<2,1>(0,3));
        submap = selection.cloud;
        result.submap_updated = selection.updated;
        result.window_center_x = selection.center.x();
        result.window_center_y = selection.center.y();
    } else {
        int center_idx = NearestKeyframeIdx(poses_, T_init);
        if (center_idx < 0) return RecordFailure("no_nearest_kf");
        submap = BuildSubmapAround(cache_, poses_, center_idx, cfg_.submap_halfwidth, cfg_.icp_ds);
        result.submap_updated = true;
    }
    result.submap_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - submap_start).count();
    result.submap_points = submap->size();
    if (submap->size() < 1000) {
        result.rejection_reason = "submap_small";
        return RecordResult(result);
    }

    // Transform the body scan into map coordinates before registration.
    pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*scan_body, *src, T_init);
    // Reject sparse scans before covariance estimation (20 neighbors).
    if (src->size() < 100) {
        return RecordFailure("source_too_small");
    }

    static const std::vector<size_t> no_map_indices;
    const auto match = matcher_->Align(src, submap,
        submap_cache_ ? submap_cache_->Indices() : no_map_indices);
    const Eigen::Matrix4d T_current = match.T_target_source * T_init;
    result.registration_ms = match.registration_ms;
    result.fitness = match.fitness;
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
    return submap_cache_ ? submap_cache_->Map() : nullptr;
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
    // Preserve gate order: logs report the first failed condition.
    if (!result.converged)
        result.rejection_reason = "icp_not_converged";
    else if (!std::isfinite(result.fitness))
        result.rejection_reason = "fitness_nonfinite";
    else if (result.fitness > cfg_.fitness_threshold)
        result.rejection_reason = "fitness=" + std::to_string(result.fitness);
    else if (result.inlier_fraction < cfg_.inlier_fraction_threshold)
        result.rejection_reason = "inlier_frac=" + std::to_string(result.inlier_fraction);
    else if (result.correction_m > cfg_.max_translation_jump)
        result.rejection_reason = "translation_jump=" + std::to_string(result.correction_m);
    else if (result.correction_yaw_deg > cfg_.max_yaw_jump)
        result.rejection_reason = "yaw_jump=" + std::to_string(result.correction_yaw_deg);
    else if (!result.T_map_base.allFinite())
        result.rejection_reason = "pose_nonfinite";
    // Keep the positive checks, including their NaN rejection semantics.
    result.accepted = result.converged && std::isfinite(result.fitness) &&
        result.fitness <= cfg_.fitness_threshold &&
        result.inlier_fraction >= cfg_.inlier_fraction_threshold &&
        result.correction_m <= cfg_.max_translation_jump &&
        result.correction_yaw_deg <= cfg_.max_yaw_jump && result.T_map_base.allFinite();
}

double Tracker::YawOf(const Eigen::Matrix4d& T)
{
    return std::atan2(T(1, 0), T(0, 0));
}

} // namespace ml_icp_loc
