#pragma once
#include "sc_pgo/common.h"
#include <pcl/point_cloud.h>
#include <algorithm>
#include <optional>
#include <vector>

namespace sc_pgo {
struct LoopFrame {
    pcl::PointCloud<PointType>::ConstPtr cloud;
    Pose6D pose;
};
struct LoopSnapshot {
    Pose6D history_pose, current_pose;
    double scan_time;
    std::vector<LoopFrame> source, target;
};
// 调用方持有关键帧锁。位姿按值复制，入库后不再修改的点云只保留只读引用。
inline std::optional<LoopSnapshot> captureLoopSnapshot(
    const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds,
    const std::vector<Pose6D>& poses, const std::vector<double>& times,
    int history, int current, int source_radius = 5, int target_radius = 25) {
    if (clouds.size() != poses.size() || times.size() != poses.size() ||
        history < 0 || current < 0 || history >= int(poses.size()) ||
        current >= int(poses.size()) || source_radius < 0 || target_radius < 0)
        return std::nullopt;
    LoopSnapshot result{poses[history], poses[current], times[current], {}, {}};
    auto copyWindow = [&](int center, int radius, std::vector<LoopFrame>& out) {
        for (int i = std::max(0, center-radius); i <= std::min(int(poses.size())-1, center+radius); ++i)
            out.push_back({clouds[i], poses[i]});
    };
    copyWindow(current, source_radius, result.source);
    copyWindow(history, target_radius, result.target);
    return result;
}
}
