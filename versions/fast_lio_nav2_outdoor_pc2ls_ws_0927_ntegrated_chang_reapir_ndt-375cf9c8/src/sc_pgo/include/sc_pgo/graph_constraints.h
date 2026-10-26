#pragma once
#include <gtsam/geometry/Pose3.h>

namespace sc_pgo {
// 因子节点顺序为历史帧、当前帧；测量方向必须与此一致。
inline gtsam::Pose3 loopMeasurement(const gtsam::Pose3& map_history,
                                  const gtsam::Pose3& map_current_corrected) {
    return map_history.between(map_current_corrected);
}
// 初值沿用后端修正；relative_odometry 必须仅由原始前端位姿计算。
inline gtsam::Pose3 propagateInitialGuess(const gtsam::Pose3& map_previous,
                                        const gtsam::Pose3& relative_odometry) {
    return map_previous.compose(relative_odometry);
}
// Pose3 方差顺序为旋转、平移，分别使用 rad² 和 m²。
inline gtsam::Vector6 odometryVariances(double rotation, double translation) {
    gtsam::Vector6 variances;
    variances << rotation, rotation, rotation, translation, translation, translation;
    return variances;
}
}
