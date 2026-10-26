#include "sc_pgo/graph_constraints.h"
#include <gtsam/slam/BetweenFactor.h>
#include <iostream>

int main() {
    using namespace gtsam;
    const Pose3 history(Rot3::RzRyRx(0.1, -0.2, 0.7), Point3(12, -4, 2));
    const Pose3 current(Rot3::RzRyRx(-0.1, 0.15, -0.5), Point3(15, 3, 1));
    const auto noise = noiseModel::Isotropic::Sigma(6, 1.0);
    const auto measurement = sc_pgo::loopMeasurement(history, current);
    BetweenFactor<Pose3> factor(0, 1, measurement, noise);
    // 正确约束应在已知位姿上为零；逆方向约束必须被测试识别。
    if (factor.evaluateError(history, current).norm() > 1e-10) return 1;
    BetweenFactor<Pose3> reversed(0, 1, measurement.inverse(), noise);
    if (reversed.evaluateError(history, current).norm() < 1.0) return 2;
    const auto variances = sc_pgo::odometryVariances(0.02, 0.3);
    if ((variances.head<3>().array() != 0.02).any() ||
        (variances.tail<3>().array() != 0.3).any()) return 3;
    // 非交换旋转/平移：后端整体修正后，新初值必须延续修正后的轨迹。
    const Pose3 correction(Rot3::RzRyRx(-0.2, 0.3, 1.1), Point3(-7, 5, 0.4));
    const Pose3 delta = history.between(current);
    const Pose3 previousOptimized = correction.compose(history);
    const Pose3 guess = sc_pgo::propagateInitialGuess(previousOptimized, delta);
    if (!guess.equals(correction.compose(current), 1e-9)) return 4;
    BetweenFactor<Pose3> rawFactor(0, 1, delta, noise);
    if (rawFactor.evaluateError(previousOptimized, guess).norm() > 1e-9) return 5;
    // 多个关键帧尚未优化时仍连续递推；不退回原始 odom 世界坐标。
    const Pose3 next(Rot3::RzRyRx(0.2, 0.1, 0.8), Point3(18, 5, 2));
    const auto nextGuess = sc_pgo::propagateInitialGuess(guess, current.between(next));
    if (!nextGuess.equals(correction.compose(next), 1e-9)) return 6;
    if (!sc_pgo::propagateInitialGuess(history, delta).equals(current, 1e-9)) return 7;
    std::cout << "回环方向、方差维度、优化后初值及待优化连续递推测试通过\n";
    return 0;
}
