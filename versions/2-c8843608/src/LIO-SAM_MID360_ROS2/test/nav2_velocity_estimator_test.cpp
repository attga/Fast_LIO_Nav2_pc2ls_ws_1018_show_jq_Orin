#include "nav2_velocity_estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

static void check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << std::endl;
        std::exit(1);
    }
}

static Eigen::Isometry3d pose(double x, double y, double yaw) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translate(Eigen::Vector3d(x, y, 0.0));
    result.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return result;
}

int main() {
    Nav2VelocityEstimator estimator({});
    Eigen::Vector3d twist;
    for (int i = 0; i <= 800; ++i) {
        const double t = i * 0.005;
        // A 2 cm scan correction appears at 2 s, but is not a true speed step.
        twist = estimator.update(t, pose(0.5 * t + (t >= 2.0 ? 0.02 : 0.0), 0.0, 0.0));
        if (i > 400) check(std::abs(twist.x() - 0.5) < 0.15, "scan correction changed speed too much");
        check(std::abs(twist.x()) <= 1.2, "linear speed bound violated");
    }
    check(std::abs(twist.x() - 0.5) < 0.03, "steady forward speed wrong");

    double previous = twist.x();
    for (int i = 1; i <= 400; ++i) {
        const double t = 4.0 + i * 0.005;
        twist = estimator.update(t, pose(2.02 - 0.5 * (t - 4.0), 0.0, 0.0));
        check(twist.x() >= previous - 0.0051, "reverse transition exceeded acceleration limit");
        previous = twist.x();
    }
    check(twist.x() < -0.4, "real reverse motion was suppressed");

    estimator.reset();
    for (int i = 0; i <= 600; ++i) {
        const double t = i * 0.005;
        twist = estimator.update(t, pose(0.002 * std::sin(10.0 * t), 0.0, 0.0));
    }
    check(std::abs(twist.x()) < 0.03, "stationary pose noise became false motion");

    estimator.reset();
    for (int i = 0; i <= 400; ++i) {
        const double t = i * 0.005;
        twist = estimator.update(t, pose(0.03 * std::sin(3.0 * t), 0.0, 0.0), 0.0);
    }
    check(twist.x() == 0.0, "wheel stop did not suppress scan-induced reverse speed");
    for (int i = 401; i <= 800; ++i) {
        const double t = i * 0.005;
        twist = estimator.update(t, pose(0.0, 0.0, 0.0), 0.3);
    }
    check(std::abs(twist.x() - 0.28) < 0.02, "wheel forward speed was not used");

    estimator.reset();
    for (int i = 0; i <= 400; ++i) {
        const double t = i * 0.005;
        twist = estimator.update(t, pose(0.0, 0.3 * t, 3.0 + 0.5 * t));
    }
    check(std::abs(twist.z() - 0.5) < 0.04, "yaw wrap produced angular spike");
    check(std::abs(twist.z()) <= 2.0, "angular speed bound violated");
    twist = estimator.update(0.0, pose(0.0, 0.0, 0.0));
    check(twist.norm() == 0.0, "time rewind did not reset velocity");
    return 0;
}
