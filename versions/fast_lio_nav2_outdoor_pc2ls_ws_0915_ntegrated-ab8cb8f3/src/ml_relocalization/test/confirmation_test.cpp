#include <gtest/gtest.h>
#include "ml_relocalization/confirmation.hpp"
#include <Eigen/Geometry>

namespace ml_relocalization {
namespace {
Eigen::Matrix4d Pose(double x, double yaw_degrees = 0.0) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 3) = x;
  pose.block<3,3>(0,0) = Eigen::AngleAxisd(
      yaw_degrees * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

TEST(ConfirmationTest, CountsConsistentValidResults) {
  Config config;
  Confirmation confirmation(config);
  EXPECT_EQ(confirmation.observe(Pose(0.0)), 1);
  EXPECT_EQ(confirmation.observe(Pose(0.1)), 2);
  EXPECT_EQ(confirmation.observe(Pose(0.2)), 3);
}

TEST(ConfirmationTest, TriggerResetRestartsAtOne) {
  Config config;
  Confirmation confirmation(config);
  confirmation.observe(Pose(0.0));
  confirmation.observe(Pose(0.1));
  confirmation.reset();
  EXPECT_EQ(confirmation.observe(Pose(0.2)), 1);
}

TEST(ConfirmationTest, PositionBoundaryRestartsAndUsesNewReference) {
  Config config;
  config.confirm_position_thresh = 2.0;
  Confirmation confirmation(config);
  EXPECT_EQ(confirmation.observe(Pose(0.0)), 1);
  // 现有阈值为严格小于，边界点应重置；后续比较新参考位姿。
  EXPECT_EQ(confirmation.observe(Pose(2.0)), 1);
  EXPECT_EQ(confirmation.observe(Pose(2.1)), 2);
}

TEST(ConfirmationTest, YawWrapIsConsistentButLargeTurnRestarts) {
  Config config;
  config.confirm_yaw_thresh_deg = 15.0;
  Confirmation confirmation(config);
  EXPECT_EQ(confirmation.observe(Pose(0.0, 179.0)), 1);
  EXPECT_EQ(confirmation.observe(Pose(0.0, -179.0)), 2);
  EXPECT_EQ(confirmation.observe(Pose(0.0, -150.0)), 1);
}
} // namespace
} // namespace ml_relocalization
