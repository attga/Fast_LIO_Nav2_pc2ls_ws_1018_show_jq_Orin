#include <gtest/gtest.h>
#include "body_output.hpp"

namespace {
Eigen::Matrix3d Rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

TEST(BodyOutput, IdentityMountPreservesLegacyPoseAndCovariance) {
  const auto R = Rotation(.1, -.2, .4);
  const Eigen::Vector3d position(1., 2., 3.);
  const auto mount = Eigen::Isometry3d::Identity();
  const auto pose = fast_lio::BasePose(R, position, mount);
  EXPECT_TRUE(pose.linear().isApprox(R, 1e-12));
  EXPECT_TRUE(pose.translation().isApprox(position, 1e-12));
  fast_lio::PoseCovariance legacy = fast_lio::PoseCovariance::Identity();
  legacy.block<3,3>(3,3) = R;
  EXPECT_TRUE(fast_lio::BasePoseJacobian(R, mount).isApprox(legacy, 1e-12));
}

TEST(BodyOutput, LeftFacingSensorProducesForwardFacingBase) {
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.linear() = Rotation(0, 0, M_PI/2);
  mount.translation() = Eigen::Vector3d(1., .2, .8);
  // 底盘位于 (4,5,0) 且朝前，传感器位置偏移并朝左。
  const auto pose = fast_lio::BasePose(mount.linear(), Eigen::Vector3d(5.,5.2,.8), mount);
  EXPECT_TRUE(pose.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(4.,5.,0.), 1e-12));
  const Eigen::Vector3d point_base = mount * Eigen::Vector3d(2.,0.,0.);
  EXPECT_TRUE(point_base.isApprox(Eigen::Vector3d(1.,2.2,.8), 1e-12));
  EXPECT_TRUE((pose * point_base).isApprox(Eigen::Vector3d(5.,7.2,.8), 1e-12));
}

TEST(BodyOutput, RotationAboutRearAxleDoesNotBecomeTranslation) {
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.linear() = Rotation(0, 0, M_PI/2);
  mount.translation() = Eigen::Vector3d(1.,.2,.8);
  for (double yaw : {0., .3, 1.5, -2.}) {
    const auto R = Rotation(0,0,yaw);
    const auto pose = fast_lio::BasePose(R * mount.linear(), R * mount.translation(), mount);
    EXPECT_LT(pose.translation().norm(), 1e-12);
    EXPECT_TRUE(pose.linear().isApprox(R, 1e-12));
  }
}

TEST(BodyOutput, JacobianMatchesFiniteDifferencesWithNonzeroLeverArm) {
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.linear() = Rotation(.15,-.2,1.4);
  mount.translation() = Eigen::Vector3d(1.2,-.3,.8);
  const auto R = Rotation(-.1,.25,-.8);
  const Eigen::Vector3d position(3.,4.,.5);
  const auto nominal = fast_lio::BasePose(R, position, mount);
  fast_lio::PoseCovariance numeric;
  const double eps = 1e-6;
  for (int col=0; col<6; ++col) {
    Eigen::Vector3d p = position;
    Eigen::Matrix3d perturbed_R = R;
    if (col<3) p(col) += eps;
    else perturbed_R = R * Eigen::AngleAxisd(eps, Eigen::Vector3d::Unit(col-3)).toRotationMatrix();
    const auto shifted = fast_lio::BasePose(perturbed_R, p, mount);
    numeric.block<3,1>(0,col) = (shifted.translation() - nominal.translation()) / eps;
    const Eigen::AngleAxisd rotation_error(shifted.linear() * nominal.linear().transpose());
    numeric.block<3,1>(3,col) = rotation_error.axis() * rotation_error.angle() / eps;
  }
  EXPECT_TRUE(numeric.isApprox(fast_lio::BasePoseJacobian(R, mount), 2e-6));
}

TEST(BodyOutput, YawUncertaintyCreatesLateralPositionUncertainty) {
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.translation().x() = 2.;
  const auto J = fast_lio::BasePoseJacobian(Eigen::Matrix3d::Identity(), mount);
  fast_lio::PoseCovariance P = fast_lio::PoseCovariance::Zero();
  P(5,5) = .04;
  const fast_lio::PoseCovariance converted = J * P * J.transpose();
  EXPECT_NEAR(converted(1,1), .16, 1e-12);
  EXPECT_NEAR(converted(1,5), -.08, 1e-12);
  EXPECT_TRUE(converted.isApprox(converted.transpose(), 1e-12));
}
}  // namespace
