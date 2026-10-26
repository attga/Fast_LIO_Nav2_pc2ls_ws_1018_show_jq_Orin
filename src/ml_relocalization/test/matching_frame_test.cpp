#include <gtest/gtest.h>
#include "ml_relocalization/matching_frame.hpp"
#include "ml_relocalization/confirmation.hpp"

namespace ml_relocalization {
namespace {
Eigen::Matrix4d Mount() {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.block<3,1>(0,3) = Eigen::Vector3d(1., .2, .8);
  return transform;
}

TEST(MatchingFrame, IdentityPreservesLegacyInputAndPose) {
  MatchingFrame frame;
  Cloud scan; Point p; p.x=2.;p.y=-3.;p.z=.5;p.intensity=9.;scan.push_back(p);
  frame.convertScan(scan);
  EXPECT_FLOAT_EQ(scan[0].x,2.);EXPECT_FLOAT_EQ(scan[0].y,-3.);
  EXPECT_FLOAT_EQ(scan[0].z,.5);EXPECT_FLOAT_EQ(scan[0].intensity,9.);
  EXPECT_TRUE(frame.basePose(Mount()).isApprox(Mount(),1e-12));
}

TEST(MatchingFrame, RestoresSensorCentredCloudAndConvertsResultToRearAxle) {
  MatchingFrame frame(Mount());
  // 传感器点 (2,0,0) 经 +90° 和安装平移变为底盘点 (1,2.2,0.8)。
  Cloud scan;Point p;p.x=1.;p.y=2.2;p.z=.8;p.intensity=8.;scan.push_back(p);
  frame.convertScan(scan);
  EXPECT_NEAR(scan[0].x,2.,1e-6);EXPECT_NEAR(scan[0].y,0.,1e-6);EXPECT_NEAR(scan[0].z,0.,1e-6);
  EXPECT_FLOAT_EQ(scan[0].intensity,8.);
  // 地图中的底盘位于 (4,5,0) 且朝前。
  Eigen::Matrix4d sensor = Mount();sensor(0,3)+=4.;sensor(1,3)+=5.;
  const auto base = frame.basePose(sensor);
  EXPECT_TRUE((base.block<3,3>(0,0).isApprox(Eigen::Matrix3d::Identity(),1e-12)));
  EXPECT_TRUE((base.block<3,1>(0,3).isApprox(Eigen::Vector3d(4.,5.,0.),1e-12)));
}

TEST(MatchingFrame, InvalidMountIsRejected) {
  auto invalid=Mount();invalid(0,0)=2.;
  EXPECT_THROW(MatchingFrame{invalid},std::invalid_argument);
  invalid=Mount();invalid(0,3)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MatchingFrame{invalid},std::invalid_argument);
}

TEST(MatchingFrame, SnapshotCopyRetainsItsOwnInstallation) {
  MatchingFrame current(Mount());
  const auto snapshot=current;
  current=MatchingFrame{};
  EXPECT_TRUE(snapshot.basePose(Mount()).isApprox(Eigen::Matrix4d::Identity(),1e-12));
  EXPECT_TRUE(current.basePose(Mount()).isApprox(Mount(),1e-12));
}

TEST(MatchingFrame, ConfirmationUsesRearAxleRatherThanSensorArc) {
  Config config;config.confirm_position_thresh=.1;config.confirm_yaw_thresh_deg=15.;
  Confirmation confirmation(config); MatchingFrame frame(Mount());
  EXPECT_EQ(confirmation.observe(frame.basePose(Mount())),1);
  Eigen::Matrix4d turn=Eigen::Matrix4d::Identity();
  turn.block<3,3>(0,0)=Eigen::AngleAxisd(.15,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // 后轴原地转动，传感器绕行超过 0.1m；转换后后轴位置应保持不动。
  EXPECT_GT(((turn*Mount()).block<3,1>(0,3)-Mount().block<3,1>(0,3)).norm(),.1);
  EXPECT_EQ(confirmation.observe(frame.basePose(turn*Mount())),2);
}
}  // namespace
}  // namespace ml_relocalization
