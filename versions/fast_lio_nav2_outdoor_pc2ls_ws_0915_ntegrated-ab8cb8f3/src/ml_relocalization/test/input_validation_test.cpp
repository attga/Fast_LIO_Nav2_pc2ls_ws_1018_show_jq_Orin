#include <gtest/gtest.h>
#include "ml_relocalization/input_validation.hpp"
#include <Eigen/Geometry>
#include <limits>
namespace ml_relocalization {
TEST(InputValidation, ScanTimeFields) {
  EXPECT_FALSE(ValidScanStamp(0,0));
  EXPECT_FALSE(ValidScanStamp(-1,500));
  EXPECT_FALSE(ValidScanStamp(1,1000000000u));
  EXPECT_TRUE(ValidScanStamp(0,1));
  EXPECT_TRUE(ValidScanStamp(10,999999999));
}
TEST(InputValidation, RemovesInvalidXyzEvenWhenDense) {
  Cloud cloud; Point point; point.x=1;point.y=2;point.z=3;point.intensity=7;
  cloud.push_back(point);
  point.x=std::numeric_limits<float>::quiet_NaN();cloud.push_back(point);
  point.x=1;point.z=std::numeric_limits<float>::infinity();cloud.push_back(point);
  cloud.is_dense=true;
  EXPECT_EQ(RemoveNonfinitePoints(cloud),2u);
  ASSERT_EQ(cloud.size(),1u);EXPECT_FLOAT_EQ(cloud[0].intensity,7);
  EXPECT_EQ(cloud.width,1u);EXPECT_EQ(cloud.height,1u);
  EXPECT_EQ(RemoveNonfinitePoints(cloud),0u);
}
TEST(InputValidation, EmptyOrEntirelyInvalidScan) {
  Cloud cloud;EXPECT_EQ(RemoveNonfinitePoints(cloud),0u);
  Point p;p.x=p.y=p.z=std::numeric_limits<float>::quiet_NaN();cloud.push_back(p);
  EXPECT_EQ(RemoveNonfinitePoints(cloud),1u);EXPECT_TRUE(cloud.empty());
}
TEST(InputValidation, RejectsScaleReflectionAndNonfinitePose) {
  auto pose=Eigen::Matrix4d::Identity().eval();EXPECT_TRUE(ValidMapPose(pose));
  pose(0,0)=2;EXPECT_FALSE(ValidMapPose(pose));
  pose(0,0)=-1;EXPECT_FALSE(ValidMapPose(pose));
  pose.setIdentity();pose(0,3)=std::numeric_limits<double>::infinity();EXPECT_FALSE(ValidMapPose(pose));
}
TEST(InputValidation, AcceptsRigidMotionAndTextRounding) {
  auto pose=Eigen::Matrix4d::Identity().eval();
  pose.block<3,3>(0,0)=Eigen::AngleAxisd(0.8,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose(0,3)=123;pose(1,3)=-24;pose(0,0)+=1e-6;
  EXPECT_TRUE(ValidMapPose(pose));
}
} // namespace ml_relocalization
