#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "ml_relocalization/candidate_selection.hpp"
using namespace ml_relocalization;
MatchResult candidate(double x, double yaw, double score) {
  MatchResult r;r.valid=true;r.likelihood=score;r.T_map_body(0,3)=x;
  r.T_map_body.block<3,3>(0,0)=Eigen::AngleAxisd(yaw*M_PI/180.,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return r;
}
TEST(PolarSelection, RejectsHighScoreButSeparatedSolutions) {
  auto r=SelectPolarCandidate({candidate(0,0,3),candidate(10,0,4)},Config{});
  EXPECT_FALSE(r.valid);EXPECT_EQ(r.reason,"polar_ambiguous_candidates");
}
TEST(PolarSelection, ChecksAllPairsNotOnlyBest) {
  auto r=SelectPolarCandidate({candidate(0,0,4),candidate(-1.5,0,3),candidate(1.5,0,3)},Config{});
  EXPECT_FALSE(r.valid);
}
TEST(PolarSelection, HandlesYawWrapAndChoosesBestConsistentScore) {
  auto r=SelectPolarCandidate({candidate(0,179,3),candidate(.4,-179,4)},Config{});
  EXPECT_TRUE(r.valid);EXPECT_DOUBLE_EQ(r.likelihood,4);
}
TEST(PolarSelection, RejectsYawAmbiguityAndEmptyInput) {
  EXPECT_FALSE(SelectPolarCandidate({candidate(0,0,3),candidate(0,90,4)},Config{}).valid);
  EXPECT_FALSE(SelectPolarCandidate({},Config{}).valid);
}
