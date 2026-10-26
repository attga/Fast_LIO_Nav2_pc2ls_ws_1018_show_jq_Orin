#include <gtest/gtest.h>
#include "ml_ndt_localization/tracker.hpp"
#include <pcl/common/transforms.h>
#include <filesystem>
#include <random>
#include <unistd.h>

namespace ml_ndt_loc {
class RegistrationTest : public ::testing::Test {
protected:
  std::filesystem::path directory;
  PointCloud::Ptr map = pcl::make_shared<PointCloud>();
  void SetUp() override {
    directory = std::filesystem::temp_directory_path() / ("ndt_component_" + std::to_string(getpid()));
    std::filesystem::create_directories(directory);
    std::mt19937 generator(41);
    std::normal_distribution<float> noise(0, .18);
    // 非对称三维结构，每个概率体素有足够采样；不依赖现场地图。
    for (int x=0; x<8; ++x) for (int y=0; y<7; ++y) for (int j=0; j<180; ++j) {
      PointType p;
      p.x=2.5f*x+noise(generator); p.y=2.5f*y+noise(generator);
      p.z=.45f*x+.13f*y*y+noise(generator); p.intensity=1;
      map->push_back(p);
    }
    pcl::io::savePCDFileBinary((directory / "map.pcd").string(), *map);
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
};
TEST_F(RegistrationTest, MapWindowCachingAndMissingMap) {
  EXPECT_THROW(SubmapCache("/nonexistent/map.pcd", .4, 200, 80), std::runtime_error);
  SubmapCache cache((directory/"map.pcd").string(), .1, 200, 80);
  auto first=cache.Select({0,0}); EXPECT_TRUE(first.updated);
  auto inside=cache.Select({19,0}); EXPECT_FALSE(inside.updated);
  EXPECT_EQ(first.cloud, inside.cloud);
  auto edge=cache.Select({20,0}); EXPECT_TRUE(edge.updated);
  EXPECT_NE(first.cloud, edge.cloud);
}
TEST_F(RegistrationTest, RejectsInvalidInputAndResetsHealth) {
  Tracker::Config cfg; cfg.voxel_size=.1;
  Tracker tracker(directory.string(), cfg);
  auto empty=pcl::make_shared<PointCloud>();
  EXPECT_EQ(tracker.TrackOnce(empty, Eigen::Matrix4d::Identity()).rejection_reason, "scan_empty");
  Eigen::Matrix4d bad=Eigen::Matrix4d::Identity(); bad(0,0)=2;
  EXPECT_EQ(tracker.TrackOnce(map, bad).rejection_reason, "prediction_not_rigid");
  bad(0,0)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tracker.TrackOnce(map, bad).rejection_reason, "prediction_nonfinite");
  EXPECT_EQ(tracker.GetStatus().fail_count, 3);
  tracker.MarkLost(); EXPECT_TRUE(tracker.IsLost());
  tracker.Reset(); EXPECT_FALSE(tracker.IsLost()); EXPECT_EQ(tracker.GetStatus().fail_count, 0);
}
TEST_F(RegistrationTest, AbsoluteSixDofPoseAndTargetReuse) {
  NdtMatcher matcher(NdtMatcher::Config{});
  Eigen::Matrix4d truth=Eigen::Matrix4d::Identity();
  truth.block<3,3>(0,0)=(Eigen::AngleAxisd(.23,Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(.06,Eigen::Vector3d::UnitY())).toRotationMatrix();
  truth.block<3,1>(0,3)=Eigen::Vector3d(3,-2,.7);
  auto scan=pcl::make_shared<PointCloud>();
  pcl::transformPointCloud(*map,*scan,truth.inverse().eval());
  auto initial=truth; initial(0,3)+=.12;
  auto result=matcher.Align(scan,map,initial);
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.likelihood,2.3);
  EXPECT_LT((result.T_map_base-truth).norm(),.12);
  EXPECT_GT(result.inlier_fraction,.99);
  EXPECT_GT(result.target_build_ms,0);
  EXPECT_TRUE(result.covariance_valid);
  EXPECT_TRUE(result.covariance_xy.allFinite());
  EXPECT_TRUE(result.covariance_xy.isApprox(result.covariance_xy.transpose(),1e-10));
  EXPECT_GE(result.covariance_xy.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),-1e-12);
  EXPECT_GT(result.covariance_ms,0);
  NdtMatcher::Config fixed_config; fixed_config.estimate_covariance=false;
  NdtMatcher fixed(fixed_config);
  const auto fixed_result=fixed.Align(scan,map,initial);
  EXPECT_TRUE(fixed_result.T_map_base.isApprox(result.T_map_base,1e-6));
  EXPECT_DOUBLE_EQ(fixed_result.covariance_ms,0);
  EXPECT_TRUE(fixed_result.covariance_xy.isZero());
  NdtMatcher::Config broad_config; broad_config.covariance_temperature=10.;
  NdtMatcher broad(broad_config);
  const auto broad_result=broad.Align(scan,map,initial);
  EXPECT_GT(broad_result.covariance_xy.trace(),result.covariance_xy.trace());
  auto second=matcher.Align(scan,map,initial);
  EXPECT_DOUBLE_EQ(second.target_build_ms,0);
  EXPECT_TRUE(second.T_map_base.isApprox(result.T_map_base,1e-6));
}
TEST_F(RegistrationTest, QualityRejectAndLostRecovery) {
  Tracker::Config cfg; cfg.voxel_size=.1; cfg.likelihood_threshold=1e6;
  Tracker reject(directory.string(),cfg);
  auto initial=Eigen::Matrix4d::Identity().eval(); initial(0,3)=.1;
  EXPECT_FALSE(reject.TrackOnce(map,initial).accepted);
  EXPECT_EQ(reject.GetStatus().fail_count,1);
  cfg.likelihood_threshold=2.3;
  Tracker recover(directory.string(),cfg);
  recover.MarkLost();
  const auto result=recover.TrackOnce(map,initial);
  EXPECT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_FALSE(recover.IsLost()); EXPECT_EQ(recover.GetStatus().fail_count,0);
  cfg.pause_on_lost=true;
  Tracker paused(directory.string(),cfg); paused.MarkLost();
  EXPECT_EQ(paused.TrackOnce(map,initial).rejection_reason,"tracker_lost_waiting_reset");
  EXPECT_EQ(paused.GetStatus().fail_count,0);
}
// 绕过 ROS 参数校验直接调用核心，确保 NaN 门限不会被比较式漏过。
TEST_F(RegistrationTest, QualityGatesRejectNaNThresholds) {
  using Field = double Tracker::Config::*;
  const Field fields[] = {&Tracker::Config::likelihood_threshold,
      &Tracker::Config::inlier_fraction_threshold,
      &Tracker::Config::max_translation_jump, &Tracker::Config::max_yaw_jump};
  auto initial=Eigen::Matrix4d::Identity().eval(); initial(0,3)=.1;
  for (const auto field : fields) {
    Tracker::Config cfg; cfg.voxel_size=.1;
    cfg.*field=std::numeric_limits<double>::quiet_NaN();
    Tracker tracker(directory.string(),cfg);
    const auto result=tracker.TrackOnce(map,initial);
    ASSERT_TRUE(result.converged);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.rejection_reason.empty());
    EXPECT_EQ(tracker.GetStatus().fail_count,1);
  }
}
} // namespace ml_ndt_loc
