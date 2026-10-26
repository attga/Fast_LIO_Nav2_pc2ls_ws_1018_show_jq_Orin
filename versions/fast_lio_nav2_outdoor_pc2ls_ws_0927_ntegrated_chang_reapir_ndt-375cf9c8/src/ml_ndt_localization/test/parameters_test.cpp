#include <gtest/gtest.h>
#include "node_parameters.hpp"
#include <limits>
namespace ml_ndt_loc {
class ParametersTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  NodeParams Load(std::vector<rclcpp::Parameter> values = {}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(values);
    return LoadNodeParams(std::make_shared<rclcpp::Node>("parameters_test", options));
  }
};
TEST_F(ParametersTest, DefaultsAndOverrides) {
  auto p = Load();
  EXPECT_EQ(p.tracker.ndt.max_iterations, 30);
  EXPECT_DOUBLE_EQ(p.tracker.likelihood_threshold, 2.3);
  EXPECT_TRUE(p.wait_for_initialization);
  EXPECT_NO_THROW(Load({{"registration_rate", 1000.0}, {"inlier_fraction_threshold", 0.0},
                       {"max_translation_jump", 0.0}, {"max_yaw_jump", 360.0}}));
}
TEST_F(ParametersTest, LoadsGroupedOverrides) {
  const auto p = Load({{"input_cloud_topic", "/test/scan"}, {"voxel_size", 0.6},
                       {"registration_threads", 3}, {"ndt.resolution", 1.5},
                       {"covariance.dynamic_xy", false}, {"covariance.temperature", 0.1},
                       {"position_stddev", 0.4}, {"registration_rate", 4.0},
                       {"pause_on_lost", true}, {"wait_for_initialization", false}});
  EXPECT_EQ(p.input_cloud_topic, "/test/scan");
  EXPECT_DOUBLE_EQ(p.tracker.voxel_size, 0.6);
  EXPECT_EQ(p.tracker.ndt.threads, 3);
  EXPECT_DOUBLE_EQ(p.tracker.ndt.resolution, 1.5);
  EXPECT_FALSE(p.tracker.ndt.estimate_covariance);
  EXPECT_DOUBLE_EQ(p.tracker.ndt.covariance_temperature, 0.1);
  EXPECT_DOUBLE_EQ(p.position_stddev, 0.4);
  EXPECT_DOUBLE_EQ(p.registration_rate, 4.0);
  EXPECT_TRUE(p.tracker.pause_on_lost);
  EXPECT_FALSE(p.wait_for_initialization);
}
TEST_F(ParametersTest, RejectsUnsafeParameters) {
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<rclcpp::Parameter> invalid = {
    {"registration_rate", 0.0}, {"registration_rate", 1001.0}, {"registration_rate", 1e-300},
    {"ndt.resolution", nan}, {"ndt.step_size", -1.0}, {"ndt.max_iterations", 1},
    {"ndt.transformation_epsilon", 0.0}, {"registration_threads", 0},
    {"inlier_fraction_threshold", 1.1}, {"inlier_fraction_threshold", nan},
    {"max_translation_jump", -1.0}, {"max_consecutive_failures", 0},
    {"submap_update_margin", 100.0}, {"voxel_size", 0.0},
    {"covariance.temperature", 0.0}, {"covariance.temperature", nan}, {"position_stddev", 1e300}, {"position_stddev", 1e-300}, {"position_stddev", inf}, {"scan_timeout", inf}, {"orientation_stddev", 0.0}, {"base_frame", "map"}};
  for (const auto& value : invalid) {
    SCOPED_TRACE(value.get_name());
    EXPECT_THROW(Load({value}), std::invalid_argument);
  }
}
TEST_F(ParametersTest, RejectsOldAlgorithmConfig) {
  for (const auto& value : std::vector<rclcpp::Parameter>{
    {"registration_backend", "fast_gicp"}, {"fitness_threshold", 0.3},
    {"icp_ds", 0.4}, {"pose_file", "isam_poses.txt"}})
    EXPECT_THROW(Load({value}), std::invalid_argument);
}
} // namespace ml_ndt_loc
