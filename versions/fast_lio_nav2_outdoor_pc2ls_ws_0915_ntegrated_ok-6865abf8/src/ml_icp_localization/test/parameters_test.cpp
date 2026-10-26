#include <gtest/gtest.h>
#include "node_parameters.hpp"
#include <limits>

namespace ml_icp_loc {
class ParametersTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  NodeParams Load(std::vector<rclcpp::Parameter> values = {}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(values);
    auto node = std::make_shared<rclcpp::Node>("parameters_test", options);
    return LoadNodeParams(node);
  }
};
TEST_F(ParametersTest, DefaultsAndInclusiveQualityBoundsRemainValid) {
  EXPECT_NO_THROW(Load());
  EXPECT_NO_THROW(Load({{"icp_rate", 1000.0}, {"inlier_fraction_threshold", 0.0},
                       {"max_translation_jump", 0.0}, {"max_yaw_jump", 360.0}}));
  EXPECT_NO_THROW(Load({{"inlier_fraction_threshold", 1.0}}));
}
TEST_F(ParametersTest, RejectsInvalidNumericRanges) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<rclcpp::Parameter> invalid = {
    {"icp_rate", 0.0}, {"icp_rate", -1.0}, {"icp_rate", 1001.0},
    {"icp_rate", 1e-300}, {"icp_rate", nan}, {"icp_rate", inf},
    {"correspondence_distance", 0.0}, {"correspondence_distance", -1.0},
    {"correspondence_distance", 1e300}, {"correspondence_distance", 1e-300},
    {"inlier_fraction_threshold", -0.1}, {"inlier_fraction_threshold", 1.1},
    {"inlier_fraction_threshold", nan}, {"max_translation_jump", -1.0},
    {"max_yaw_jump", inf}, {"failure_count_before_relocalization", 0},
    {"submap_halfwidth", -1}, {"registration_threads", 0},
    {"registration_backend", "unknown"}, {"submap_strategy", "unknown"}};
  for (const auto& value : invalid) {
    SCOPED_TRACE(value.get_name() + "=" + value.value_to_string());
    EXPECT_THROW(Load({value}), std::invalid_argument);
  }
}
TEST_F(ParametersTest, ChecksOnlyActiveSubmapAndLostPolicy) {
  EXPECT_NO_THROW(Load({{"submap_strategy", "window"}, {"submap_halfwidth", -1}}));
  EXPECT_NO_THROW(Load({{"submap_strategy", "keyframes"}, {"submap_size", -1.0},
                       {"submap_update_margin", -1.0}}));
  EXPECT_NO_THROW(Load({{"enter_lost_on_failure", false},
                       {"failure_count_before_relocalization", 0}}));
  EXPECT_THROW(Load({{"submap_strategy", "window"}, {"submap_update_margin", 100.0}}), std::invalid_argument);
  EXPECT_THROW(Load({{"submap_strategy", "window"}, {"submap_size", -1.0}}), std::invalid_argument);
}
TEST_F(ParametersTest, RejectsRemovedOverridesExplicitly) {
  const std::vector<rclcpp::Parameter> removed = {
    {"input_ekf_topic", "/old_odom"}, {"sync_tolerance", 0.2},
    {"init_pose", std::vector<double>{0.,0.,0.,0.,0.,0.}}, {"odom_frame", "odom"}};
  for (const auto& value : removed) {
    try { Load({value}); FAIL() << "accepted " << value.get_name(); }
    catch (const std::invalid_argument& e) {
      EXPECT_EQ(std::string(e.what()), "removed ICP parameter: " + value.get_name());
    }
  }
}
} // namespace ml_icp_loc
