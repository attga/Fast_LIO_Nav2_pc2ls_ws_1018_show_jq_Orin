#include "node_parameters.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ml_ndt_loc
{
namespace
{
void RejectRemovedParameters(rclcpp::Node& node)
{
    const auto& overrides = node.get_node_parameters_interface()->get_parameter_overrides();
    for (const auto* name : {"registration_backend", "correspondence_distance", "fitness_threshold",
        "icp_rate", "icp_ds", "submap_strategy", "submap_halfwidth", "pose_file",
        "pose_covariance", "orientation_covariance", "covariance_fitness_scale",
        "covariance_fitness_scale_ori", "failure_count_before_relocalization",
        "input_ekf_topic", "sync_tolerance", "init_pose", "odom_frame"}) {
        if (overrides.count(name))
            throw std::invalid_argument(std::string("removed localization parameter: ") + name);
    }
}

void LoadInputParameters(rclcpp::Node& node, NodeParams& p)
{
    p.input_cloud_topic =
        node.declare_parameter<std::string>("input_cloud_topic", p.input_cloud_topic);
    p.map_frame =
        node.declare_parameter<std::string>("map_frame", p.map_frame);
    p.base_frame =
        node.declare_parameter<std::string>("base_frame", p.base_frame);
    p.scan_timeout =
        node.declare_parameter<double>("scan_timeout", p.scan_timeout);
    p.tf_wait_timeout =
        node.declare_parameter<double>("tf_wait_timeout", p.tf_wait_timeout);
    p.map_directory =
        node.declare_parameter<std::string>("map_directory", p.map_directory);
    p.pub_map =
        node.declare_parameter<bool>("pub_map", p.pub_map);
    p.tracker.submap_size =
        node.declare_parameter<double>("submap_size", p.tracker.submap_size);
    p.tracker.submap_update_margin =
        node.declare_parameter<double>("submap_update_margin", p.tracker.submap_update_margin);
    p.tracker.voxel_size =
        node.declare_parameter<double>("voxel_size", p.tracker.voxel_size);
}

void LoadRegistrationParameters(rclcpp::Node& node, NodeParams& p)
{
    p.tracker.ndt.threads =
        node.declare_parameter<int>("registration_threads", p.tracker.ndt.threads);
    p.tracker.ndt.resolution =
        node.declare_parameter<double>("ndt.resolution", p.tracker.ndt.resolution);
    p.tracker.ndt.step_size =
        node.declare_parameter<double>("ndt.step_size", p.tracker.ndt.step_size);
    p.tracker.ndt.transformation_epsilon =
        node.declare_parameter<double>("ndt.transformation_epsilon", p.tracker.ndt.transformation_epsilon);
    p.tracker.ndt.max_iterations =
        node.declare_parameter<int>("ndt.max_iterations", p.tracker.ndt.max_iterations);
    p.tracker.likelihood_threshold =
        node.declare_parameter<double>("likelihood_threshold", p.tracker.likelihood_threshold);
    p.tracker.inlier_fraction_threshold =
        node.declare_parameter<double>("inlier_fraction_threshold", p.tracker.inlier_fraction_threshold);
    p.tracker.max_translation_jump =
        node.declare_parameter<double>("max_translation_jump", p.tracker.max_translation_jump);
    p.tracker.max_yaw_jump =
        node.declare_parameter<double>("max_yaw_jump", p.tracker.max_yaw_jump);
}

void LoadCovarianceParameters(rclcpp::Node& node, NodeParams& p)
{
    p.tracker.ndt.estimate_covariance =
        node.declare_parameter<bool>("covariance.dynamic_xy", p.tracker.ndt.estimate_covariance);
    p.tracker.ndt.covariance_temperature =
        node.declare_parameter<double>("covariance.temperature", p.tracker.ndt.covariance_temperature);
    p.position_stddev =
        node.declare_parameter<double>("position_stddev", p.position_stddev);
    p.orientation_stddev =
        node.declare_parameter<double>("orientation_stddev", p.orientation_stddev);
}

void LoadRuntimeParameters(rclcpp::Node& node, NodeParams& p)
{
    p.registration_rate =
        node.declare_parameter<double>("registration_rate", p.registration_rate);
    p.max_consecutive_failures =
        node.declare_parameter<int>("max_consecutive_failures", p.max_consecutive_failures);
    p.enter_lost_on_failure =
        node.declare_parameter<bool>("enter_lost_on_failure", p.enter_lost_on_failure);
    p.tracker.pause_on_lost =
        node.declare_parameter<bool>("pause_on_lost", p.tracker.pause_on_lost);
    p.wait_for_initialization =
        node.declare_parameter<bool>("wait_for_initialization", p.wait_for_initialization);
}

void RequireFinitePositive(const char* name, double value)
{
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
}

void RequireValidStddev(const char* name, double value)
{
    RequireFinitePositive(name, value);
    // 标准差需要平方后发布为方差，额外检查溢出与下溢。
    const double variance = value * value;
    if (!std::isfinite(variance) || variance == 0.0)
        throw std::invalid_argument(std::string(name) + " must have a finite positive square");
}

void ValidateParameters(const NodeParams& p)
{
    RequireFinitePositive("registration_rate", p.registration_rate);
    const double period = 1000.0 / p.registration_rate;
    if (period < 1.0 || period > std::numeric_limits<int>::max())
        throw std::invalid_argument("registration_rate produces an unrepresentable timer period");
    RequireFinitePositive("scan_timeout", p.scan_timeout);
    RequireFinitePositive("tf_wait_timeout", p.tf_wait_timeout);
    RequireFinitePositive("voxel_size", p.tracker.voxel_size);
    RequireFinitePositive("submap_size", p.tracker.submap_size);
    RequireFinitePositive("submap_update_margin", p.tracker.submap_update_margin);
    if (p.tracker.submap_update_margin >= p.tracker.submap_size / 2.0)
        throw std::invalid_argument("submap_update_margin must be less than half the window size");
    RequireFinitePositive("ndt.resolution", p.tracker.ndt.resolution);
    RequireFinitePositive("ndt.step_size", p.tracker.ndt.step_size);
    RequireFinitePositive("ndt.transformation_epsilon", p.tracker.ndt.transformation_epsilon);
    RequireFinitePositive("covariance.temperature", p.tracker.ndt.covariance_temperature);
    RequireValidStddev("position_stddev", p.position_stddev);
    RequireValidStddev("orientation_stddev", p.orientation_stddev);
    if (p.tracker.ndt.threads < 1 || p.tracker.ndt.max_iterations < 2)
        throw std::invalid_argument("NDT requires threads>=1 and max_iterations>=2");
    for (double v : {p.tracker.likelihood_threshold, p.tracker.max_translation_jump, p.tracker.max_yaw_jump})
        if (!std::isfinite(v) || v < 0) throw std::invalid_argument("quality limits must be finite and nonnegative");
    if (!std::isfinite(p.tracker.inlier_fraction_threshold) || p.tracker.inlier_fraction_threshold < 0 ||
        p.tracker.inlier_fraction_threshold > 1)
        throw std::invalid_argument("inlier_fraction_threshold must be in [0,1]");
    if (p.enter_lost_on_failure && p.max_consecutive_failures < 1)
        throw std::invalid_argument("max_consecutive_failures must be >=1 when automatic LOST is enabled");
    if (p.map_frame.empty() || p.base_frame.empty() || p.map_frame == p.base_frame || p.input_cloud_topic.empty())
        throw std::invalid_argument("nonempty distinct map/base frames and input topic are required");
}
} // namespace

NodeParams LoadNodeParams(const rclcpp::Node::SharedPtr& node)
{
    RejectRemovedParameters(*node);
    NodeParams params;
    LoadInputParameters(*node, params);
    LoadRegistrationParameters(*node, params);
    LoadCovarianceParameters(*node, params);
    LoadRuntimeParameters(*node, params);
    ValidateParameters(params);
    return params;
}
} // namespace ml_ndt_loc
