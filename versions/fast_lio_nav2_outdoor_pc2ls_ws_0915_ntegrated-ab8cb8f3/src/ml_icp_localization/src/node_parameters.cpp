#include "node_parameters.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace ml_icp_loc
{
namespace
{

// 校验参数可表示范围与算法约束；拒绝非法值，不自动钳制。
void ValidateNodeParams(const NodeParams& p)
{
    const double period_ms = 1000.0 / p.icp_rate;
    if (!std::isfinite(p.icp_rate) || p.icp_rate <= 0.0 ||
        !std::isfinite(period_ms) || period_ms < 1.0 ||
        period_ms > static_cast<double>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("icp_rate must produce a finite timer period in [1, INT_MAX] milliseconds");
    if (p.tracker.registration_backend != "pcl_gicp" && p.tracker.registration_backend != "fast_gicp")
        throw std::invalid_argument("registration_backend must be pcl_gicp or fast_gicp");
    if (p.tracker.registration_threads < 1)
        throw std::invalid_argument("registration_threads must be >= 1");
    const double distance = p.tracker.correspondence_distance;
    if (!std::isfinite(distance) || distance <= 0.0 ||
        !std::isfinite(distance * distance) || distance * distance <= 0.0)
        throw std::invalid_argument("correspondence_distance must have a finite positive square");
    if (!std::isfinite(p.tracker.inlier_fraction_threshold) ||
        p.tracker.inlier_fraction_threshold < 0.0 || p.tracker.inlier_fraction_threshold > 1.0)
        throw std::invalid_argument("inlier_fraction_threshold must be in [0, 1]");
    if (!std::isfinite(p.tracker.max_translation_jump) || p.tracker.max_translation_jump < 0.0)
        throw std::invalid_argument("max_translation_jump must be finite and nonnegative");
    // 大于 180 度可用于关闭实际 yaw 限制，保留原有合法配置能力。
    if (!std::isfinite(p.tracker.max_yaw_jump) || p.tracker.max_yaw_jump < 0.0)
        throw std::invalid_argument("max_yaw_jump must be finite and nonnegative");
    if (p.enter_lost_on_failure && p.tracker.failure_count_before_relocalization < 1)
        throw std::invalid_argument("failure_count_before_relocalization must be >= 1 when automatic LOST is enabled");
    if (p.tracker.submap_strategy == "window") {
        const double size = p.tracker.submap_size, margin = p.tracker.submap_update_margin;
        if (!std::isfinite(size) || size <= 0.0 || !std::isfinite(margin) ||
            margin <= 0.0 || margin >= size / 2.0)
            throw std::invalid_argument("window requires finite submap_size>0 and 0<submap_update_margin<submap_size/2");
    } else if (p.tracker.submap_strategy == "keyframes") {
        if (p.tracker.submap_halfwidth < 0)
            throw std::invalid_argument("submap_halfwidth must be nonnegative for keyframes");
    } else {
        throw std::invalid_argument("submap_strategy must be keyframes or window");
    }

    if (!std::isfinite(p.scan_timeout) || p.scan_timeout <= 0.0)
        throw std::invalid_argument("scan_timeout must be finite and positive");
    if (!std::isfinite(p.tf_wait_timeout) || p.tf_wait_timeout <= 0.0)
        throw std::invalid_argument("tf_wait_timeout must be finite and positive");
    // 配置使用标准差；验证最大可接受评分下的方差可表示。
    if (!std::isfinite(p.tracker.fitness_threshold) || p.tracker.fitness_threshold < 0.0)
        throw std::invalid_argument("fitness_threshold must be finite and nonnegative");
    const auto validate_noise = [&](double floor, double scale) {
        const double maximum = std::max(floor, scale * p.tracker.fitness_threshold);
        return std::isfinite(floor) && floor > 0.0 && floor * floor > 0.0 &&
               std::isfinite(scale) && scale >= 0.0 && std::isfinite(maximum * maximum);
    };
    if (!validate_noise(p.pose_covariance, p.covariance_fitness_scale) ||
        !validate_noise(p.orientation_covariance, p.covariance_fitness_scale_ori))
        throw std::invalid_argument("pose noise must produce finite positive variances; scales must be nonnegative");
    if (!std::isfinite(p.tracker.icp_ds) || p.tracker.icp_ds <= 0.0)
        throw std::invalid_argument("icp_ds must be finite and positive");
}

} // namespace

// 默认值统一来自 NodeParams / Tracker::Config，参数名与 YAML 保持不变。
NodeParams LoadNodeParams(const rclcpp::Node::SharedPtr& node)
{
    const auto& overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    for (const auto* name : {"input_ekf_topic", "sync_tolerance", "init_pose", "odom_frame"}) {
        if (overrides.count(name))
            throw std::invalid_argument(std::string("removed ICP parameter: ") + name);
    }
    NodeParams p;
    // 输入坐标系与调度时效。
    p.input_cloud_topic = node->declare_parameter<std::string>("input_cloud_topic", p.input_cloud_topic);
    p.map_frame = node->declare_parameter<std::string>("map_frame", p.map_frame);
    p.base_frame = node->declare_parameter<std::string>("base_frame", p.base_frame);
    p.icp_rate = node->declare_parameter<double>("icp_rate", p.icp_rate);
    p.scan_timeout = node->declare_parameter<double>("scan_timeout", p.scan_timeout);
    p.tf_wait_timeout = node->declare_parameter<double>("tf_wait_timeout", p.tf_wait_timeout);

    // 地图加载与子图。
    p.map_directory = node->declare_parameter<std::string>("map_directory", p.map_directory);
    p.pose_file = node->declare_parameter<std::string>("pose_file", p.pose_file);
    p.pub_map = node->declare_parameter<bool>("pub_map", p.pub_map);
    p.tracker.submap_strategy = node->declare_parameter<std::string>("submap_strategy", p.tracker.submap_strategy);
    p.tracker.submap_size = node->declare_parameter<double>("submap_size", p.tracker.submap_size);
    p.tracker.submap_update_margin = node->declare_parameter<double>("submap_update_margin", p.tracker.submap_update_margin);
    p.tracker.submap_halfwidth = node->declare_parameter<int>("submap_halfwidth", p.tracker.submap_halfwidth);

    // 配准后端与点云降采样。
    p.tracker.registration_backend = node->declare_parameter<std::string>("registration_backend", p.tracker.registration_backend);
    p.tracker.registration_threads = node->declare_parameter<int>("registration_threads", p.tracker.registration_threads);
    p.tracker.correspondence_distance = node->declare_parameter<double>("correspondence_distance", p.tracker.correspondence_distance);
    p.tracker.icp_ds = node->declare_parameter<double>("icp_ds", p.tracker.icp_ds);

    // 质量门限与初始化/恢复策略。
    p.tracker.fitness_threshold = node->declare_parameter<double>("fitness_threshold", p.tracker.fitness_threshold);
    p.tracker.inlier_fraction_threshold = node->declare_parameter<double>("inlier_fraction_threshold", p.tracker.inlier_fraction_threshold);
    p.tracker.max_translation_jump = node->declare_parameter<double>("max_translation_jump", p.tracker.max_translation_jump);
    p.tracker.max_yaw_jump = node->declare_parameter<double>("max_yaw_jump", p.tracker.max_yaw_jump);
    p.tracker.failure_count_before_relocalization = node->declare_parameter<int>("failure_count_before_relocalization", p.tracker.failure_count_before_relocalization);
    p.enter_lost_on_failure = node->declare_parameter<bool>("enter_lost_on_failure", p.enter_lost_on_failure);
    p.tracker.pause_on_lost = node->declare_parameter<bool>("pause_on_lost", p.tracker.pause_on_lost);
    p.wait_for_initialization = node->declare_parameter<bool>("wait_for_initialization", p.wait_for_initialization);

    // 测量输出噪声（参数为标准差，发布时转为方差）。
    p.pose_covariance = node->declare_parameter<double>("pose_covariance", p.pose_covariance);
    p.orientation_covariance = node->declare_parameter<double>("orientation_covariance", p.orientation_covariance);
    p.covariance_fitness_scale = node->declare_parameter<double>("covariance_fitness_scale", p.covariance_fitness_scale);
    p.covariance_fitness_scale_ori = node->declare_parameter<double>("covariance_fitness_scale_ori", p.covariance_fitness_scale_ori);

    ValidateNodeParams(p);
    return p;
}

} // namespace ml_icp_loc
