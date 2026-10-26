#include "ml_relocalization/relocalizer.hpp"
#include "ml_relocalization/confirmation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace ml_relocalization {
class RelocalizationNode final : public rclcpp::Node {
  using CloudMessage = sensor_msgs::msg::PointCloud2;
  using PoseMessage = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Trigger = std_srvs::srv::Trigger;
public:
  RelocalizationNode() : Node("alaserRelocalizer"), config_(parameters()),
                        relocalizer_(config_), confirmation_(config_) {
    localized_ = !declare_parameter<bool>("start_on_launch", true);
    pose_publisher_ = create_publisher<PoseMessage>("/relocalized_pose", 10);
    if (config_.debug_publish_clouds) {
      hypothesis_publisher_ = create_publisher<CloudMessage>("/relocalization/current_hyp", 1);
      submap_publisher_ = create_publisher<CloudMessage>("/relocalization/submap", 1);
      aligned_publisher_ = create_publisher<CloudMessage>("/relocalization/aligned", 1);
    }
    scan_subscription_ = create_subscription<CloudMessage>(
        "/cloud_registered_body", rclcpp::QoS(10).best_effort(),
        [this](CloudMessage::ConstSharedPtr message) { receiveScan(*message); });
    trigger_service_ = create_service<Trigger>("trigger_relocalization",
        [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(session_mutex_);
          localized_ = false;
          confirmation_.reset();
          ++generation_;  // In-flight results from an earlier request cannot publish.
          response->success = true;
          response->message = "relocalization re-triggered";
          RCLCPP_INFO(get_logger(), "relocalization restarted");
        });
    RCLCPP_INFO(get_logger(), "map loaded; Scan Context %s", localized_ ? "waiting for trigger" : "active (1 Hz)");
    worker_ = std::thread([this] { run(); });
  }

  ~RelocalizationNode() override {
    { std::lock_guard<std::mutex> lock(stop_mutex_); stopping_ = true; }
    stop_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  Config parameters() {
    Config config;
    config.map_directory = declare_parameter("map_directory", config.map_directory);
    config.pose_file = declare_parameter("pose_file", config.pose_file);
    config.sc_dist_thres = declare_parameter("sc_dist_thres", config.sc_dist_thres);
    config.icp_fitness_thres = declare_parameter("icp_fitness_thres", config.icp_fitness_thres);
    config.icp_t_max = declare_parameter("icp_t_max", config.icp_t_max);
    config.icp_yaw_max_deg = declare_parameter("icp_yaw_max_deg", config.icp_yaw_max_deg);
    config.icp_max_corr_dist = declare_parameter("icp_max_corr_dist", config.icp_max_corr_dist);
    config.confirm_position_thresh = declare_parameter("confirm_position_thresh", config.confirm_position_thresh);
    config.confirm_yaw_thresh_deg = declare_parameter("confirm_yaw_thresh_deg", config.confirm_yaw_thresh_deg);
    config.yaw_diff_sign = declare_parameter("yaw_diff_sign", config.yaw_diff_sign);
    config.debug_publish_clouds = declare_parameter("debug_publish_clouds", config.debug_publish_clouds);
    config.submap_min_points = declare_parameter("submap_min_points", config.submap_min_points);
    config.confirm_frames = declare_parameter("confirm_frames", config.confirm_frames);
    config.submap_halfwidth = declare_parameter("submap_halfwidth", config.submap_halfwidth);
    config.sc_ds_size = declare_parameter("sc_ds_size", config.sc_ds_size);
    config.icp_ds_size = declare_parameter("icp_ds_size", config.icp_ds_size);
    if (!(config.sc_ds_size > 0.0) || !std::isfinite(config.sc_ds_size) ||
        !(config.icp_ds_size > 0.0) || !std::isfinite(config.icp_ds_size) ||
        config.confirm_frames < 1 || config.submap_halfwidth < 0 || config.submap_min_points < 1)
      throw std::invalid_argument("invalid voxel size, confirmation count or submap bounds");
    return config;
  }

  void receiveScan(const CloudMessage& message) {
    if (localized_) return;
    if (message.header.frame_id != "base_link") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "expected body cloud in base_link, received %s", message.header.frame_id.c_str());
      return;
    }
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(message, *cloud);
    if (cloud->empty()) return;
    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(config_.sc_ds_size, config_.sc_ds_size, config_.sc_ds_size);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = filtered;
    latest_stamp_ = message.header.stamp;
  }

  void run() {
    // A single matching worker and a latest-scan slot bound the work queue.
    // Wall-time scheduling retains the original rclcpp::Rate(1.0) behavior.
    auto next = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      next += std::chrono::seconds(1);
      std::unique_lock<std::mutex> stop_lock(stop_mutex_);
      if (stop_condition_.wait_until(stop_lock, next, [this] { return stopping_; })) break;
      stop_lock.unlock();
      if (!rclcpp::ok()) break;
      try { attempt(); }
      catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "relocalization attempt failed: %s", error.what());
      }
      // Do not accumulate timer work when matching takes longer than one period.
      if (next < std::chrono::steady_clock::now() - std::chrono::seconds(1))
        next = std::chrono::steady_clock::now();
    }
  }

  void attempt() {
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (localized_) return;
      generation = generation_;
    }
    Cloud::Ptr scan;
    builtin_interfaces::msg::Time stamp;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!latest_scan_ || latest_scan_->empty()) return;
      stamp = latest_stamp_;
      const auto nanoseconds = rclcpp::Time(stamp).nanoseconds();
      if (nanoseconds == last_attempt_stamp_) return;
      last_attempt_stamp_ = nanoseconds;  // Claim before matching, including rejected scans.
      scan = latest_scan_;
    }
    const auto start = std::chrono::steady_clock::now();
    auto result = relocalizer_.match(scan);
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    publishCloud(hypothesis_publisher_, result.hypothesis, stamp);
    publishCloud(submap_publisher_, result.submap, stamp);
    publishCloud(aligned_publisher_, result.aligned, stamp);
    if (!result.valid) { log(result, stamp, "REJECT", result.reason, 0, elapsed); return; }

    std::lock_guard<std::mutex> lock(session_mutex_);
    if (generation != generation_ || localized_) {
      log(result, stamp, "SKIP", "superseded_trigger", 0, elapsed); return;
    }
    const int confirmed = confirmation_.observe(result.T_map_body);
    if (confirmed < config_.confirm_frames) {
      log(result, stamp, "PENDING", "consistency_pending", confirmed, elapsed); return;
    }
    PoseMessage message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    const auto position = result.T_map_body.block<3,1>(0,3);
    const Eigen::Quaterniond orientation(result.T_map_body.block<3,3>(0,0));
    message.pose.pose.position.x = position.x();
    message.pose.pose.position.y = position.y();
    message.pose.pose.position.z = position.z();
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    // Existing variances, not standard deviations; quality adaptation is separate work.
    for (int index : {0, 7, 14}) message.pose.covariance[index] = 0.5;
    for (int index : {21, 28, 35}) message.pose.covariance[index] = 0.05;
    pose_publisher_->publish(message);  // Atomic with respect to trigger reset.
    localized_ = true;
    log(result, stamp, "ACCEPT", "none", confirmed, elapsed);
  }

  void publishCloud(const rclcpp::Publisher<CloudMessage>::SharedPtr& publisher,
                    const Cloud::Ptr& cloud, const builtin_interfaces::msg::Time& stamp) {
    if (!publisher || !cloud) return;
    CloudMessage message;
    pcl::toROSMsg(*cloud, message);
    message.header.frame_id = "map";
    message.header.stamp = stamp;  // Debug data corresponds to the same sensor frame as the pose.
    publisher->publish(message);
  }

  void log(const MatchResult& result, const builtin_interfaces::msg::Time& stamp,
           const char* outcome, const std::string& reason, int confirmed, double elapsed) {
    RCLCPP_INFO(get_logger(),
        "[sc_reloc] %s scan_stamp=%.9f candidate=%d sc_distance=%.6f sc_limit=%.3f"
        " icp_ran=%d converged=%d fitness=%.6f fitness_limit=%.3f"
        " correction_m=%.3f correction_yaw_deg=%.3f submap_points=%zu"
        " confirm=%d/%d attempt_ms=%.1f reason=%s",
        outcome, rclcpp::Time(stamp).seconds(), result.candidate,
        result.sc_distance, config_.sc_dist_thres, result.icp_ran, result.converged,
        result.fitness, config_.icp_fitness_thres, result.correction_m,
        result.correction_yaw_deg, result.submap_points, confirmed,
        config_.confirm_frames, elapsed, reason.c_str());
  }

  Config config_;
  Relocalizer relocalizer_;
  Confirmation confirmation_;
  std::mutex scan_mutex_, session_mutex_, stop_mutex_;
  Cloud::Ptr latest_scan_;
  builtin_interfaces::msg::Time latest_stamp_;
  int64_t last_attempt_stamp_ = -1;  // Worker-only; retained across service triggers.
  uint64_t generation_ = 0;  // Protected by session_mutex_.
  std::atomic<bool> localized_{false};
  bool stopping_ = false;
  std::condition_variable stop_condition_;
  std::thread worker_;
  rclcpp::Subscription<CloudMessage>::SharedPtr scan_subscription_;
  rclcpp::Service<Trigger>::SharedPtr trigger_service_;
  rclcpp::Publisher<PoseMessage>::SharedPtr pose_publisher_;
  rclcpp::Publisher<CloudMessage>::SharedPtr hypothesis_publisher_, submap_publisher_, aligned_publisher_;
};
}  // namespace ml_relocalization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try { rclcpp::spin(std::make_shared<ml_relocalization::RelocalizationNode>()); }
  catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("alaserRelocalizer"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
