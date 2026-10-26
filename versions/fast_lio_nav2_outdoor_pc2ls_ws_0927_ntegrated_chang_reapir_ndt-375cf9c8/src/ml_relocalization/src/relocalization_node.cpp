#include "ml_relocalization/relocalizer.hpp"
#include "ml_relocalization/confirmation.hpp"
#include "registration.hpp"
#include "ml_relocalization/input_validation.hpp"
#include "ml_relocalization/matching_frame.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
#include <optional>
#include <thread>

namespace ml_relocalization {
class RelocalizationNode final : public rclcpp::Node {
  using CloudMessage = sensor_msgs::msg::PointCloud2;
  using PoseMessage = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Trigger = std_srvs::srv::Trigger;
public:
  RelocalizationNode() : Node("alaserRelocalizer"), config_(parameters()),
                        relocalizer_(config_), confirmation_(config_) {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    matching_frame_ = declare_parameter<std::string>("matching_frame", "base_link");
    if (base_frame_.empty() || matching_frame_.empty())
      throw std::invalid_argument("base_frame and matching_frame must not be empty");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // TF 回调共用节点 executor；不新增线程，不阻塞输入。
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    max_reloc_age_ = declare_parameter<double>("max_reloc_age", 1.5);
    future_tolerance_ = declare_parameter<double>("reloc_sync_tolerance", 0.2);
    if (!std::isfinite(max_reloc_age_) || max_reloc_age_ <= 0.0 ||
        !std::isfinite(future_tolerance_) || future_tolerance_ < 0.0)
      throw std::invalid_argument("invalid relocalization time limits");
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
          std::scoped_lock lock(session_mutex_, scan_mutex_);
          requested_at_ = now().seconds();
          latest_scan_.reset();
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
    {
      // 与结果提交串行化：先完成的提交可发布，停止标记之后不再提交。
      std::scoped_lock lock(session_mutex_, stop_mutex_);
      stopping_ = true;
    }
    stop_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  Config parameters() {
    Config config;
    config.map_directory = declare_parameter("map_directory", config.map_directory);
    config.pose_file = declare_parameter("pose_file", config.pose_file);
    config.sc_dist_thres = declare_parameter("sc_dist_thres", config.sc_dist_thres);
    config.ndt_resolution = declare_parameter("ndt.resolution", config.ndt_resolution);
    config.ndt_step_size = declare_parameter("ndt.step_size", config.ndt_step_size);
    config.ndt_transformation_epsilon = declare_parameter("ndt.transformation_epsilon", config.ndt_transformation_epsilon);
    config.ndt_max_iterations = declare_parameter("ndt.max_iterations", config.ndt_max_iterations);
    config.ndt_threads = declare_parameter("ndt.threads", config.ndt_threads);
    config.ndt_likelihood_threshold = declare_parameter("ndt.likelihood_threshold", config.ndt_likelihood_threshold);
    config.inlier_fraction_threshold = declare_parameter("inlier_fraction_threshold", config.inlier_fraction_threshold);
    config.max_translation_correction = declare_parameter("max_translation_correction", config.max_translation_correction);
    config.max_yaw_correction_deg = declare_parameter("max_yaw_correction_deg", config.max_yaw_correction_deg);
    for (const auto& entry : get_node_parameters_interface()->get_parameter_overrides()) {
      if (entry.first.rfind("icp_", 0) == 0)
        throw std::invalid_argument("obsolete ICP parameter: " + entry.first);
    }
    ValidateRegistrationConfig(config);
    config.confirm_position_thresh = declare_parameter("confirm_position_thresh", config.confirm_position_thresh);
    config.confirm_yaw_thresh_deg = declare_parameter("confirm_yaw_thresh_deg", config.confirm_yaw_thresh_deg);
    config.yaw_diff_sign = declare_parameter("yaw_diff_sign", config.yaw_diff_sign);
    config.debug_publish_clouds = declare_parameter("debug_publish_clouds", config.debug_publish_clouds);
    config.submap_min_points = declare_parameter("submap_min_points", config.submap_min_points);
    config.confirm_frames = declare_parameter("confirm_frames", config.confirm_frames);
    config.submap_halfwidth = declare_parameter("submap_halfwidth", config.submap_halfwidth);
    config.sc_ds_size = declare_parameter("sc_ds_size", config.sc_ds_size);
    config.registration_voxel_size = declare_parameter("registration_voxel_size", config.registration_voxel_size);
    config.relocalization_position_variance = declare_parameter(
        "relocalization_position_variance", config.relocalization_position_variance);
    config.relocalization_orientation_variance = declare_parameter(
        "relocalization_orientation_variance", config.relocalization_orientation_variance);
    // 参数直接表示输出方差，不再平方；非法配置在地图加载前拒绝。
    if (!std::isfinite(config.relocalization_position_variance) ||
        config.relocalization_position_variance <= 0.0)
      throw std::invalid_argument("relocalization_position_variance must be finite and positive");
    if (!std::isfinite(config.relocalization_orientation_variance) ||
        config.relocalization_orientation_variance <= 0.0)
      throw std::invalid_argument("relocalization_orientation_variance must be finite and positive");
    if (!(config.sc_ds_size > 0.0) || !std::isfinite(config.sc_ds_size) ||
        !(config.registration_voxel_size > 0.0) || !std::isfinite(config.registration_voxel_size) ||
        config.confirm_frames < 1 || config.submap_halfwidth < 0 || config.submap_min_points < 1)
      throw std::invalid_argument("invalid voxel size, confirmation count or submap bounds");
    return config;
  }

  std::optional<MatchingFrame> matchingFrame(const builtin_interfaces::msg::Time& stamp) {
    if (base_frame_ == matching_frame_) return MatchingFrame{};
    try {
      // 只依赖传感器安装 TF；启动重定位不要求 map/odom 已经存在。
      const auto transform = tf_buffer_->lookupTransform(base_frame_, matching_frame_, rclcpp::Time(stamp));
      const auto& r = transform.transform.rotation;
      const auto& t = transform.transform.translation;
      Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
      if (!q.coeffs().allFinite() || std::abs(q.squaredNorm() - 1.0) > 1e-3)
        throw std::invalid_argument("invalid installation rotation");
      Eigen::Matrix4d T_base_matching = Eigen::Matrix4d::Identity();
      T_base_matching.block<3,3>(0,0) = q.normalized().toRotationMatrix();
      T_base_matching.block<3,1>(0,3) = Eigen::Vector3d(t.x, t.y, t.z);
      return MatchingFrame(T_base_matching);
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "scan skipped: installation TF %s <- %s unavailable/invalid: %s",
                          base_frame_.c_str(), matching_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }

  // 调用者持有 session_mutex_，保证检查与触发边界一致。
  bool checkScanTime(const builtin_interfaces::msg::Time& stamp, const char* phase) {
    const double time = rclcpp::Time(stamp).seconds();
    const double current = now().seconds();
    const char* reason = ScanTimeRejection(time, current, requested_at_, max_reloc_age_, future_tolerance_);
    if (!reason) return true;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "scan time rejected phase=%s reason=%s stamp=%.9f request=%.9f now=%.9f age=%.3f max_age=%.3f future_tolerance=%.3f",
      phase, reason, time, requested_at_, current, current - time, max_reloc_age_, future_tolerance_);
    return false;
  }

  void receiveScan(const CloudMessage& message) {
    {
      std::lock_guard<std::mutex> lock(session_mutex_);
      if (localized_) return;
      if (!ValidScanStamp(message.header.stamp.sec, message.header.stamp.nanosec)) return;
      if (!checkScanTime(message.header.stamp, "receive")) return;
    }
    if (message.header.frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "expected body cloud in %s, received %s", base_frame_.c_str(), message.header.frame_id.c_str());
      return;
    }
    if (!ValidScanStamp(message.header.stamp.sec, message.header.stamp.nanosec)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "invalid scan timestamp; dropped");
      return;
    }
    const auto frame = matchingFrame(message.header.stamp);
    if (!frame) return;
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(message, *cloud);
    const auto removed = RemoveNonfinitePoints(*cloud);
    if (removed > 0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "removed %zu nonfinite XYZ points from scan", removed);
    if (cloud->empty()) return;
    // 在体素滤波和 SC 描述子生成之前回到传感器中心，避免改变描述子参考点。
    frame->convertScan(*cloud);
    RemoveNonfinitePoints(*cloud);
    if (cloud->empty()) return;
    auto filtered = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(config_.sc_ds_size, config_.sc_ds_size, config_.sc_ds_size);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    if (filtered->empty()) return;
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = filtered;
    latest_stamp_ = message.header.stamp;
    latest_frame_ = *frame;
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

  // 一次匹配固定使用同一帧点云、传感器时间和任务代次。
  // 后续确认可取同任务的新扫描，不把整个任务固定为一帧。
  struct AttemptSnapshot {
    Cloud::Ptr scan;
    builtin_interfaces::msg::Time stamp;
    uint64_t generation;
    MatchingFrame frame;
  };

  std::optional<AttemptSnapshot> takeSnapshot() {
    AttemptSnapshot snapshot;
    std::scoped_lock lock(session_mutex_, scan_mutex_);
    if (localized_ || !latest_scan_ || latest_scan_->empty()) return std::nullopt;
    snapshot.generation = generation_;
    snapshot.stamp = latest_stamp_;
    const auto nanoseconds = rclcpp::Time(snapshot.stamp).nanoseconds();
    if (nanoseconds <= last_attempt_stamp_) return std::nullopt;
    last_attempt_stamp_ = nanoseconds;
    if (!checkScanTime(snapshot.stamp, "select")) return std::nullopt;
    snapshot.scan = latest_scan_;
    snapshot.frame = latest_frame_;
    return snapshot;
  }

  void attempt() {
    const auto snapshot = takeSnapshot();
    if (!snapshot) return;
    const auto start = std::chrono::steady_clock::now();
    auto result = relocalizer_.match(snapshot->scan);
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    finishAttempt(*snapshot, result, elapsed);
  }

  // 所有结果输出共用任务/停止边界；配准本身在锁外运行。
  void finishAttempt(const AttemptSnapshot& snapshot, const MatchResult& result, double elapsed) {
    const auto& stamp = snapshot.stamp;
    std::scoped_lock lock(session_mutex_, stop_mutex_);
    if (stopping_ || !rclcpp::ok(get_node_base_interface()->get_context())) {
      log(result, stamp, "SKIP", "stopping", 0, elapsed); return;
    }
    if (snapshot.generation != generation_ || localized_) {
      log(result, stamp, "SKIP", "superseded_trigger", 0, elapsed); return;
    }
    if (!checkScanTime(stamp, "commit")) {
      confirmation_.reset();
      log(result, stamp, "SKIP", "invalid_result_time", 0, elapsed);
      return;  // 保持任务活动，等待下一帧；不能向 Manager 交付过期成功结果。
    }
    publishCloud(hypothesis_publisher_, result.hypothesis, stamp);
    publishCloud(submap_publisher_, result.submap, stamp);
    publishCloud(aligned_publisher_, result.aligned, stamp);
    if (!result.valid) { log(result, stamp, "REJECT", result.reason, 0, elapsed); return; }

    const Eigen::Matrix4d T_map_base = snapshot.frame.basePose(result.T_map_body);
    if (!ValidMapPose(T_map_base)) {
      log(result, stamp, "REJECT", "invalid_base_pose", 0, elapsed); return;
    }
    // 一致性确认与最终输出都比较底盘参考点，而非传感器安装位置。
    const int confirmed = confirmation_.observe(T_map_base);
    if (confirmed < config_.confirm_frames) {
      log(result, stamp, "PENDING", "consistency_pending", confirmed, elapsed); return;
    }
    PoseMessage message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    const auto position = T_map_base.block<3,1>(0,3);
    Eigen::Quaterniond orientation(T_map_base.block<3,3>(0,0));
    orientation.normalize();
    message.pose.pose.position.x = position.x();
    message.pose.pose.position.y = position.y();
    message.pose.pose.position.z = position.z();
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    // 参数约定为最终底盘位姿的输出方差，不是待转换的传感器配准协方差。
    for (int index : {0, 7, 14})
      message.pose.covariance[index] = config_.relocalization_position_variance;
    for (int index : {21, 28, 35})
      message.pose.covariance[index] = config_.relocalization_orientation_variance;
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
        " registration_ran=%d converged=%d nvtl=%.6f nvtl_limit=%.3f"
        " iterations=%d inlier_1m=%.3f tp=%.6f"
        " correction_m=%.3f correction_yaw_deg=%.3f submap_points=%zu"
        " confirm=%d/%d attempt_ms=%.1f reason=%s",
        outcome, rclcpp::Time(stamp).seconds(), result.candidate,
        result.sc_distance, config_.sc_dist_thres, result.registration_ran, result.converged,
        result.likelihood, config_.ndt_likelihood_threshold, result.iterations,
        result.inlier_fraction, result.transformation_probability, result.correction_m,
        result.correction_yaw_deg, result.submap_points, confirmed,
        config_.confirm_frames, elapsed, reason.c_str());
  }

  std::string base_frame_, matching_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  Config config_;
  Relocalizer relocalizer_;
  Confirmation confirmation_;
  std::mutex scan_mutex_, session_mutex_, stop_mutex_;
  Cloud::Ptr latest_scan_;
  MatchingFrame latest_frame_;
  builtin_interfaces::msg::Time latest_stamp_;
  int64_t last_attempt_stamp_ = -1;  // Worker-only; retained across service triggers.
  double requested_at_ = 0.0;  // session_mutex_ 保护；独立自动启动允许已有新鲜扫描。
  double max_reloc_age_ = 1.5, future_tolerance_ = 0.2;
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
