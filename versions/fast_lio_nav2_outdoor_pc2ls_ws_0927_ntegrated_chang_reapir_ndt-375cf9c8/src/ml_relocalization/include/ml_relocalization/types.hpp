#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <string>

namespace ml_relocalization {
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct Config {
  std::string map_directory = "/";
  std::string pose_file = "isam_poses.txt";
  double sc_dist_thres = 0.15;
  double ndt_resolution = 2.0;
  double ndt_step_size = 0.1;
  double ndt_transformation_epsilon = 0.01;
  int ndt_max_iterations = 30;
  int ndt_threads = 2;
  double ndt_likelihood_threshold = 2.3;
  double inlier_fraction_threshold = 0.4;
  double max_translation_correction = 5.0;
  double max_yaw_correction_deg = 30.0;
  double confirm_position_thresh = 2.0;
  double confirm_yaw_thresh_deg = 15.0;
  double yaw_diff_sign = -1.0;
  bool debug_publish_clouds = false;
  int submap_min_points = 1000;
  int confirm_frames = 3;
  int submap_halfwidth = 10;
  double sc_ds_size = 0.4;
  double registration_voxel_size = 0.4;
  // PoseWithCovarianceStamped diagonal entries; units m^2 and rad^2.
  double relocalization_position_variance = 0.5;
  double relocalization_orientation_variance = 0.05;
};

// T_A_B maps coordinates from B to A: p_A = T_A_B * p_B.
struct MatchResult {
  bool valid = false;
  std::string reason;
  int candidate = -1;
  double sc_distance = 0.0;
  bool registration_ran = false;
  bool converged = false;
  double likelihood = std::numeric_limits<double>::quiet_NaN();
  double transformation_probability = std::numeric_limits<double>::quiet_NaN();
  double inlier_fraction = 0.0;
  int iterations = 0;
  double correction_m = std::numeric_limits<double>::quiet_NaN();
  double correction_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  std::size_t submap_points = 0;
  // 当前扫描的 matching_frame 位姿；节点转换到底盘参考点后才对外发布。
  Eigen::Matrix4d T_map_body = Eigen::Matrix4d::Identity();
  Cloud::Ptr hypothesis, submap, aligned;  // Debug clouds in map; optional.
};
}  // namespace ml_relocalization
