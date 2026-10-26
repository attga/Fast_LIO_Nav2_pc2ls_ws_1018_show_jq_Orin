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
  double icp_fitness_thres = 0.3;
  double icp_t_max = 5.0;
  double icp_yaw_max_deg = 30.0;
  double icp_max_corr_dist = 3.0;
  double confirm_position_thresh = 2.0;
  double confirm_yaw_thresh_deg = 15.0;
  double yaw_diff_sign = -1.0;
  bool debug_publish_clouds = false;
  int submap_min_points = 1000;
  int confirm_frames = 3;
  int submap_halfwidth = 10;
  double sc_ds_size = 0.4;
  double icp_ds_size = 0.4;
};

// T_A_B maps coordinates from B to A: p_A = T_A_B * p_B.
struct MatchResult {
  bool valid = false;
  std::string reason;
  int candidate = -1;
  double sc_distance = 0.0;
  bool icp_ran = false;
  bool converged = false;
  double fitness = std::numeric_limits<double>::quiet_NaN();
  double correction_m = std::numeric_limits<double>::quiet_NaN();
  double correction_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  std::size_t submap_points = 0;
  Eigen::Matrix4d T_map_body = Eigen::Matrix4d::Identity();
  Cloud::Ptr hypothesis, submap, aligned;  // Debug clouds in map; optional.
};
}  // namespace ml_relocalization
