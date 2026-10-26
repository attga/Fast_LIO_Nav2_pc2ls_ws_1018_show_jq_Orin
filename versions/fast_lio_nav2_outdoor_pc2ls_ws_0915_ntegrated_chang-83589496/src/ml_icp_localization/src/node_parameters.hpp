#pragma once

#include "ml_icp_localization/tracker.hpp"
#include <rclcpp/node.hpp>
#include <string>

namespace ml_icp_loc
{

// 节点参数(集中声明与读取,单一维护点)
struct NodeParams
{
    std::string input_cloud_topic = "/cloud_registered_body"; // 点云订阅源
    std::string map_frame = "map";         // 全局系(icp_pose/icp_path/map 的 frame)
    std::string base_frame = "base_link";  // body 系(EKF child_frame_id / 点云 frame)
    double icp_rate = 2.0;                // 节点尝试配准的调度频率，单位 Hz。
    double scan_timeout = 2.0;            // 输入/处理时效窗口，单位为 ROS 时间秒。
    double tf_wait_timeout = 1.0;         // 待处理扫描等待 TF 的上限，单位为单调时钟秒。
    double pose_covariance = 0.3;          // icp_pose 位置协方差底噪 σ(m)
    double orientation_covariance = 0.05;  // icp_pose 姿态协方差底噪 σ(rad)
    // 动态协方差(2026-09-12 定稿公式):
    //   σ_pos = max(pose_covariance, covariance_fitness_scale x fitness)
    //   σ_ori = max(orientation_covariance, covariance_fitness_scale_ori x fitness)
    double covariance_fitness_scale = 3.0;
    double covariance_fitness_scale_ori = 0.5;
    Tracker::Config tracker;               // 配准/失效计数参数
    std::string map_directory = "/";
    std::string pose_file = "isam_poses.txt";
    bool pub_map = true;
    bool wait_for_initialization = false; // 等待 Manager 通过 start_tracking 放行。
    bool enter_lost_on_failure = true; // 仅连续离线诊断时关闭自动 LOST。
};

NodeParams LoadNodeParams(const rclcpp::Node::SharedPtr& node);

} // namespace ml_icp_loc
