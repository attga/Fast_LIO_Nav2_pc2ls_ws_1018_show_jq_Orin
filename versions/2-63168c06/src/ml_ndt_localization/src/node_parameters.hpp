#pragma once
#include "ml_ndt_localization/tracker.hpp"
#include <rclcpp/node.hpp>
#include <string>

namespace ml_ndt_loc
{
struct NodeParams
{
    std::string input_cloud_topic = "/cloud_registered_body";
    std::string map_frame = "map";
    std::string base_frame = "base_link";
    double registration_rate = 2.0;
    double scan_timeout = 2.0;
    double tf_wait_timeout = 1.0;
    // 标准差，不是方差；输出使用平方后的 SI 单位对角线。
    double position_stddev = 0.3;
    double orientation_stddev = 0.05;
    Tracker::Config tracker;
    std::string map_directory = "/";
    bool pub_map = true;
    bool wait_for_initialization = true;
    bool enter_lost_on_failure = true;
    int max_consecutive_failures = 30; // 节点决定何时将连续失败升级为 LOST。
};
NodeParams LoadNodeParams(const rclcpp::Node::SharedPtr& node);
} // namespace ml_ndt_loc
