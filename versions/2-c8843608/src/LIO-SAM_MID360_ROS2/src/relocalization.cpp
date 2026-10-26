#include "utility.hpp"
#include "lidar_factor.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ceres/ceres.h>
#include <omp.h>
#include <filesystem>
#include <chrono>
#include <mutex>
#include <deque>

namespace fs = std::filesystem;

struct EdgeResidualData {
    Eigen::Vector3d point_curr;
    Eigen::Vector3d point_a;
    Eigen::Vector3d point_b;
};

struct PlaneResidualData {
    Eigen::Vector3d point_curr;
    Eigen::Vector3d unit_norm;
    double negative_OA_dot_norm;
};

struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) const {
        return left.value < right.value;
    }
};

class Relocalization : public ParamServer
{
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_lidar_odom;
    // sc_liorf_localization 接口: odom 系连续位姿, 供 imuPreintegration (TransformFusion / GTSAM) 消费
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_mapping_odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_mapping_odom_incremental;
    // 全局校正量 T_map_odom (campus: pose_odomTo_map)
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_odom_to_map_pose;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corner_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_corner_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_surf_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_corner_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_surf_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_registered;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_obstacles;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_status_text;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_custom_cloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_initial_odom;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    std::mutex cloud_mutex;
    std::mutex imu_mutex;
    std::deque<pcl::PointCloud<PointType>::Ptr> cloud_queue;
    std::deque<std_msgs::msg::Header> header_queue;
    std::deque<sensor_msgs::msg::Imu> imu_queue;

    pcl::PointCloud<PointType>::Ptr global_corner_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr global_surf_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr global_corner_pcl_ptr_ds;
    pcl::PointCloud<PointType>::Ptr global_surf_pcl_ptr_ds;
    pcl::PointCloud<PointType>::Ptr local_corner_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr local_surf_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr cur_corner_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr cur_surf_pcl_ptr;

    pcl::VoxelGrid<PointType> down_filter_surf;
    pcl::VoxelGrid<PointType> down_filter_corner;
    pcl::VoxelGrid<PointType> down_filter_global_surf;
    pcl::VoxelGrid<PointType> down_filter_global_corner;
    pcl::CropBox<PointType> box_filter;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_map_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_map_;

    Eigen::Matrix4f first_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pre_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    // sc_liorf_localization 风格校正分离:
    // T_odom_base 由帧间匹配增量连续积分(永不跳变), T_map_odom 吸收全部全局校正
    Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_map_odom = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d prev_raw_pose = Eigen::Isometry3d::Identity();
    bool odom_chain_init = false;
    int reject_streak = 0; // 连续超阈值帧计数(毛刺拦截状态机)
    double prev_raw_time = -1.0; // 上一帧时间戳, 用于帧间隔自适应阈值
    bool first_flag = false;
    bool submap_initialized = false;

    std::vector<float> submap_origin{0, 0, 0};
    std::vector<float> edge_left{0, 0, 0};
    std::vector<float> edge_right{0, 0, 0};
    float submap_half_size = 150.0f;
    float submap_update_thresh = 30.0f;

    std::string map_data_dir;
    nav_msgs::msg::Path global_path;
    double last_cloud_time = -1;

    // IMU integration
    Eigen::Vector3d last_imu_gyro{0, 0, 0};
    double last_imu_time = -1;
    bool imu_initialized = false;

    Eigen::Vector3f last_step_trans = Eigen::Vector3f::Zero();
    Eigen::Vector3f last_submap_center = Eigen::Vector3f::Zero();

    // Ceres parameters
    double ceres_param[7] = {0, 0, 0, 1, 0, 0, 0}; // qx, qy, qz, qw, tx, ty, tz

    // High-frequency pose state & mutex for real-time IMU extrapolation and TF broadcast
    std::mutex opt_pose_mutex;
    Eigen::Matrix4d latest_opt_pose = Eigen::Matrix4d::Identity();
    double latest_opt_time = -1.0;
    Eigen::Vector3d latest_opt_vel = Eigen::Vector3d::Zero();
    bool is_pose_initialized = false;

public:
    Relocalization(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_relocalization", options)
    {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        declare_parameter("mapDataDir", "");
        get_parameter("mapDataDir", map_data_dir);
        if (map_data_dir.empty())
        {
            char abs_path[PATH_MAX];
            if (realpath(__FILE__, abs_path) != nullptr)
            {
                std::string path_str(abs_path);
                path_str = path_str.substr(0, path_str.find_last_of("/"));
                path_str = path_str.substr(0, path_str.find_last_of("/"));
                map_data_dir = path_str + "/data/";
            }
            else
            {
                map_data_dir = savePCDDirectory;
            }
        }
        if (map_data_dir.back() != '/')
            map_data_dir += "/";

        declare_parameter("submapHalfSize", 150.0);
        double shs = 150.0;
        get_parameter("submapHalfSize", shs);
        submap_half_size = static_cast<float>(shs);

        declare_parameter("submapUpdateThresh", 30.0);
        double sut = 30.0;
        get_parameter("submapUpdateThresh", sut);
        submap_update_thresh = static_cast<float>(sut);

        auto qos_latched = rclcpp::QoS(1).transient_local().reliable();
        auto qos_reliable = rclcpp::QoS(5).reliable();

        // Publishers
        pub_lidar_odom = create_publisher<nav_msgs::msg::Odometry>("lidar_odom", qos_reliable);
        // 与 sc_liorf_localization 相同的定位输出话题, 接入 imuPreintegration 融合管线
        pub_mapping_odom = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry", qos_reliable);
        pub_mapping_odom_incremental = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry_incremental", qos_reliable);
        pub_odom_to_map_pose = create_publisher<geometry_msgs::msg::PoseStamped>("lio_sam/mapping/pose_odomTo_map", qos_reliable);
        pub_path = create_publisher<nav_msgs::msg::Path>("lidar_path", qos_reliable);
        pub_corner_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("corner_cloud", qos_reliable);
        pub_surf_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("surf_cloud", qos_reliable);
        pub_global_corner_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("global_corner_cloud", qos_latched);
        pub_global_surf_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("global_surf_cloud", qos_latched);
        pub_local_corner_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("local_corner_cloud", qos_reliable);
        pub_local_surf_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("local_surf_cloud", qos_reliable);
        pub_cloud_registered = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", qos_reliable);
        // Keep the obstacle scan in the sensor frame. Its base_link transform is static,
        // so scan conversion does not depend on a delayed map-frame pose estimate.
        pub_cloud_obstacles = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_obstacles", qos_reliable);
        pub_status_text = create_publisher<visualization_msgs::msg::Marker>("reloc_status", qos_reliable);

        // Subscribers
        sub_custom_cloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            pointCloudTopic, qos_lidar,
            std::bind(&Relocalization::customCloudCallback, this, std::placeholders::_1));

        sub_point_cloud = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_raw", qos_lidar,
            std::bind(&Relocalization::pointCloud2Callback, this, std::placeholders::_1));

        sub_imu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            std::bind(&Relocalization::imuCallback, this, std::placeholders::_1));

        sub_initial_odom = create_subscription<nav_msgs::msg::Odometry>(
            "initial_odom", qos,
            std::bind(&Relocalization::initialOdomCallback, this, std::placeholders::_1));

        sub_initialpose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&Relocalization::initialPoseCallback, this, std::placeholders::_1));

        init();
    }

    void init()
    {
        down_filter_surf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        down_filter_corner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        down_filter_global_surf.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
        down_filter_global_corner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);

        global_corner_pcl_ptr.reset(new pcl::PointCloud<PointType>());
        global_surf_pcl_ptr.reset(new pcl::PointCloud<PointType>());
        global_corner_pcl_ptr_ds.reset(new pcl::PointCloud<PointType>());
        global_surf_pcl_ptr_ds.reset(new pcl::PointCloud<PointType>());
        local_corner_pcl_ptr.reset(new pcl::PointCloud<PointType>());
        local_surf_pcl_ptr.reset(new pcl::PointCloud<PointType>());
        cur_corner_pcl_ptr.reset(new pcl::PointCloud<PointType>());
        cur_surf_pcl_ptr.reset(new pcl::PointCloud<PointType>());

        kdtree_corner_map_.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_surf_map_.reset(new pcl::KdTreeFLANN<PointType>());

        std::string sharp_map_path = map_data_dir + "sharp_cloud_map.pcd";
        std::string flat_map_path = map_data_dir + "flat_cloud_map.pcd";

        RCLCPP_INFO(get_logger(), "Loading global PCD maps: %s and %s", sharp_map_path.c_str(), flat_map_path.c_str());

        bool load_corner = fs::exists(sharp_map_path) && (pcl::io::loadPCDFile(sharp_map_path, *global_corner_pcl_ptr) == 0);
        bool load_surf = fs::exists(flat_map_path) && (pcl::io::loadPCDFile(flat_map_path, *global_surf_pcl_ptr) == 0);

        if (!load_corner || !load_surf)
        {
            RCLCPP_WARN(get_logger(), "Feature maps not found or failed to load. Checking globalMap.pcd...");
            std::string global_map_path = map_data_dir + "globalMap.pcd";
            if (fs::exists(global_map_path))
            {
                pcl::PointCloud<PointType>::Ptr raw_global(new pcl::PointCloud<PointType>());
                if (pcl::io::loadPCDFile(global_map_path, *raw_global) == 0)
                {
                    *global_surf_pcl_ptr = *raw_global;
                    *global_corner_pcl_ptr = *raw_global;
                    RCLCPP_INFO(get_logger(), "Loaded fallback global map: %s (%zu points)", global_map_path.c_str(), raw_global->size());
                }
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "No valid PCD maps found in %s! Please run mapping first.", map_data_dir.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "\033[1;32m----> Global maps loaded successfully (Corner: %zu points, Surf: %zu points).\033[0m",
                        global_corner_pcl_ptr->size(), global_surf_pcl_ptr->size());
        }

        // Downsample global maps for crisp, flicker-free visualization
        if (!global_surf_pcl_ptr->empty())
        {
            down_filter_global_surf.setInputCloud(global_surf_pcl_ptr);
            down_filter_global_surf.filter(*global_surf_pcl_ptr_ds);
            publishCloud(pub_global_surf_cloud, global_surf_pcl_ptr_ds, now(), mapFrame);
        }
        if (!global_corner_pcl_ptr->empty())
        {
            down_filter_global_corner.setInputCloud(global_corner_pcl_ptr);
            down_filter_global_corner.filter(*global_corner_pcl_ptr_ds);
            publishCloud(pub_global_corner_cloud, global_corner_pcl_ptr_ds, now(), mapFrame);
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        sensor_msgs::msg::Imu converted_imu = imuConverter(*msg);
        imu_queue.push_back(converted_imu);
        while (imu_queue.size() > 2000)
            imu_queue.pop_front();
    }

    void customCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
        scan->reserve(msg->point_num);
        PointType pt;
        for (uint i = 0; i < msg->point_num; ++i)
        {
            pt.x = msg->points[i].x;
            pt.y = msg->points[i].y;
            pt.z = msg->points[i].z;
            pt.intensity = msg->points[i].reflectivity;
            float r = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (r >= lidarMinRange && r <= lidarMaxRange)
                scan->push_back(pt);
        }

        if (pub_cloud_obstacles->get_subscription_count() > 0)
            publishCloud(pub_cloud_obstacles, scan, msg->header.stamp, lidarFrame);

        std::lock_guard<std::mutex> lock(cloud_mutex);
        cloud_queue.push_back(scan);
        header_queue.push_back(msg->header);
        while (cloud_queue.size() > 2)
        {
            cloud_queue.pop_front();
            header_queue.pop_front();
        }
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*msg, *scan);

        if (pub_cloud_obstacles->get_subscription_count() > 0)
            publishCloud(pub_cloud_obstacles, scan, msg->header.stamp, lidarFrame);

        std::lock_guard<std::mutex> lock(cloud_mutex);
        cloud_queue.push_back(scan);
        header_queue.push_back(msg->header);
        while (cloud_queue.size() > 2)
        {
            cloud_queue.pop_front();
            header_queue.pop_front();
        }
    }

    void initialOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Eigen::Quaternionf q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        first_pose.block<3, 3>(0, 0) = q.matrix();
        first_pose(0, 3) = msg->pose.pose.position.x;
        first_pose(1, 3) = msg->pose.pose.position.y;
        first_pose(2, 3) = msg->pose.pose.position.z;

        if (!first_flag)
        {
            pre_pose = first_pose;
            last_pose = first_pose;
            last_step_trans = Eigen::Vector3f::Zero();
            last_cloud_time = -1.0;
            first_flag = true;
            submap_initialized = false;

            {
                std::lock_guard<std::mutex> lock(opt_pose_mutex);
                latest_opt_pose = first_pose.cast<double>();
                latest_opt_time = stamp2Sec(msg->header.stamp);
                latest_opt_vel.setZero();
                is_pose_initialized = true;
            }

            RCLCPP_INFO(get_logger(),
                "\033[1;32m[Relocalization] Initial pose set from initial_odom: (%.2f, %.2f, %.2f)\033[0m",
                first_pose(0, 3), first_pose(1, 3), first_pose(2, 3));
        }
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        Eigen::Quaternionf q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        first_pose.block<3, 3>(0, 0) = q.matrix();
        first_pose(0, 3) = msg->pose.pose.position.x;
        first_pose(1, 3) = msg->pose.pose.position.y;
        first_pose(2, 3) = msg->pose.pose.position.z;

        pre_pose = first_pose;
        last_pose = first_pose;
        last_step_trans = Eigen::Vector3f::Zero();
        last_cloud_time = -1.0;
        first_flag = true;
        submap_initialized = false;

        {
            std::lock_guard<std::mutex> lock(opt_pose_mutex);
            latest_opt_pose = first_pose.cast<double>();
            latest_opt_time = stamp2Sec(msg->header.stamp);
            latest_opt_vel.setZero();
            is_pose_initialized = true;
        }

        RCLCPP_INFO(get_logger(),
            "\033[1;33m[Relocalization] Manual Initial pose set from /initialpose: (%.2f, %.2f, %.2f)\033[0m",
            first_pose(0, 3), first_pose(1, 3), first_pose(2, 3));
    }

    void extractFeatures(const pcl::PointCloud<PointType>::Ptr& input_cloud,
                         pcl::PointCloud<PointType>::Ptr& out_corner,
                         pcl::PointCloud<PointType>::Ptr& out_surf)
    {
        out_corner->clear();
        out_surf->clear();

        if (input_cloud->empty())
            return;

        // Voxel downsample surface points for dense, robust 6-DOF planar constraints
        down_filter_surf.setInputCloud(input_cloud);
        down_filter_surf.filter(*out_surf);

        // Voxel downsample corner points
        down_filter_corner.setInputCloud(input_cloud);
        down_filter_corner.filter(*out_corner);
    }

    void updateSubmap(const Eigen::Vector3f& center)
    {
        submap_origin = {center.x(), center.y(), center.z()};
        edge_left = {center.x() - submap_half_size, center.y() - submap_half_size, center.z() - submap_half_size};
        edge_right = {center.x() + submap_half_size, center.y() + submap_half_size, center.z() + submap_half_size};

        box_filter.setMin(Eigen::Vector4f(edge_left[0], edge_left[1], edge_left[2], -1e6f));
        box_filter.setMax(Eigen::Vector4f(edge_right[0], edge_right[1], edge_right[2], 1e6f));

        local_corner_pcl_ptr->clear();
        if (!global_corner_pcl_ptr->empty())
        {
            box_filter.setInputCloud(global_corner_pcl_ptr);
            box_filter.filter(*local_corner_pcl_ptr);
            down_filter_corner.setInputCloud(local_corner_pcl_ptr);
            down_filter_corner.filter(*local_corner_pcl_ptr);
        }

        local_surf_pcl_ptr->clear();
        if (!global_surf_pcl_ptr->empty())
        {
            box_filter.setInputCloud(global_surf_pcl_ptr);
            box_filter.filter(*local_surf_pcl_ptr);
            down_filter_surf.setInputCloud(local_surf_pcl_ptr);
            down_filter_surf.filter(*local_surf_pcl_ptr);
        }

        if (!local_corner_pcl_ptr->empty())
            kdtree_corner_map_->setInputCloud(local_corner_pcl_ptr);
        if (!local_surf_pcl_ptr->empty())
            kdtree_surf_map_->setInputCloud(local_surf_pcl_ptr);

        submap_initialized = true;

        // Publish submap only when updated to prevent flicker
        publishCloud(pub_local_corner_cloud, local_corner_pcl_ptr, now(), mapFrame);
        publishCloud(pub_local_surf_cloud, local_surf_pcl_ptr, now(), mapFrame);

        RCLCPP_DEBUG(get_logger(),
            "[Relocalization] Submap updated at (%.1f, %.1f, %.1f) | Local Corner: %zu, Local Surf: %zu",
            center.x(), center.y(), center.z(), local_corner_pcl_ptr->size(), local_surf_pcl_ptr->size());
    }

    Eigen::Matrix3d predictRotationFromImu(double from_time, double to_time)
    {
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
        if (from_time < 0 || to_time <= from_time)
            return delta_R;

        std::lock_guard<std::mutex> lock(imu_mutex);
        double prev_t = from_time;
        for (const auto& imu_msg : imu_queue)
        {
            double t = stamp2Sec(imu_msg.header.stamp);
            if (t >= from_time && t <= to_time)
            {
                double dt = t - prev_t;
                if (dt > 0 && dt < 0.1)
                {
                    Eigen::Vector3d ang_vel(imu_msg.angular_velocity.x,
                                            imu_msg.angular_velocity.y,
                                            imu_msg.angular_velocity.z);
                    Eigen::Vector3d d_theta = ang_vel * dt;
                    double angle = d_theta.norm();
                    if (angle > 1e-6)
                    {
                        Eigen::Vector3d axis = d_theta.normalized();
                        delta_R = delta_R * Eigen::AngleAxisd(angle, axis).toRotationMatrix();
                    }
                }
                prev_t = t;
            }
        }
        return delta_R;
    }

    void run()
    {
        pcl::PointCloud<PointType>::Ptr cur_cloud(new pcl::PointCloud<PointType>());
        std_msgs::msg::Header cur_header;

        {
            std::lock_guard<std::mutex> lock(cloud_mutex);
            if (cloud_queue.empty())
                return;

            cur_cloud = cloud_queue.front();
            cur_header = header_queue.front();
            cloud_queue.pop_front();
            header_queue.pop_front();
        }

        if (!first_flag)
            return;

        double cur_time = stamp2Sec(cur_header.stamp);

        // Extract features
        extractFeatures(cur_cloud, cur_corner_pcl_ptr, cur_surf_pcl_ptr);

        // Publish extracted feature clouds
        publishCloud(pub_corner_cloud, cur_corner_pcl_ptr, cur_header.stamp, lidarFrame);
        publishCloud(pub_surf_cloud, cur_surf_pcl_ptr, cur_header.stamp, lidarFrame);

        // Initialize or update submap when necessary
        float dist_to_submap = (last_pose.block<3, 1>(0, 3) - last_submap_center).norm();
        if (!submap_initialized || dist_to_submap > 10.0f)
        {
            updateSubmap(last_pose.block<3, 1>(0, 3));
            last_submap_center = last_pose.block<3, 1>(0, 3);
        }

        // Clean, single-source pose prediction
        Eigen::Matrix4f predict_pose = last_pose;

        // Rotation prediction using IMU
        if (last_cloud_time > 0)
        {
            Eigen::Matrix3d delta_R = predictRotationFromImu(last_cloud_time, cur_time);
            Eigen::Matrix3d last_R = last_pose.block<3, 3>(0, 0).cast<double>();
            predict_pose.block<3, 3>(0, 0) = (last_R * delta_R).cast<float>();
        }
        last_cloud_time = cur_time;

        // Translation prediction using constant velocity
        Eigen::Vector3f step_trans = last_step_trans;
        if (step_trans.norm() > 0.8f)
        {
            step_trans = Eigen::Vector3f::Zero();
        }
        predict_pose.block<3, 1>(0, 3) = last_pose.block<3, 1>(0, 3) + step_trans;

        // Perform Ceres scan matching against submap
        result_pose = scanMatch(cur_corner_pcl_ptr, cur_surf_pcl_ptr, local_corner_pcl_ptr, local_surf_pcl_ptr, predict_pose);

        // Update step translation and last_pose for next frame
        last_step_trans = result_pose.block<3, 1>(0, 3) - last_pose.block<3, 1>(0, 3);
        last_pose = result_pose;

        {
            std::lock_guard<std::mutex> lock(opt_pose_mutex);
            if (latest_opt_time > 0)
            {
                double dt = cur_time - latest_opt_time;
                if (dt > 0.01 && dt < 1.0)
                {
                    Eigen::Vector3d current_trans = result_pose.block<3, 1>(0, 3).cast<double>();
                    Eigen::Vector3d prev_trans = latest_opt_pose.block<3, 1>(0, 3);
                    Eigen::Vector3d inst_vel = (current_trans - prev_trans) / dt;
                    if (inst_vel.norm() < 10.0)
                    {
                        latest_opt_vel = 0.7 * inst_vel + 0.3 * latest_opt_vel;
                    }
                }
            }
            latest_opt_pose = result_pose.cast<double>();
            latest_opt_time = cur_time;
            is_pose_initialized = true;
        }

        // Publish Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = mapFrame;
        odom_msg.header.stamp = cur_header.stamp;
        // scanMatch estimates the lidar origin; TransformFusion applies
        // lidar -> base_link before publishing the Nav2 odometry.
        odom_msg.child_frame_id = lidarFrame;
        odom_msg.pose.pose.position.x = result_pose(0, 3);
        odom_msg.pose.pose.position.y = result_pose(1, 3);
        odom_msg.pose.pose.position.z = result_pose(2, 3);
        Eigen::Quaternionf q(result_pose.block<3, 3>(0, 0));
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        pub_lidar_odom->publish(odom_msg);

        // sc_liorf_localization 风格校正分离: 帧间匹配增量在物理范围内 -> 积分进 T_odom_base;
        // 超阈值判定为校正帧(重定位/跟踪重置) -> 不积分, 跳变全部吸收进 T_map_odom
        Eigen::Isometry3d raw_pose = Eigen::Isometry3d(result_pose.cast<double>());
        if (!odom_chain_init) {
            // 首帧: odom 系与 map 系重合
            T_odom_base = raw_pose;
            T_map_odom.setIdentity();
            odom_chain_init = true;
        } else {
            Eigen::Isometry3d delta = prev_raw_pose.inverse() * raw_pose;
            double d_trans = delta.translation().norm();
            double d_rot = Eigen::AngleAxisd(delta.rotation()).angle();
            // 毛刺拦截: 增量超物理上限(0.8m/s × 实际帧间隔; 实测匹配瞬时退化伪增量约12cm
            // 且单帧即回弹)时不积分, 跳变吸收进 T_map_odom; 连续 5 帧超阈值判定为真实
            // 高速运动, 恢复积分防止 odom 链长期冻结(此前真实位移由 T_map_odom 恒定吸收)
            double frame_dt = (prev_raw_time > 0) ? std::max(cur_time - prev_raw_time, 1e-3) : 0.1;
            double dyn_thresh = std::max(0.08, 0.8 * frame_dt);
            if (d_trans < dyn_thresh && d_rot < 0.3)
            {
                T_odom_base = T_odom_base * delta;
                reject_streak = 0;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[Glitch Reject] d_trans=%.3f (thres %.3f), d_rot=%.3f, streak=%d",
                    d_trans, dyn_thresh, d_rot, reject_streak + 1);
                if (++reject_streak >= 5)
                    T_odom_base = T_odom_base * delta;
            }
            T_map_odom = raw_pose * T_odom_base.inverse();
        }
        prev_raw_pose = raw_pose;
        prev_raw_time = cur_time;

        // 供 imuPreintegration 的 odom 系连续位姿(锚点连续, 原版锚定融合无泄漏):
        // TransformFusion 订阅 mapping/odometry, GTSAM 预积分订阅 odometry_incremental
        nav_msgs::msg::Odometry odom_frame_msg;
        odom_frame_msg.header.frame_id = odometryFrame;
        odom_frame_msg.header.stamp = cur_header.stamp;
        odom_frame_msg.child_frame_id = lidarFrame;
        odom_frame_msg.pose.pose.position.x = T_odom_base(0, 3);
        odom_frame_msg.pose.pose.position.y = T_odom_base(1, 3);
        odom_frame_msg.pose.pose.position.z = T_odom_base(2, 3);
        Eigen::Quaterniond q_odom(T_odom_base.rotation());
        odom_frame_msg.pose.pose.orientation.w = q_odom.w();
        odom_frame_msg.pose.pose.orientation.x = q_odom.x();
        odom_frame_msg.pose.pose.orientation.y = q_odom.y();
        odom_frame_msg.pose.pose.orientation.z = q_odom.z();
        pub_mapping_odom->publish(odom_frame_msg);
        pub_mapping_odom_incremental->publish(odom_frame_msg);

        // 全局校正量 (campus: pose_odomTo_map), TransformFusion 以此为 map -> odom TF 数据源
        geometry_msgs::msg::PoseStamped pose_odom_to_map;
        pose_odom_to_map.header.frame_id = mapFrame;
        pose_odom_to_map.header.stamp = cur_header.stamp;
        pose_odom_to_map.pose.position.x = T_map_odom(0, 3);
        pose_odom_to_map.pose.position.y = T_map_odom(1, 3);
        pose_odom_to_map.pose.position.z = T_map_odom(2, 3);
        Eigen::Quaterniond q_m2o(T_map_odom.rotation());
        pose_odom_to_map.pose.orientation.w = q_m2o.w();
        pose_odom_to_map.pose.orientation.x = q_m2o.x();
        pose_odom_to_map.pose.orientation.y = q_m2o.y();
        pose_odom_to_map.pose.orientation.z = q_m2o.z();
        pub_odom_to_map_pose->publish(pose_odom_to_map);

        // 注意: 不再广播 map -> base_link 直连 TF (违反 REP-105, 与 odom -> base_link 冲突)。
        // TF 链由 lio_sam_imuPreintegration 节点按 sc_liorf_localization 方式维护:
        //   map -> odom (全局校正量) + odom -> base_link (200Hz IMU 积分, 平滑不跳变)



        // Publish Registered Cloud in Map Frame
        if (pub_cloud_registered->get_subscription_count() > 0 && !cur_cloud->empty())
        {
            pcl::PointCloud<PointType>::Ptr reg_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cur_cloud, *reg_cloud, result_pose);
            publishCloud(pub_cloud_registered, reg_cloud, cur_header.stamp, mapFrame);
        }

        // Publish Markers at 10Hz to avoid CPU & rendering load
        static int marker_count = 0;
        if (++marker_count % 10 == 0)
        {
            if (pub_status_text->get_subscription_count() > 0)
            {
                visualization_msgs::msg::Marker text_marker;
                text_marker.header.frame_id = mapFrame;
                text_marker.header.stamp = cur_header.stamp;
                text_marker.ns = "reloc_status";
                text_marker.id = 1;
                text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                text_marker.action = visualization_msgs::msg::Marker::ADD;
                text_marker.pose.position.x = result_pose(0, 3);
                text_marker.pose.position.y = result_pose(1, 3);
                text_marker.pose.position.z = result_pose(2, 3) + 1.2;
                text_marker.scale.z = 0.4;
                text_marker.color.r = 0.2f;
                text_marker.color.g = 1.0f;
                text_marker.color.b = 0.4f;
                text_marker.color.a = 0.95f;
                text_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

                char status_buf[256];
                snprintf(status_buf, sizeof(status_buf),
                         "RELOCALIZED (6-DOF Tracking)\nSubmap: %zu pts | Pos: (%.2f, %.2f, %.2f)",
                         local_corner_pcl_ptr->size() + local_surf_pcl_ptr->size(),
                         result_pose(0, 3), result_pose(1, 3), result_pose(2, 3));
                text_marker.text = std::string(status_buf);
                pub_status_text->publish(text_marker);
            }
        }

        // Update Path
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header;
        pose_stamped.pose = odom_msg.pose.pose;
        global_path.header = odom_msg.header;
        global_path.poses.push_back(pose_stamped);
        if (global_path.poses.size() > 2000)
            global_path.poses.erase(global_path.poses.begin());
        pub_path->publish(global_path);

        // Check if robot is near submap boundary, update if needed
        float cur_x = result_pose(0, 3);
        float cur_y = result_pose(1, 3);
        float cur_z = result_pose(2, 3);

        if (std::abs(cur_x - edge_left[0]) < submap_update_thresh || std::abs(cur_x - edge_right[0]) < submap_update_thresh ||
            std::abs(cur_y - edge_left[1]) < submap_update_thresh || std::abs(cur_y - edge_right[1]) < submap_update_thresh ||
            std::abs(cur_z - edge_left[2]) < submap_update_thresh || std::abs(cur_z - edge_right[2]) < submap_update_thresh)
        {
            updateSubmap(Eigen::Vector3f(cur_x, cur_y, cur_z));
        }
    }

    Eigen::Matrix4f scanMatch(const pcl::PointCloud<PointType>::Ptr& corner_ptr,
                              const pcl::PointCloud<PointType>::Ptr& surf_ptr,
                              const pcl::PointCloud<PointType>::Ptr& local_corner_ptr,
                              const pcl::PointCloud<PointType>::Ptr& local_surf_ptr,
                              const Eigen::Matrix4f& predict_pose)
    {
        Eigen::Matrix3d predict_rot_mat = predict_pose.block<3, 3>(0, 0).cast<double>();
        Eigen::Vector3d predict_trans(predict_pose(0, 3), predict_pose(1, 3), predict_pose(2, 3));

        Eigen::Quaterniond q_w_curr(predict_rot_mat);
        q_w_curr.normalize();
        Eigen::Vector3d t_w_curr = predict_trans;

        ceres_param[0] = q_w_curr.x();
        ceres_param[1] = q_w_curr.y();
        ceres_param[2] = q_w_curr.z();
        ceres_param[3] = q_w_curr.w();
        ceres_param[4] = t_w_curr.x();
        ceres_param[5] = t_w_curr.y();
        ceres_param[6] = t_w_curr.z();

        for (int iter_count = 0; iter_count < 2; ++iter_count)
        {
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();

            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(ceres_param, 4, q_parameterization);
            problem.AddParameterBlock(ceres_param + 4, 3);

            q_w_curr = Eigen::Quaterniond(ceres_param[3], ceres_param[0], ceres_param[1], ceres_param[2]);
            t_w_curr = Eigen::Vector3d(ceres_param[4], ceres_param[5], ceres_param[6]);

            // 1. Parallel Edge constraints (OpenMP multi-threaded search)
            std::vector<EdgeResidualData> valid_edges;
            int laser_corner_num = static_cast<int>(corner_ptr->size());
            if (!local_corner_ptr->empty() && laser_corner_num > 0)
            {
                valid_edges.reserve(laser_corner_num);
                #pragma omp parallel
                {
                    std::vector<EdgeResidualData> local_edges;
                    std::vector<int> point_search_ind;
                    std::vector<float> point_search_dist;

                    #pragma omp for nowait
                    for (int i = 0; i < laser_corner_num; ++i)
                    {
                        const auto& point_ori = corner_ptr->points[i];
                        Eigen::Vector3d point_curr(point_ori.x, point_ori.y, point_ori.z);
                        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;

                        PointType point_sel;
                        point_sel.x = point_w.x();
                        point_sel.y = point_w.y();
                        point_sel.z = point_w.z();

                        kdtree_corner_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_dist);

                        if (point_search_dist.size() >= 5 && point_search_dist[4] < 1.0f)
                        {
                            Eigen::Vector3d center(0, 0, 0);
                            std::vector<Eigen::Vector3d> near_corners;
                            near_corners.reserve(5);
                            for (int j = 0; j < 5; ++j)
                            {
                                const auto& pt = local_corner_ptr->points[point_search_ind[j]];
                                Eigen::Vector3d temp(pt.x, pt.y, pt.z);
                                center += temp;
                                near_corners.push_back(temp);
                            }
                            center /= 5.0;

                            Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
                            for (int j = 0; j < 5; ++j)
                            {
                                Eigen::Vector3d zero_mean = near_corners[j] - center;
                                cov_mat += zero_mean * zero_mean.transpose();
                            }

                            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
                            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                            {
                                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                                Eigen::Vector3d point_a = 0.1 * unit_direction + center;
                                Eigen::Vector3d point_b = -0.1 * unit_direction + center;
                                local_edges.push_back({point_curr, point_a, point_b});
                            }
                        }
                    }

                    #pragma omp critical
                    {
                        valid_edges.insert(valid_edges.end(), local_edges.begin(), local_edges.end());
                    }
                }

                for (const auto& edge : valid_edges)
                {
                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(edge.point_curr, edge.point_a, edge.point_b, 1.0);
                    problem.AddResidualBlock(cost_function, loss_function, ceres_param, ceres_param + 4);
                }
            }

            // 2. Parallel Plane constraints (OpenMP multi-threaded search)
            std::vector<PlaneResidualData> valid_planes;
            int laser_surf_num = static_cast<int>(surf_ptr->size());
            if (!local_surf_ptr->empty() && laser_surf_num > 0)
            {
                valid_planes.reserve(laser_surf_num);
                #pragma omp parallel
                {
                    std::vector<PlaneResidualData> local_planes;
                    std::vector<int> point_search_ind;
                    std::vector<float> point_search_dist;

                    #pragma omp for nowait
                    for (int i = 0; i < laser_surf_num; ++i)
                    {
                        const auto& point_ori = surf_ptr->points[i];
                        Eigen::Vector3d point_curr(point_ori.x, point_ori.y, point_ori.z);
                        Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;

                        PointType point_sel;
                        point_sel.x = point_w.x();
                        point_sel.y = point_w.y();
                        point_sel.z = point_w.z();

                        kdtree_surf_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_dist);

                        if (point_search_dist.size() >= 5 && point_search_dist[4] < 1.0f)
                        {
                            Eigen::Vector3d center(0, 0, 0);
                            std::vector<Eigen::Vector3d> near_surfs;
                            near_surfs.reserve(5);
                            for (int j = 0; j < 5; ++j)
                            {
                                const auto& pt = local_surf_ptr->points[point_search_ind[j]];
                                Eigen::Vector3d temp(pt.x, pt.y, pt.z);
                                center += temp;
                                near_surfs.push_back(temp);
                            }
                            center /= 5.0;

                            Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
                            for (int j = 0; j < 5; ++j)
                            {
                                Eigen::Vector3d zero_mean = near_surfs[j] - center;
                                cov_mat += zero_mean * zero_mean.transpose();
                            }

                            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
                            if (saes.eigenvalues()[0] < 0.25 * saes.eigenvalues()[1])
                            {
                                Eigen::Vector3d unit_norm = saes.eigenvectors().col(0);
                                double negative_OA_dot_norm = -unit_norm.dot(center);

                                bool plane_valid = true;
                                for (int j = 0; j < 5; ++j)
                                {
                                    double dist = std::abs(unit_norm.dot(near_surfs[j]) + negative_OA_dot_norm);
                                    if (dist > 0.15)
                                    {
                                        plane_valid = false;
                                        break;
                                    }
                                }

                                if (plane_valid)
                                {
                                    local_planes.push_back({point_curr, unit_norm, negative_OA_dot_norm});
                                }
                            }
                        }
                    }

                    #pragma omp critical
                    {
                        valid_planes.insert(valid_planes.end(), local_planes.begin(), local_planes.end());
                    }
                }

                for (const auto& plane : valid_planes)
                {
                    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(plane.point_curr, plane.unit_norm, plane.negative_OA_dot_norm);
                    problem.AddResidualBlock(cost_function, loss_function, ceres_param, ceres_param + 4);
                }
            }

            if (valid_edges.size() + valid_planes.size() < 10)
            {
                break;
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.num_threads = numberOfCores;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }

        q_w_curr = Eigen::Quaterniond(ceres_param[3], ceres_param[0], ceres_param[1], ceres_param[2]);
        q_w_curr.normalize();
        t_w_curr = Eigen::Vector3d(ceres_param[4], ceres_param[5], ceres_param[6]);

        Eigen::Matrix4f tmp_pose = Eigen::Matrix4f::Identity();
        tmp_pose.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix().cast<float>();
        tmp_pose(0, 3) = static_cast<float>(t_w_curr.x());
        tmp_pose(1, 3) = static_cast<float>(t_w_curr.y());
        tmp_pose(2, 3) = static_cast<float>(t_w_curr.z());

        return tmp_pose;
    }

    void runThread()
    {
        while (rclcpp::ok())
        {
            bool has_cloud = false;
            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                has_cloud = !cloud_queue.empty();
            }
            if (has_cloud)
            {
                run();
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }

    void visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(1.0); // 1Hz check rate
        int count = 0;
        while (rclcpp::ok())
        {
            try
            {
                rate.sleep();
            }
            catch (const std::exception&)
            {
                break;
            }

            if (!rclcpp::ok()) break;

            count++;
            // Publish frequently (1Hz) during the first 10 seconds for instant RViz2 display, then every 3s
            if (count <= 10 || count % 3 == 0 ||
                pub_global_surf_cloud->get_subscription_count() > 0 ||
                pub_global_corner_cloud->get_subscription_count() > 0)
            {
                if (!global_surf_pcl_ptr_ds->empty())
                    publishCloud(pub_global_surf_cloud, global_surf_pcl_ptr_ds, now(), mapFrame);
                if (!global_corner_pcl_ptr_ds->empty())
                    publishCloud(pub_global_corner_cloud, global_corner_pcl_ptr_ds, now(), mapFrame);
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);

    auto node = std::make_shared<Relocalization>(options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> LIO-SAM Mid-360 Relocalization Node Started.\033[0m");

    std::thread run_thread(&Relocalization::runThread, node);
    std::thread map_vis_thread(&Relocalization::visualizeGlobalMapThread, node);

    rclcpp::spin(node);

    rclcpp::shutdown();

    if (run_thread.joinable())
        run_thread.join();
    if (map_vis_thread.joinable())
        map_vis_thread.join();

    return 0;
}
