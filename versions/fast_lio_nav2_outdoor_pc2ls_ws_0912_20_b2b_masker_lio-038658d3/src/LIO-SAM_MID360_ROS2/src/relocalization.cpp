#include "utility.hpp"
#include "lidar_factor.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ceres/ceres.h>
// ---- GTSAM：标准 LIO-SAM IMU 预积分因子图(与 imuPreintegration.cpp 一致)----
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <omp.h>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <mutex>
#include <deque>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

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
    // 发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_lidar_odom;      // 全局里程计 (map)
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_local_odom;      // 局部里程计 (odom)  -- 新增
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corner_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_corner_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_surf_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_corner_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_surf_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_registered;

    // 订阅器
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_custom_cloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_initial_odom;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // base_link <- lidar(livox_frame) 外参：scanMatch 得到的是 T_map_lidar，而对外发布的
    // 里程计/TF 是 base_link，需右乘 T_lidar_base。优先从 TF(robot_state_publisher 按 URDF
    // 发布的静态变换)解析，保证与 URDF 单一真值一致；TF 不可用时用构造函数里的回退值。
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    Eigen::Matrix4f T_lidar_base_ = Eigen::Matrix4f::Identity();
    bool extrinsic_resolved_ = false;

    std::mutex cloud_mutex;
    std::mutex imu_mutex;
    std::mutex state_mutex;
    std::deque<pcl::PointCloud<PointType>::Ptr> cloud_queue;
    std::deque<std_msgs::msg::Header> header_queue;
    std::deque<sensor_msgs::msg::Imu> imu_queue;

    // 点云指针
    pcl::PointCloud<PointType>::Ptr global_corner_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr global_surf_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr global_corner_pcl_ptr_ds;
    pcl::PointCloud<PointType>::Ptr global_surf_pcl_ptr_ds;
    pcl::PointCloud<PointType>::Ptr local_corner_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr local_surf_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr cur_corner_pcl_ptr;
    pcl::PointCloud<PointType>::Ptr cur_surf_pcl_ptr;

    // 滤波器
    pcl::VoxelGrid<PointType> down_filter_surf;
    pcl::VoxelGrid<PointType> down_filter_corner;
    pcl::VoxelGrid<PointType> down_filter_global_surf;
    pcl::VoxelGrid<PointType> down_filter_global_corner;
    pcl::CropBox<PointType> box_filter;

    // KD树
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_map_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_map_;

    // 位姿状态
    Eigen::Matrix4f first_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    bool first_flag = false;
    bool submap_initialized = false;

    // 局部里程计原点（地图坐标系下的起始位姿）
    Eigen::Matrix4f odom_origin_ = Eigen::Matrix4f::Identity();
    // true: 局部里程计原点已经锁定为“第一次实际定位结果”
    // 注意：不能直接使用 initial_odom / initialpose，因为第一次 scanMatch 后的结果
    // 可能与初始位姿存在微小偏差；否则 local_lidar_odom 初值不会严格为 0。
    bool local_odom_origin_initialized_ = false;

    // 子地图参数
    std::vector<float> submap_origin{0, 0, 0};
    std::vector<float> edge_left{0, 0, 0};
    std::vector<float> edge_right{0, 0, 0};
    float submap_half_size = 150.0f;
    float submap_update_thresh = 30.0f;

    std::string map_data_dir;
    nav_msgs::msg::Path global_path;
    double last_cloud_time = -1;

    Eigen::Vector3f last_step_trans = Eigen::Vector3f::Zero();
    Eigen::Vector3f last_submap_center = Eigen::Vector3f::Zero();

    // Ceres参数
    double ceres_param[7] = {0, 0, 0, 1, 0, 0, 0};

    // ---- 性能调优（弱设备可下调 max*_pts_ 提速；对应 params.yaml relocMax*Pts）----
    int max_corner_pts_ = 1000;   // scanMatch 每帧最多处理的角点数(均匀跨步封顶)
    int max_surf_pts_ = 2000;     // scanMatch 每帧最多处理的平面点数
    int ceres_num_threads_ = 1;   // = min(numberOfCores, 硬件并发)，避免弱设备线程超订
    float max_translation_jump_ = 5.0f;   // scanMatch 单帧平移跳变门限(m)，超出回退到预测位姿(params.yaml relocMaxTranslationJump)

    // ---- 高频局部里程计发布频率(按 localOdomRate 发布 local_lidar_odom)----
    double hr_rate_ = 100.0;                                      // 发布频率(Hz)

    // ---- 标准 LIO-SAM 方法：GTSAM IMU 预积分因子图，联合优化 位姿/速度/零偏 ----
    // 与建图管线 imuPreintegration.cpp 的 IMUPreintegration 一致：以 scanMatch 位姿(map 系
    // T_map_lidar)作为位姿观测因子，IMU 预积分因子连接相邻帧，ISAM2 增量优化得到 NavState
    // (含速度)与 imuBias(含陀螺零偏)。线速度取 NavState.velocity()，角速度取去零偏后的陀螺。
    // 运行在 map 系(建图地图已重力对齐，避免 odom 系随首帧姿态倾斜导致重力错位)。
    // 全部预积分状态在 preint_mutex_ 保护下访问(imuCallback/run/hrOdomThread 共享)。
    std::mutex preint_mutex_;
    bool systemInitialized_ = false;
    bool doneFirstOpt_ = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_;
    gtsam::Vector noiseModelBetweenBias_;

    gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_ = nullptr;
    gtsam::PreintegratedImuMeasurements* imuIntegratorImu_ = nullptr;

    std::deque<sensor_msgs::msg::Imu> imuQueOpt_;
    std::deque<sensor_msgs::msg::Imu> imuQueImu_;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_ = gtsam::Vector3(0, 0, 0);
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::NavState prevStateOdom_;
    gtsam::imuBias::ConstantBias prevBiasOdom_;

    double lastImuT_imu_ = -1;
    double lastImuT_opt_ = -1;

    gtsam::ISAM2 optimizer_;
    gtsam::NonlinearFactorGraph graphFactors_;
    gtsam::Values graphValues_;
    int key_ = 1;
    const double delta_t_ = 0;

    gtsam::Pose3 imu2Lidar_;
    gtsam::Pose3 lidar2Imu_;

    // map->odom 原点快照(= 首次 scanMatch 的 T_map_base)，供 hrOdomThread 把预测的 map 位姿转到 odom 系
    Eigen::Matrix4f preint_odom_origin_ = Eigen::Matrix4f::Identity();
    bool preint_odom_origin_valid_ = false;
    // T_lidar_base 快照：hrOdomThread 跨线程读取，避免与 run()/resolveBaseLidarExtrinsic 的写入竞争
    Eigen::Matrix4f preint_T_lidar_base_ = Eigen::Matrix4f::Identity();

    // ---- 预积分噪声/门限可调参数(对应 params.yaml relocImu*；默认值与移植前硬编码一致)----
    double reloc_imu_prior_pose_sigma_ = 1e-2;    // 首帧/重置位姿先验 sigma(rad,rad,rad,m,m,m)
    double reloc_imu_prior_vel_sigma_ = 1e4;      // 速度先验 sigma(m/s)，大值=弱先验
    double reloc_imu_prior_bias_sigma_ = 1e-3;    // 零偏先验 sigma
    double reloc_imu_correct_rot_sigma_ = 0.05;   // scanMatch 位姿观测旋转 sigma(rad)
    double reloc_imu_correct_trans_sigma_ = 0.1;  // scanMatch 位姿观测平移 sigma(m)
    double reloc_imu_integration_sigma_ = 1e-4;   // 预积分积分误差 sigma
    double reloc_imu_max_velocity_ = 30.0;        // 失败检测：速度上限(m/s)
    double reloc_imu_max_bias_ = 1.0;             // 失败检测：零偏上限

public:
    Relocalization(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_relocalization", options)
    {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // 回退外参(须与 URDF lidar_joint 一致)：雷达在 base 系 (-0.2,0,+0.4) → base 在雷达系 (0.2,0,-0.4)
        T_lidar_base_ = Eigen::Matrix4f::Identity();
        T_lidar_base_(0, 3) = 0.2f;
        T_lidar_base_(1, 3) = 0.0f;
        T_lidar_base_(2, 3) = -0.4f;

        // 参数声明与获取
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
        if (!map_data_dir.empty() && map_data_dir.back() != '/')
            map_data_dir += "/";

        declare_parameter("submapHalfSize", 150.0);
        double shs = 150.0;
        get_parameter("submapHalfSize", shs);
        submap_half_size = static_cast<float>(shs);

        declare_parameter("submapUpdateThresh", 30.0);
        double sut = 30.0;
        get_parameter("submapUpdateThresh", sut);
        submap_update_thresh = static_cast<float>(sut);

        // ---- 性能调优参数（弱设备可下调以提速；params.yaml 不写则用这些默认值）----
        declare_parameter("relocMaxCornerPts", 1000);
        get_parameter("relocMaxCornerPts", max_corner_pts_);
        declare_parameter("relocMaxSurfPts", 2000);
        get_parameter("relocMaxSurfPts", max_surf_pts_);
        if (max_corner_pts_ < 50) max_corner_pts_ = 50;
        if (max_surf_pts_ < 50) max_surf_pts_ = 50;

        // Ceres 线程数不超过硬件并发，避免弱设备上线程超订反而变慢
        int hw_conc = static_cast<int>(std::thread::hardware_concurrency());
        ceres_num_threads_ = std::max(1, std::min(numberOfCores, hw_conc > 0 ? hw_conc : 1));

        // 高频局部里程计发布频率(IMU 外推)，限幅到 [10,400] Hz
        declare_parameter("localOdomRate", 100.0);
        double lor = 100.0;
        get_parameter("localOdomRate", lor);
        hr_rate_ = std::min(400.0, std::max(10.0, lor));

        // scanMatch 单帧平移跳变门限(超出回退到 IMU 预测位姿)，限幅到 [0.5,50] m
        declare_parameter("relocMaxTranslationJump", 5.0);
        double mtj = 5.0;
        get_parameter("relocMaxTranslationJump", mtj);
        max_translation_jump_ = static_cast<float>(std::min(50.0, std::max(0.5, mtj)));

        auto qos_latched = rclcpp::QoS(1).transient_local().reliable();
        auto qos_reliable = rclcpp::QoS(5).reliable();

        // 创建发布器
        pub_lidar_odom = create_publisher<nav_msgs::msg::Odometry>("lidar_odom", qos_reliable);            // 全局
        pub_local_odom  = create_publisher<nav_msgs::msg::Odometry>("local_lidar_odom", qos_reliable);     // 局部（新增）
        pub_path = create_publisher<nav_msgs::msg::Path>("lidar_path", qos_reliable);
        pub_corner_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("corner_cloud", qos_reliable);
        pub_surf_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("surf_cloud", qos_reliable);
        pub_global_corner_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("global_corner_cloud", qos_latched);
        pub_global_surf_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("global_surf_cloud", qos_latched);
        pub_local_corner_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("local_corner_cloud", qos_reliable);
        pub_local_surf_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("local_surf_cloud", qos_reliable);
        pub_cloud_registered = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", qos_reliable);

        // 订阅器
        sub_custom_cloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            pointCloudTopic, qos_lidar,
            std::bind(&Relocalization::customCloudCallback, this, std::placeholders::_1));

        sub_point_cloud = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_raw", qos_lidar,
            std::bind(&Relocalization::pointCloud2Callback, this, std::placeholders::_1));

        sub_imu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            std::bind(&Relocalization::imuCallback, this, std::placeholders::_1));

        // Latched to match sc_pose's transient_local publisher: guarantees this node receives the
        // (single-shot) initial_odom handshake even if sc_pose publishes before we are matched.
        sub_initial_odom = create_subscription<nav_msgs::msg::Odometry>(
            "initial_odom", qos_latched,
            std::bind(&Relocalization::initialOdomCallback, this, std::placeholders::_1));

        sub_initialpose = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&Relocalization::initialPoseCallback, this, std::placeholders::_1));

        // 初始化标准 LIO-SAM 的 GTSAM IMU 预积分因子图(噪声模型/积分器/IMU<-lidar 外参)
        initPreintegration();

        init();
    }

    // 去除 NaN/Inf 及远离地图的极端离群点（单遍原地压缩）。返回被删除的点数。
    // 目的：避免 pcl::VoxelGrid 因包围盒过大触发整型索引溢出，并保证 KD 树/scanMatch 输入干净。
    size_t sanitizeGlobalMap(const pcl::PointCloud<PointType>::Ptr& cloud)
    {
        if (!cloud || cloud->empty())
            return 0;
        const size_t orig = cloud->size();
        const float kMaxAbs = 1.0e4f;   // 10km：远超真实地图范围（本图约 1.5km），只剔除非法/极端点
        size_t kept = 0;
        for (size_t i = 0; i < orig; ++i)
        {
            const PointType& p = cloud->points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            if (std::fabs(p.x) > kMaxAbs || std::fabs(p.y) > kMaxAbs || std::fabs(p.z) > kMaxAbs)
                continue;
            cloud->points[kept++] = p;
        }
        cloud->points.resize(kept);
        cloud->width = static_cast<uint32_t>(kept);
        cloud->height = 1;
        cloud->is_dense = true;
        return orig - kept;
    }

    void init()
    {
        down_filter_surf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        down_filter_corner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        down_filter_global_surf.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
        // 全局角点可视化也用可视化 leaf(1.0)。原先误用 mappingCornerLeafSize(0.1)：
        // 对整张地图(~569x296x142m)做 0.1m 体素会得到 5690*2958*1420≈2.4e10 个体素，
        // 超过 int 上限 → pcl::VoxelGrid 报 "Integer indices would overflow" 并跳过滤波。
        down_filter_global_corner.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);

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
            RCLCPP_WARN(get_logger(), "Feature maps not found or failed to load. Checking cloudGlobal.pcd...");
            std::string global_map_path = map_data_dir + "cloudGlobal.pcd";
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

        // 清理 NaN/Inf 与极端离群点：否则 pcl::VoxelGrid 会因包围盒过大导致整型索引溢出
        // ("Leaf size is too small ... Integer indices would overflow") 而使全局地图可视化失败；
        // 同时也保证用于 scanMatch 的局部子图/KD 树不含非法点。
        size_t removed_surf = sanitizeGlobalMap(global_surf_pcl_ptr);
        size_t removed_corner = sanitizeGlobalMap(global_corner_pcl_ptr);
        if (removed_surf || removed_corner)
            RCLCPP_WARN(get_logger(),
                "[Reloc] Sanitized global maps: removed %zu surf + %zu corner NaN/outlier points (remain Surf: %zu, Corner: %zu).",
                removed_surf, removed_corner, global_surf_pcl_ptr->size(), global_corner_pcl_ptr->size());

        // 可视化全局地图（降采样后）
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

    // ---------- 回调函数 ----------
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // imuConverter 只在本回调(单线程 executor)调用，无需加锁；随后分别送入 scanMatch 队列与预积分队列。
        sensor_msgs::msg::Imu converted_imu = imuConverter(*msg);
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            imu_queue.push_back(converted_imu);
            while (imu_queue.size() > 2000)
                imu_queue.pop_front();
        }
        // 标准 LIO-SAM：把 IMU 送入预积分队列，并对高频积分器 imuIntegratorImu_ 累积一帧(供 predict)
        {
            std::lock_guard<std::mutex> lock(preint_mutex_);
            imuQueOpt_.push_back(converted_imu);
            imuQueImu_.push_back(converted_imu);
            while (imuQueOpt_.size() > 5000) imuQueOpt_.pop_front();
            while (imuQueImu_.size() > 5000) imuQueImu_.pop_front();
            if (doneFirstOpt_)
            {
                double imuTime = stamp2Sec(converted_imu.header.stamp);
                double dt = (lastImuT_imu_ < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu_);
                lastImuT_imu_ = imuTime;
                imuIntegratorImu_->integrateMeasurement(
                    gtsam::Vector3(converted_imu.linear_acceleration.x, converted_imu.linear_acceleration.y, converted_imu.linear_acceleration.z),
                    gtsam::Vector3(converted_imu.angular_velocity.x, converted_imu.angular_velocity.y, converted_imu.angular_velocity.z),
                    dt);
            }
        }
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
        // Shares pose state with run() (runThread); serialize on state_mutex to avoid a data race.
        std::lock_guard<std::mutex> state_lock(state_mutex);

        Eigen::Quaternionf q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        first_pose.block<3, 3>(0, 0) = q.matrix();
        first_pose(0, 3) = msg->pose.pose.position.x;
        first_pose(1, 3) = msg->pose.pose.position.y;
        first_pose(2, 3) = msg->pose.pose.position.z;

        if (!first_flag)
        {
            last_pose = first_pose;
            last_step_trans = Eigen::Vector3f::Zero();
            last_cloud_time = -1.0;
            first_flag = true;
            submap_initialized = false;

            // 初始定位只用于确定地图中的全局起点。
            // 局部里程计原点在第一次 scanMatch 得到实际定位结果后再锁定，
            // 这样可以保证 local_lidar_odom 的第一帧严格从 (0,0,0) 开始。
            odom_origin_ = first_pose;
            local_odom_origin_initialized_ = false;
            resetPreintegration();   // 重新定位：重置 GTSAM 预积分，等待用新首帧 scanMatch 位姿重新初始化

            RCLCPP_INFO(get_logger(),
                "\033[1;32m[Relocalization] Initial pose set from initial_odom: (%.2f, %.2f, %.2f)\033[0m",
                first_pose(0, 3), first_pose(1, 3), first_pose(2, 3));
        }
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Shares pose state with run() (runThread); serialize on state_mutex to avoid a data race.
        std::lock_guard<std::mutex> state_lock(state_mutex);

        Eigen::Quaternionf q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        first_pose.block<3, 3>(0, 0) = q.matrix();
        first_pose(0, 3) = msg->pose.pose.position.x;
        first_pose(1, 3) = msg->pose.pose.position.y;
        first_pose(2, 3) = msg->pose.pose.position.z;

        last_pose = first_pose;
        last_step_trans = Eigen::Vector3f::Zero();
        last_cloud_time = -1.0;
        first_flag = true;
        submap_initialized = false;

        // 初始定位只用于确定地图中的全局起点。
        // 局部里程计原点在第一次 scanMatch 得到实际定位结果后再锁定，
        // 这样可以保证 local_lidar_odom 的第一帧严格从 (0,0,0) 开始。
        odom_origin_ = first_pose;
        local_odom_origin_initialized_ = false;
        resetPreintegration();   // 重新定位：重置 GTSAM 预积分

        RCLCPP_INFO(get_logger(),
            "\033[1;33m[Relocalization] Manual Initial pose set from /initialpose: (%.2f, %.2f, %.2f)\033[0m",
            first_pose(0, 3), first_pose(1, 3), first_pose(2, 3));
    }

    // ---------- 特征提取 ----------
    void extractFeatures(const pcl::PointCloud<PointType>::Ptr& input_cloud,
                         pcl::PointCloud<PointType>::Ptr& out_corner,
                         pcl::PointCloud<PointType>::Ptr& out_surf)
    {
        out_corner->clear();
        out_surf->clear();

        if (input_cloud->empty())
            return;

        down_filter_surf.setInputCloud(input_cloud);
        down_filter_surf.filter(*out_surf);

        down_filter_corner.setInputCloud(input_cloud);
        down_filter_corner.filter(*out_corner);
    }

    // ---------- 子地图更新 ----------
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

        publishCloud(pub_local_corner_cloud, local_corner_pcl_ptr, now(), mapFrame);
        publishCloud(pub_local_surf_cloud, local_surf_pcl_ptr, now(), mapFrame);

        RCLCPP_DEBUG(get_logger(),
            "[Relocalization] Submap updated at (%.1f, %.1f, %.1f) | Local Corner: %zu, Local Surf: %zu",
            center.x(), center.y(), center.z(), local_corner_pcl_ptr->size(), local_surf_pcl_ptr->size());
    }

    // ---------- IMU旋转预测 ----------
    Eigen::Matrix3d predictRotationFromImu(double from_time, double to_time)
    {
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
        if (from_time < 0 || to_time <= from_time)
            return delta_R;

        std::lock_guard<std::mutex> lock(imu_mutex);
        // imu_queue 按时间递增，窗口 [from_time, to_time] 只覆盖队列尾部极少数样本(约一个雷达周期)。
        // 先从尾部反向定位窗口起点，再正向积分，避免每帧扫描整个队列(最多 2000 条)。
        const size_t n = imu_queue.size();
        size_t start = n;
        for (size_t k = n; k-- > 0; )
        {
            if (stamp2Sec(imu_queue[k].header.stamp) < from_time)
            {
                start = k + 1;
                break;
            }
            if (k == 0)
                start = 0;
        }

        double prev_t = from_time;
        for (size_t k = start; k < n; ++k)
        {
            const auto& imu_msg = imu_queue[k];
            double t = stamp2Sec(imu_msg.header.stamp);
            if (t > to_time)
                break;
            if (t >= from_time)
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

    // ---------- 最新机体系角速度(用于 odom.twist.angular) ----------
    // extrinsicRot 为单位阵且 base 与雷达同姿态，故陀螺读数即机体系角速度。
    Eigen::Vector3d latestGyro()
    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        if (imu_queue.empty())
            return Eigen::Vector3d::Zero();
        const auto& av = imu_queue.back().angular_velocity;
        return Eigen::Vector3d(av.x, av.y, av.z);
    }

    // ---------- base<-lidar 外参解析 ----------
    // 从 TF 查询 base_link->livox_frame(robot_state_publisher 按 URDF 发布的静态变换)，
    // 得到 T_base_lidar 后求逆存为 T_lidar_base_。TF 暂不可用时沿用回退值并每帧重试。
    void resolveBaseLidarExtrinsic()
    {
        if (extrinsic_resolved_ || !tf_buffer_)
            return;
        try
        {
            // lookupTransform(target, source) 返回 T_target_source = 雷达在 base_link 下的位姿。
            // 用 3 参重载取最新静态变换；尚未收到时抛异常(下方捕获)→沿用回退值并下帧重试。
            auto tf_bl = tf_buffer_->lookupTransform(baselinkFrame, lidarFrame, tf2::TimePointZero);
            const auto& tr = tf_bl.transform.translation;
            const auto& ro = tf_bl.transform.rotation;
            Eigen::Quaternionf q(ro.w, ro.x, ro.y, ro.z);
            Eigen::Matrix4f T_base_lidar = Eigen::Matrix4f::Identity();
            T_base_lidar.block<3, 3>(0, 0) = q.toRotationMatrix();
            T_base_lidar(0, 3) = tr.x;
            T_base_lidar(1, 3) = tr.y;
            T_base_lidar(2, 3) = tr.z;
            T_lidar_base_ = T_base_lidar.inverse();
            extrinsic_resolved_ = true;
            RCLCPP_INFO(get_logger(),
                "[Reloc] base<-lidar extrinsic resolved from TF/URDF: lidar_in_base=(%.3f, %.3f, %.3f)",
                tr.x, tr.y, tr.z);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[Reloc] TF %s->%s not ready (%s); using fallback T_lidar_base translation (0.2, 0, -0.4).",
                baselinkFrame.c_str(), lidarFrame.c_str(), e.what());
        }
    }

    // ---------- 标准 LIO-SAM：GTSAM IMU 预积分初始化/重置/优化 ----------
    // 以下方法移植自建图管线 imuPreintegration.cpp 的 IMUPreintegration，噪声模型与流程保持一致，
    // 仅把“订阅 lio_sam/mapping/odometry_incremental”换成“直接用本节点 scanMatch 位姿 result_pose”。
    void initPreintegration()
    {
        // ---- 噪声/门限可调参数(对应 params.yaml relocImu*；未写则用成员默认值，与移植前硬编码一致)----
        declare_parameter("relocImuPriorPoseSigma", reloc_imu_prior_pose_sigma_);
        get_parameter("relocImuPriorPoseSigma", reloc_imu_prior_pose_sigma_);
        declare_parameter("relocImuPriorVelSigma", reloc_imu_prior_vel_sigma_);
        get_parameter("relocImuPriorVelSigma", reloc_imu_prior_vel_sigma_);
        declare_parameter("relocImuPriorBiasSigma", reloc_imu_prior_bias_sigma_);
        get_parameter("relocImuPriorBiasSigma", reloc_imu_prior_bias_sigma_);
        declare_parameter("relocImuCorrectRotSigma", reloc_imu_correct_rot_sigma_);
        get_parameter("relocImuCorrectRotSigma", reloc_imu_correct_rot_sigma_);
        declare_parameter("relocImuCorrectTransSigma", reloc_imu_correct_trans_sigma_);
        get_parameter("relocImuCorrectTransSigma", reloc_imu_correct_trans_sigma_);
        declare_parameter("relocImuIntegrationSigma", reloc_imu_integration_sigma_);
        get_parameter("relocImuIntegrationSigma", reloc_imu_integration_sigma_);
        declare_parameter("relocImuMaxVelocity", reloc_imu_max_velocity_);
        get_parameter("relocImuMaxVelocity", reloc_imu_max_velocity_);
        declare_parameter("relocImuMaxBias", reloc_imu_max_bias_);
        get_parameter("relocImuMaxBias", reloc_imu_max_bias_);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2);   // acc white noise(params.yaml imuAccNoise)
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);       // gyro white noise(params.yaml imuGyrNoise)
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(reloc_imu_integration_sigma_, 2);   // integration error
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());  // zero initial bias

        priorPoseNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << reloc_imu_prior_pose_sigma_, reloc_imu_prior_pose_sigma_, reloc_imu_prior_pose_sigma_,
                                 reloc_imu_prior_pose_sigma_, reloc_imu_prior_pose_sigma_, reloc_imu_prior_pose_sigma_).finished());  // rad,rad,rad,m,m,m
        priorVelNoise_ = gtsam::noiseModel::Isotropic::Sigma(3, reloc_imu_prior_vel_sigma_);    // m/s
        priorBiasNoise_ = gtsam::noiseModel::Isotropic::Sigma(6, reloc_imu_prior_bias_sigma_);  // bias prior
        correctionNoise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << reloc_imu_correct_rot_sigma_, reloc_imu_correct_rot_sigma_, reloc_imu_correct_rot_sigma_,
                                 reloc_imu_correct_trans_sigma_, reloc_imu_correct_trans_sigma_, reloc_imu_correct_trans_sigma_).finished());  // scanMatch pose 观测噪声
        noiseModelBetweenBias_ =
            (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);   // 高频预测用
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);   // 优化用

        // IMU<-lidar 外参(与 imuPreintegration.cpp 一致，取自 params.yaml extrinsicTrans = lidar2Imu)
        imu2Lidar_ = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
        lidar2Imu_ = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

        resetOptimization();
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer_ = gtsam::ISAM2(optParameters);
        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors_ = newGraphFactors;
        gtsam::Values NewGraphValues;
        graphValues_ = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu_ = -1;
        lastImuT_opt_ = -1;
        doneFirstOpt_ = false;
        systemInitialized_ = false;
    }

    // 重新定位(收到新的 initial_odom / /initialpose)时重置预积分，等待用新的首帧 scanMatch 位姿重新初始化
    void resetPreintegration()
    {
        std::lock_guard<std::mutex> lock(preint_mutex_);
        resetParams();
        resetOptimization();
        imuQueOpt_.clear();
        imuQueImu_.clear();
        prevVel_ = gtsam::Vector3(0, 0, 0);
        prevBias_ = gtsam::imuBias::ConstantBias();
        prevBiasOdom_ = gtsam::imuBias::ConstantBias();
        prevStateOdom_ = gtsam::NavState();
        key_ = 1;
        gtsam::imuBias::ConstantBias zero_bias;
        if (imuIntegratorImu_) imuIntegratorImu_->resetIntegrationAndSetBias(zero_bias);
        if (imuIntegratorOpt_) imuIntegratorOpt_->resetIntegrationAndSetBias(zero_bias);
        preint_odom_origin_valid_ = false;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > reloc_imu_max_velocity_)
        {
            RCLCPP_WARN(get_logger(), "[Reloc] Large velocity, reset IMU-preintegration!");
            return true;
        }
        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > reloc_imu_max_bias_ || bg.norm() > reloc_imu_max_bias_)
        {
            RCLCPP_WARN(get_logger(), "[Reloc] Large bias, reset IMU-preintegration!");
            return true;
        }
        return false;
    }

    // 标准 LIO-SAM odometryHandler 的移植：以 scanMatch 位姿 T_map_lidar 为观测做一次 ISAM2 优化，
    // 得到 map 系速度 prevVel_ 与零偏 prevBias_/prevBiasOdom_，并用新零偏重新传播 imuIntegratorImu_。
    // 须在 run() 内、每帧 scanMatch 之后调用。内部对 preint_mutex_ 加锁。
    void preintOptimize(const Eigen::Matrix4f& T_map_lidar, double currentCorrectionTime)
    {
        std::lock_guard<std::mutex> lock(preint_mutex_);

        if (imuQueOpt_.empty())
            return;

        gtsam::Pose3 lidarPose(
            gtsam::Rot3(T_map_lidar.block<3, 3>(0, 0).cast<double>()),
            gtsam::Point3(T_map_lidar(0, 3), T_map_lidar(1, 3), T_map_lidar(2, 3)));

        // 0. 初始化系统(首帧 scanMatch)
        if (!systemInitialized_)
        {
            resetOptimization();
            while (!imuQueOpt_.empty())
            {
                if (stamp2Sec(imuQueOpt_.front().header.stamp) < currentCorrectionTime - delta_t_)
                {
                    lastImuT_opt_ = stamp2Sec(imuQueOpt_.front().header.stamp);
                    imuQueOpt_.pop_front();
                }
                else
                    break;
            }
            prevPose_ = lidarPose.compose(lidar2Imu_);
            graphFactors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prevPose_, priorPoseNoise_));
            prevVel_ = gtsam::Vector3(0, 0, 0);
            graphFactors_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), prevVel_, priorVelNoise_));
            prevBias_ = gtsam::imuBias::ConstantBias();
            graphFactors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prevBias_, priorBiasNoise_));
            graphValues_.insert(X(0), prevPose_);
            graphValues_.insert(V(0), prevVel_);
            graphValues_.insert(B(0), prevBias_);
            optimizer_.update(graphFactors_, graphValues_);
            graphFactors_.resize(0);
            graphValues_.clear();
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            key_ = 1;
            systemInitialized_ = true;
            return;
        }

        // 控制图规模(与 LIO-SAM 一致，每 100 帧重置一次，用边际协方差作为新先验)
        if (key_ == 100)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise =
                gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(key_ - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise =
                gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(key_ - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise =
                gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(key_ - 1)));
            resetOptimization();
            graphFactors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prevPose_, updatedPoseNoise));
            graphFactors_.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), prevVel_, updatedVelNoise));
            graphFactors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prevBias_, updatedBiasNoise));
            graphValues_.insert(X(0), prevPose_);
            graphValues_.insert(V(0), prevVel_);
            graphValues_.insert(B(0), prevBias_);
            optimizer_.update(graphFactors_, graphValues_);
            graphFactors_.resize(0);
            graphValues_.clear();
            key_ = 1;
        }

        // 1. 积分两次优化之间的 IMU，加入 IMU 因子/零偏因子/位姿观测因子并优化
        while (!imuQueOpt_.empty())
        {
            sensor_msgs::msg::Imu* thisImu = &imuQueOpt_.front();
            double imuTime = stamp2Sec(thisImu->header.stamp);
            if (imuTime < currentCorrectionTime - delta_t_)
            {
                double dt = (lastImuT_opt_ < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt_);
                imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z),
                    dt);
                lastImuT_opt_ = imuTime;
                imuQueOpt_.pop_front();
            }
            else
                break;
        }
        const gtsam::PreintegratedImuMeasurements& preint_imu =
            dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preint_imu);
        graphFactors_.add(imu_factor);
        graphFactors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
            B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias_)));
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu_);
        graphFactors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_), curPose, correctionNoise_));
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues_.insert(X(key_), propState_.pose());
        graphValues_.insert(V(key_), propState_.v());
        graphValues_.insert(B(key_), prevBias_);
        optimizer_.update(graphFactors_, graphValues_);
        optimizer_.update();
        graphFactors_.resize(0);
        graphValues_.clear();

        gtsam::Values result = optimizer_.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key_));
        prevVel_ = result.at<gtsam::Vector3>(V(key_));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }

        // 2. 优化后用新零偏重新传播 IMU 预积分(供 hrOdomThread 高频预测)
        prevStateOdom_ = prevState_;
        prevBiasOdom_ = prevBias_;
        double lastImuQT = -1;
        while (!imuQueImu_.empty() && stamp2Sec(imuQueImu_.front().header.stamp) < currentCorrectionTime - delta_t_)
        {
            lastImuQT = stamp2Sec(imuQueImu_.front().header.stamp);
            imuQueImu_.pop_front();
        }
        if (!imuQueImu_.empty())
        {
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom_);
            for (int i = 0; i < (int)imuQueImu_.size(); ++i)
            {
                sensor_msgs::msg::Imu* thisImu = &imuQueImu_[i];
                double imuTime = stamp2Sec(thisImu->header.stamp);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
                imuIntegratorImu_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z),
                    dt);
                lastImuQT = imuTime;
            }
        }

        ++key_;
        doneFirstOpt_ = true;
    }

    // ---------- 主处理循环 ----------
    void run()
    {
        std::lock_guard<std::mutex> state_lock(state_mutex);

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

        // 特征提取
        extractFeatures(cur_cloud, cur_corner_pcl_ptr, cur_surf_pcl_ptr);

        publishCloud(pub_corner_cloud, cur_corner_pcl_ptr, cur_header.stamp, lidarFrame);
        publishCloud(pub_surf_cloud, cur_surf_pcl_ptr, cur_header.stamp, lidarFrame);

        // 子地图更新：距上次裁剪中心移动超过阈值就重裁（与下方基于边界邻近的
        // submap_update_thresh 触发是两套独立机制）
        constexpr float submap_recrop_dist_thresh = 10.0f;  // meters travelled since last submap center
        float dist_to_submap = (last_pose.block<3, 1>(0, 3) - last_submap_center).norm();
        if (!submap_initialized || dist_to_submap > submap_recrop_dist_thresh)
        {
            updateSubmap(last_pose.block<3, 1>(0, 3));
            last_submap_center = last_pose.block<3, 1>(0, 3);
        }

        // 预测位姿
        Eigen::Matrix4f predict_pose = last_pose;

        if (last_cloud_time > 0)
        {
            Eigen::Matrix3d delta_R = predictRotationFromImu(last_cloud_time, cur_time);
            Eigen::Matrix3d last_R = last_pose.block<3, 3>(0, 0).cast<double>();
            predict_pose.block<3, 3>(0, 0) = (last_R * delta_R).cast<float>();
        }
        last_cloud_time = cur_time;

        Eigen::Vector3f step_trans = last_step_trans;
        if (step_trans.norm() > 0.8f)
        {
            step_trans = Eigen::Vector3f::Zero();
        }
        predict_pose.block<3, 1>(0, 3) = last_pose.block<3, 1>(0, 3) + step_trans;

        // 扫描匹配
        result_pose = scanMatch(
            cur_corner_pcl_ptr,
            cur_surf_pcl_ptr,
            local_corner_pcl_ptr,
            local_surf_pcl_ptr,
            predict_pose);

        // Reject an obviously catastrophic translation jump. This is only a
        // safety gate; normal scan matching remains unchanged.
        // 门限 max_translation_jump_ 可在 params.yaml 的 relocMaxTranslationJump 调整。
        const float translation_jump =
            (result_pose.block<3, 1>(0, 3) -
             predict_pose.block<3, 1>(0, 3)).norm();

        if (translation_jump > max_translation_jump_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "[Reloc] Rejecting scan-match jump: %.3f m",
                translation_jump);
            result_pose = predict_pose;
        }

        last_step_trans = result_pose.block<3, 1>(0, 3) - last_pose.block<3, 1>(0, 3);
        last_pose = result_pose;

        // result_pose 是雷达在 map 的位姿 (T_map_lidar)。对外发布的里程计/TF/path 是 base_link，
        // 需右乘 T_lidar_base 换算；scanMatch 内部状态(last_pose/predict_pose)与 cloud_registered
        // 仍用 result_pose(雷达系)保持不变。
        resolveBaseLidarExtrinsic();
        const Eigen::Matrix4f T_map_base = result_pose * T_lidar_base_;

        // ---------- 计算局部里程计相对位姿 ----------
        //
        // 关键逻辑：
        // initial_odom / /initialpose 给的是“地图坐标系中的定位起点”，
        // 它不一定等于第一帧 scanMatch 的最终结果。
        //
        // 因此不能在收到 initial pose 时就永久把它作为 local odom 原点。
        // 必须把“第一次实际 scanMatch 后的 result_pose”锁定为局部里程计原点：
        //
        //     T_odom_base(t0) = T_map_base(t0)^(-1) * T_map_base(t0) = I
        //
        // 从而第一帧局部位置和姿态严格为：
        //     position = (0, 0, 0)
        //     orientation = (0, 0, 0, 1)
        //
        // 后续帧：
        //     T_odom_base(t) = T_map_odom^(-1) * T_map_base(t)
        //                    = odom_origin_.inverse() * result_pose
        //
        bool local_odom_origin_just_initialized = false;

        if (!local_odom_origin_initialized_)
        {
            // The first accepted scan-matching pose defines the local odometry origin.
            // Therefore:
            //     T_odom_base(t0) = I
            odom_origin_ = T_map_base;
            local_odom_origin_initialized_ = true;
            local_odom_origin_just_initialized = true;

            const double global_start_yaw_deg =
                rad2deg(getYawFromRotation(odom_origin_.block<3, 3>(0, 0)));

            RCLCPP_INFO(
                get_logger(),
                "[Reloc] Local odom origin locked: "
                "Global(map) XYZ=(%.3f, %.3f, %.3f), Yaw=%.3f deg | "
                "Local(odom) XYZ=(0.000, 0.000, 0.000), Yaw=0.000 deg",
                odom_origin_(0, 3),
                odom_origin_(1, 3),
                odom_origin_(2, 3),
                global_start_yaw_deg);
        }

        // First frame is explicitly Identity. For all later frames, compute the
        // rigid-body relative transform in SE(3).
        Eigen::Matrix4f relative_pose;
        if (local_odom_origin_just_initialized)
        {
            relative_pose = Eigen::Matrix4f::Identity();
        }
        else
        {
            relative_pose = odom_origin_.inverse() * T_map_base;
        }

        // 1. 标准 LIO-SAM 方法：把 scanMatch 位姿 result_pose(T_map_lidar)作为位姿观测喂给 GTSAM
        //    IMU 预积分因子图，联合优化出 map 系线速度 prevVel_ 与陀螺零偏 prevBiasOdom_。
        //    local_lidar_odom 的高频位姿/速度在 hrOdomThread 用 imuIntegratorImu_->predict() 发布。
        preintOptimize(result_pose, cur_time);

        Eigen::Quaternionf q_local(relative_pose.block<3, 3>(0, 0));   // 供下方 10Hz TF(odom->base) 用
        Eigen::Vector3d vel_map = Eigen::Vector3d::Zero();    // GTSAM NavState 线速度(map 系)
        Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();  // 估计的陀螺零偏
        bool preint_ok = false;
        {
            std::lock_guard<std::mutex> lock(preint_mutex_);
            preint_odom_origin_ = odom_origin_;                  // map->odom 原点快照(供 hrOdomThread)
            preint_odom_origin_valid_ = local_odom_origin_initialized_;
            preint_T_lidar_base_ = T_lidar_base_;                // 外参快照(供 hrOdomThread 跨线程安全读取)
            vel_map = prevVel_;
            gyro_bias = prevBiasOdom_.gyroscope();
            preint_ok = doneFirstOpt_;
        }
        // 机体系线速度(REP-103: twist 在 child_frame=base_link) = R_map_base^T · v_map
        Eigen::Vector3d vel_body = Eigen::Vector3d::Zero();
        if (preint_ok)
            vel_body = T_map_base.block<3, 3>(0, 0).cast<double>().transpose() * vel_map;
        // 机体系角速度 = 去零偏后的陀螺(GTSAM 约定：真实角速度 = 测量 - 零偏)
        const Eigen::Vector3d gyro_body = latestGyro() - gyro_bias;

        // 2. 发布全局里程计 (frame_id = "map")，含 twist(base_link 系)
        nav_msgs::msg::Odometry odom_global_msg;
        odom_global_msg.header.frame_id = mapFrame;
        odom_global_msg.header.stamp = cur_header.stamp;
        odom_global_msg.child_frame_id = baselinkFrame;
        odom_global_msg.pose.pose.position.x = T_map_base(0, 3);
        odom_global_msg.pose.pose.position.y = T_map_base(1, 3);
        odom_global_msg.pose.pose.position.z = T_map_base(2, 3);
        Eigen::Quaternionf q_global(T_map_base.block<3, 3>(0, 0));
        odom_global_msg.pose.pose.orientation.w = q_global.w();
        odom_global_msg.pose.pose.orientation.x = q_global.x();
        odom_global_msg.pose.pose.orientation.y = q_global.y();
        odom_global_msg.pose.pose.orientation.z = q_global.z();
        odom_global_msg.twist.twist.linear.x = vel_body.x();
        odom_global_msg.twist.twist.linear.y = vel_body.y();
        odom_global_msg.twist.twist.linear.z = vel_body.z();
        odom_global_msg.twist.twist.angular.x = gyro_body.x();
        odom_global_msg.twist.twist.angular.y = gyro_body.y();
        odom_global_msg.twist.twist.angular.z = gyro_body.z();
        pub_lidar_odom->publish(odom_global_msg);

        // 3. 发布TF：map -> odom（固定） 和 odom -> base_link（局部里程计）
        geometry_msgs::msg::TransformStamped tf_map_odom, tf_odom_base;

        // map -> odom
        tf_map_odom.header.stamp = cur_header.stamp;
        tf_map_odom.header.frame_id = mapFrame;
        tf_map_odom.child_frame_id = "odom";
        tf_map_odom.transform.translation.x = odom_origin_(0, 3);
        tf_map_odom.transform.translation.y = odom_origin_(1, 3);
        tf_map_odom.transform.translation.z = odom_origin_(2, 3);
        Eigen::Quaternionf q_origin(odom_origin_.block<3, 3>(0, 0));
        tf_map_odom.transform.rotation.w = q_origin.w();
        tf_map_odom.transform.rotation.x = q_origin.x();
        tf_map_odom.transform.rotation.y = q_origin.y();
        tf_map_odom.transform.rotation.z = q_origin.z();

        // odom -> base_link
        tf_odom_base.header.stamp = cur_header.stamp;
        tf_odom_base.header.frame_id = "odom";
        tf_odom_base.child_frame_id = baselinkFrame;
        tf_odom_base.transform.translation.x = relative_pose(0, 3);
        tf_odom_base.transform.translation.y = relative_pose(1, 3);
        tf_odom_base.transform.translation.z = relative_pose(2, 3);
        tf_odom_base.transform.rotation.w = q_local.w();
        tf_odom_base.transform.rotation.x = q_local.x();
        tf_odom_base.transform.rotation.y = q_local.y();
        tf_odom_base.transform.rotation.z = q_local.z();

        tf_broadcaster->sendTransform({tf_map_odom, tf_odom_base});

        // 4. 发布全局路径 (frame_id = "map")
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = cur_header.stamp;
        pose_stamped.header.frame_id = mapFrame;
        pose_stamped.pose.position.x = T_map_base(0, 3);
        pose_stamped.pose.position.y = T_map_base(1, 3);
        pose_stamped.pose.position.z = T_map_base(2, 3);
        pose_stamped.pose.orientation = odom_global_msg.pose.pose.orientation;

        global_path.header.frame_id = mapFrame;
        global_path.header.stamp = cur_header.stamp;
        global_path.poses.push_back(pose_stamped);
        if (global_path.poses.size() > 2000)
            global_path.poses.erase(global_path.poses.begin());
        pub_path->publish(global_path);

        // 5. 发布注册点云（在map坐标系）
        if (!cur_cloud->empty())
        {
            pcl::PointCloud<PointType>::Ptr reg_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cur_cloud, *reg_cloud, result_pose);
            publishCloud(pub_cloud_registered, reg_cloud, cur_header.stamp, mapFrame);
        }

        // ---------- 输出全局/局部位姿 ----------
        // 全局 yaw：result_pose 在 map 坐标系中的航向角
        // 局部 yaw：relative_pose 在 odom 坐标系中的航向角
        //
        // 由于 odom_origin_ 是第一次实际 scanMatch 的 result_pose，
        // 第一帧 local_yaw 理论上严格为 0 deg。
        const double global_yaw_rad =
            getYawFromRotation(T_map_base.block<3, 3>(0, 0));
        const double local_yaw_rad =
            getYawFromRotation(relative_pose.block<3, 3>(0, 0));

        const double global_yaw_deg = rad2deg(global_yaw_rad);
        const double local_yaw_deg = rad2deg(local_yaw_rad);

        // static int print_cnt = 0;
        // if (++print_cnt % 10 == 0)
        // {
            RCLCPP_INFO(
                get_logger(),
                "[Reloc] "
                "Global(map): XYZ=(%.3f, %.3f, %.3f), Yaw=%.3f deg | "
                "Local(odom): XYZ=(%.3f, %.3f, %.3f), Yaw=%.3f deg",
                T_map_base(0, 3),
                T_map_base(1, 3),
                T_map_base(2, 3),
                global_yaw_deg,
                relative_pose(0, 3),
                relative_pose(1, 3),
                relative_pose(2, 3),
                local_yaw_deg);
        // }

        // 6. 子地图边界检查（基于全局位姿）
        float cur_x = result_pose(0, 3);
        float cur_y = result_pose(1, 3);
        float cur_z = result_pose(2, 3);
        if (std::abs(cur_x - edge_left[0]) < submap_update_thresh || std::abs(cur_x - edge_right[0]) < submap_update_thresh ||
            std::abs(cur_y - edge_left[1]) < submap_update_thresh || std::abs(cur_y - edge_right[1]) < submap_update_thresh ||
            std::abs(cur_z - edge_left[2]) < submap_update_thresh || std::abs(cur_z - edge_right[2]) < submap_update_thresh)
        {
            updateSubmap(Eigen::Vector3f(cur_x, cur_y, cur_z));
            last_submap_center = Eigen::Vector3f(cur_x, cur_y, cur_z);
        }
    }

    // ---------- Yaw角计算 ----------
    // 从旋转矩阵提取 ROS/map 坐标系下的 Z 轴航向角。
    // 返回范围：[-pi, pi]。
    static double getYawFromRotation(const Eigen::Matrix3f& rotation)
    {
        return std::atan2(
            static_cast<double>(rotation(1, 0)),
            static_cast<double>(rotation(0, 0)));
    }

    // 将弧度转换为角度。
    static double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }

    // ---------- 扫描匹配（Ceres优化） ----------
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

            // 边缘约束（并行）
            std::vector<EdgeResidualData> valid_edges;
            int laser_corner_num = static_cast<int>(corner_ptr->size());
            if (!local_corner_ptr->empty() && laser_corner_num > 0)
            {
                // 均匀跨步封顶：限制每帧处理的角点数(max_corner_pts_)，弱设备上大幅降低
                // NN 搜索 / 特征值分解 / Ceres 残差数量的开销。
                const int corner_step = std::max(1, laser_corner_num / max_corner_pts_);
                valid_edges.reserve(static_cast<size_t>(laser_corner_num / corner_step) + 1);
                #pragma omp parallel
                {
                    std::vector<EdgeResidualData> local_edges;
                    std::vector<int> point_search_ind;
                    std::vector<float> point_search_dist;

                    #pragma omp for nowait
                    for (int i = 0; i < laser_corner_num; i += corner_step)
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

            // 平面约束（并行）
            std::vector<PlaneResidualData> valid_planes;
            int laser_surf_num = static_cast<int>(surf_ptr->size());
            if (!local_surf_ptr->empty() && laser_surf_num > 0)
            {
                // 均匀跨步封顶：限制每帧处理的平面点数(max_surf_pts_)。
                const int surf_step = std::max(1, laser_surf_num / max_surf_pts_);
                valid_planes.reserve(static_cast<size_t>(laser_surf_num / surf_step) + 1);
                #pragma omp parallel
                {
                    std::vector<PlaneResidualData> local_planes;
                    std::vector<int> point_search_ind;
                    std::vector<float> point_search_dist;

                    #pragma omp for nowait
                    for (int i = 0; i < laser_surf_num; i += surf_step)
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
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "[Reloc] Scan matching under-constrained: %zu edge + %zu plane features",
                    valid_edges.size(),
                    valid_planes.size());
                return predict_pose;
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.num_threads = ceres_num_threads_;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            if (!summary.IsSolutionUsable())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "[Reloc] Ceres solution is not usable; keeping predicted pose");
                return predict_pose;
            }
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

    // ---------- 线程函数 ----------
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
        rclcpp::Rate rate(1.0);
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

    // ---------- 高频局部里程计线程(标准 LIO-SAM：GTSAM IMU 预积分预测) ----------
    // 每帧 scanMatch 经 preintOptimize() 优化并重置传播锚点(prevStateOdom_/prevBiasOdom_)；本线程按
    // hr_rate_ 用 imuIntegratorImu_->predict() 外推到最新 IMU 时刻，发布 local_lidar_odom：
    // 位姿由预测的 T_map_base 转到 odom 系，线速度取 NavState.velocity()，角速度取去零偏后的陀螺。
    // 预测锚点每帧被 scanMatch 重置，故误差不累积(与 imuPreintegration.cpp 的 imuHandler 同理)。
    void hrOdomThread()
    {
        rclcpp::Rate rate(hr_rate_);
        while (rclcpp::ok())
        {
            try { rate.sleep(); }
            catch (const std::exception&) { break; }
            if (!rclcpp::ok()) break;

            // 1) 最新 IMU 陀螺与时间戳(用于 twist.angular 与消息 stamp)
            std_msgs::msg::Header imu_header;
            Eigen::Vector3d gyro_meas = Eigen::Vector3d::Zero();
            {
                std::lock_guard<std::mutex> lock(imu_mutex);
                if (imu_queue.empty()) continue;
                imu_header = imu_queue.back().header;
                const auto& av = imu_queue.back().angular_velocity;
                gyro_meas = Eigen::Vector3d(av.x, av.y, av.z);
            }

            // 2) GTSAM 预测当前 NavState，组装 local_lidar_odom(odom 系)
            nav_msgs::msg::Odometry msg;
            bool do_publish = false;
            {
                std::lock_guard<std::mutex> lock(preint_mutex_);
                if (!doneFirstOpt_ || !preint_odom_origin_valid_) continue;

                gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom_, prevBiasOdom_);
                gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
                gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar_);                       // T_map_lidar
                Eigen::Matrix4f T_map_lidar = lidarPose.matrix().cast<float>();
                Eigen::Matrix4f T_map_base = T_map_lidar * preint_T_lidar_base_;            // T_map_base
                Eigen::Matrix4f T_odom_base = preint_odom_origin_.inverse() * T_map_base;    // T_odom_base

                // 线速度：NavState.velocity() 在 map 系 → 机体系(REP-103) = R_map_base^T · v_map
                Eigen::Vector3d v_map(currentState.velocity().x(), currentState.velocity().y(), currentState.velocity().z());
                Eigen::Vector3d vel_body = T_map_base.block<3, 3>(0, 0).cast<double>().transpose() * v_map;
                // 角速度：去零偏后的陀螺(GTSAM 约定：真实 = 测量 - 零偏)
                Eigen::Vector3d gyro_body = gyro_meas - prevBiasOdom_.gyroscope();

                msg.header.frame_id = "odom";
                msg.header.stamp = imu_header.stamp;
                msg.child_frame_id = baselinkFrame;
                msg.pose.pose.position.x = T_odom_base(0, 3);
                msg.pose.pose.position.y = T_odom_base(1, 3);
                msg.pose.pose.position.z = T_odom_base(2, 3);
                Eigen::Quaternionf q(T_odom_base.block<3, 3>(0, 0));
                msg.pose.pose.orientation.w = q.w();
                msg.pose.pose.orientation.x = q.x();
                msg.pose.pose.orientation.y = q.y();
                msg.pose.pose.orientation.z = q.z();
                // [安全防护] 物理速度合理性硬限制，防 IMU 预积分重置/跳变时输出剧烈异常速度
                double max_safe_vel = 1.5;  // m/s
                double max_safe_wz  = 2.5;  // rad/s
                double vx_safe = std::max(-max_safe_vel, std::min(max_safe_vel, vel_body.x()));
                double wz_safe = std::max(-max_safe_wz,  std::min(max_safe_wz,  gyro_body.z()));

                msg.twist.twist.linear.x = vx_safe;
                msg.twist.twist.linear.y = vel_body.y();
                msg.twist.twist.linear.z = vel_body.z();
                msg.twist.twist.angular.x = gyro_body.x();
                msg.twist.twist.angular.y = gyro_body.y();
                msg.twist.twist.angular.z = wz_safe;
                do_publish = true;
            }
            if (do_publish) pub_local_odom->publish(msg);
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
    std::thread hr_odom_thread(&Relocalization::hrOdomThread, node);

    rclcpp::spin(node);

    rclcpp::shutdown();

    if (run_thread.joinable())
        run_thread.join();
    if (map_vis_thread.joinable())
        map_vis_thread.join();
    if (hr_odom_thread.joinable())
        hr_odom_thread.join();

    return 0;
}