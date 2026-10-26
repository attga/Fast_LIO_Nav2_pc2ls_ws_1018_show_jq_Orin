#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <eigen3/Eigen/Dense>


#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "sc_pgo/common.h"
#include "sc_pgo/tic_toc.h"
#include "scancontext/Scancontext.h"


using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;

SCManager scManager; // 2026-09-10 恢复:SC 描述子库(重定位用,检测仍走距离方式)
double scDistThres, scMaximumRadius;
pcl::VoxelGrid<PointType> downSizeFilterScancontext; // 关键帧降采样(0.2m 有损存储)

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

// 地面 z 约束(2026-09-07 新增,仿 addGPSFactor 结构,不加 GPS):
// 首个关键帧位姿 z 作为参考地面高度,后续关键帧用 GPSFactor(z-only:
// xy 噪声 1e9 不约束、z 拉回参考)锚定 z;|z-z_ref|/kappa 动态放大噪声,
// 坡道/起伏自动失活。参数 ground_z_en / ground_z_sigma / ground_z_kappa
bool groundZEn = false;
bool groundZRefInitialized = false;
double groundZRef = 0.0;
double groundZSigma = 0.3;
double groundZKappa = 1.0;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCurAftPGO; // 2026-09-07 新增:当前帧云×优化位姿
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal; // 2026-09-08 恢复:GICP source 发布
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;

rclcpp::Node::SharedPtr g_pgo_node;

// Precision-preserving seconds -> rclcpp::Time
inline rclcpp::Time sec_to_time(double seconds)
{
    uint32_t sec = static_cast<uint32_t>(seconds);
    uint32_t nanosec = static_cast<uint32_t>((seconds - sec) * 1e9 + 0.5);
    if (nanosec >= 1000000000u) { nanosec -= 1000000000u; sec += 1; }
    return rclcpp::Time(sec, nanosec);
}

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
std::fstream pgTimeSaveStream;
double g_loop_fitness_score_thres = 0.3;
double g_loop_icp_t_max = 5.0;
double g_loop_noise_score = 0.5;
double odomTransNoise = 1e-6; // odom 因子平移方差(默认 1e-6,绕楼短程回环褶皱时调 1e-4)
std::pair<int, int> g_lastLoopPair = {-1, -1};
double g_lastNewKeyframeWallTime = 0.0; // 最近一次关键帧插入的墙钟时间(process_pg 写,process_lcd 读)

// 距离回环检测:对优化后关键帧位姿做半径搜索,时间差过滤,取最近历史帧。
// 距离回环检测参数:loop_distance_radius / loop_distance_time_diff
double loopDistanceRadius = 10.0;
double loopDistanceTimeDiff = 30.0;

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr _laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr _laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // 平移方差 odom_trans_noise 参数化(默认 1e-6 极硬,σ=1mm):短程回环(绕楼一圈
    // 修正 2m 级)与硬链张力导致局部褶皱时,调软到 1e-4(σ=1cm)让链条吸收修正
    odomNoiseVector6 << odomTransNoise, odomTransNoise, odomTransNoise, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = g_loop_noise_score; // 方差：0.5(上游) 局部拽动强、有折痕；4.0 回环变软、只做全局温和修正（适合 FAST-LIO 这种漂移小的前端）
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

} // initNoises

Pose6D getOdom(nav_msgs::msg::Odometry::ConstSharedPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = "odom";
    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
        // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "odom";
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = sec_to_time(keyframeTimes.at(node_idx));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        tf2::Quaternion q_tf2;
        q_tf2.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation.x = q_tf2.x();
        odomAftPGOthis.pose.pose.orientation.y = q_tf2.y();
        odomAftPGOthis.pose.pose.orientation.z = q_tf2.z();
        odomAftPGOthis.pose.pose.orientation.w = q_tf2.w();
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "odom";
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock(); 
    pubOdomAftPGO->publish(odomAftPGO); // last pose 
    pubPathAftPGO->publish(pathAftPGO); // poses 

    static auto br = std::make_shared<tf2_ros::TransformBroadcaster>(*g_pgo_node);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = odomAftPGO.header.stamp;
    transformStamped.header.frame_id = "odom"; // 2026-09-11:FAST-LIO2 TF 改名后,camera_init 无人发布,改用 odom(同系)
    transformStamped.child_frame_id = "aft_pgo";
    transformStamped.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftPGO.pose.pose.position.z;
    transformStamped.transform.rotation.w = odomAftPGO.pose.pose.orientation.w;
    transformStamped.transform.rotation.x = odomAftPGO.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odomAftPGO.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odomAftPGO.pose.pose.orientation.z;
    br->sendTransform(transformStamped);
} // pubPath

void updatePoses(void)
{
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // 2026-09-07 改:原版所有关键帧都用 root 位姿变换(堆叠在 root 处,虚拟坐标系),
    // 该方案下 ICP 必须从单位阵出发且对反向回环不收敛。改为各自关键帧的优化位姿
    // 变换到全局系(LIO-SAM 式):两条腿的点云天然预对齐,ICP 只需找小的漂移残差。
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock();
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud

std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    int historyKeyframeSearchNum = 25; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 5, _loop_kf_idx); // 2026-09-08 单帧→±2→±5 帧:
    // Mid-360 单帧稀疏且非重复扫描 Pattern 不一致,±5 帧(约 5m 窗口)多帧叠加致密化,
    // 提重叠率与 Pattern 互补;仍远小于 10m corr 闸门
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx);

    // loop verification
    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "odom";
    pubLoopScanLocal->publish(cureKeyframeCloudMsg); // 2026-09-08 恢复:GICP source(rviz 查对齐)

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "odom";
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    // GICP(2026-09-08 换自点对点:往返段朝向相反/低重叠/平面结构下更稳)
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(30); // 全局系预对齐后残差仅 1~2m;
    // 150m 会把往返两条腿对方路径上的点拉进来当错误对应
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setUseReciprocalCorrespondences(true); // fitness 只统计双向对应点,
    // 否则 source(±5 帧)与 target(±25 帧走廊)的非重叠区会稀释 fit
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    std::cout << "[GICP] source " << cureKeyframeCloud->size()
              << " pts, target " << targetKeyframeCloud->size() << " pts" << std::endl;
    icp.align(*unused_result); // 单位阵初始化(全局系预对齐)
    Eigen::Matrix4f finalTf = icp.getFinalTransformation();
    bool converged = icp.hasConverged();
    float fitnessScore = icp.getFitnessScore();

    // Get pose transformation(每个候选都打印修正量,用于判断
    // "中等 fitness 的候选是周期性结构错配(小修正)还是真·假回环(大修正)")
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = finalTf;
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    float icpTnorm = sqrt(x*x + y*y + z*z);
    std::cout << "[Loop] ICP result: fitness=" << fitnessScore
              << " correction=" << icpTnorm << " m" << std::endl;

    float loopFitnessScoreThreshold = g_loop_fitness_score_thres;
    if (converged == false || fitnessScore > loopFitnessScoreThreshold) {
        std::cout << "[Loop] ICP fitness test failed (" << fitnessScore << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        std::cout << "[Loop] ICP fitness test passed (" << fitnessScore << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // 几何一致性闸门：ICP 残差 = 对齐修正量。
    // 真实回环：残差 ≈ 前端漂移（米级以内）；
    // 假回环：两个不同场景被强行对齐，残差动辄十几到上百米。
    if (icpTnorm > g_loop_icp_t_max) {
        std::cout << "[Loop] ICP implies " << icpTnorm << " m correction (>" << g_loop_icp_t_max
                  << " m). Reject as false loop." << std::endl;
        return std::nullopt;
    }

    // 2026-09-07 改:回环因子 = 修正后的当前位姿 与 回环帧位姿 的相对变换
    // (LIO-SAM 式;原版把 ICP 残差直接当相对位姿,只在虚拟坐标系方案下成立)
    Pose6D pose_loop_6d, pose_curr_6d;
    mKF.lock();
    pose_loop_6d = keyframePosesUpdated.at(_loop_kf_idx);
    pose_curr_6d = keyframePosesUpdated.at(_curr_kf_idx);
    mKF.unlock();
    Eigen::Affine3f tWrong = pcl::getTransformation(pose_curr_6d.x, pose_curr_6d.y, pose_curr_6d.z,
                                                    pose_curr_6d.roll, pose_curr_6d.pitch, pose_curr_6d.yaw);
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    float cx, cy, cz, cr, cp, cyw;
    pcl::getTranslationAndEulerAngles(tCorrect, cx, cy, cz, cr, cp, cyw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(cr, cp, cyw), Point3(cx, cy, cz));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(pose_loop_6d.roll, pose_loop_6d.pitch, pose_loop_6d.yaw),
                                Point3(pose_loop_6d.x, pose_loop_6d.y, pose_loop_6d.z));

    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{
    while(rclcpp::ok())
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && rclcpp::Time(odometryBuf.front()->header.stamp).seconds() < rclcpp::Time(fullResBuf.front()->header.stamp).seconds())
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = rclcpp::Time(odometryBuf.front()->header.stamp).seconds();
            timeLaser = rclcpp::Time(fullResBuf.front()->header.stamp).seconds();
            // TODO

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();

            mBuf.unlock();

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
                continue; 

            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock(); 
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);
            g_lastNewKeyframeWallTime = rclcpp::Clock().now().seconds(); // bag 结束检测判据(30s 无新关键帧停实时检测)


            laserCloudMapPGORedraw = true;
            mKF.unlock(); 

            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    // runISAM2opt();          
                }   
                mtxPosegraph.unlock();

                gtSAMgraphMade = true;

                // 地面 z 约束(2026-09-07 新增):首个关键帧位姿 z 作为参考地面高度
                if (groundZEn && !groundZRefInitialized) {
                    groundZRef = poseOrigin.z();
                    groundZRefInitialized = true;
                    cout << "Ground z reference set: " << groundZRef << endl;
                }

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                    // 地面 z 约束(2026-09-07 新增,仿 GPS 因子的 z-only 锚定,不加 GPS):
                    // 观测 = 参考地面高度;xy 噪声 1e9 自参考不约束;z 噪声动态降权
                    if (groundZEn && groundZRefInitialized) {
                        double dz = fabs(poseTo.z() - groundZRef);
                        double sigma_z = groundZSigma * (1.0 + dz / groundZKappa);
                        gtsam::Vector groundZNoise(3);
                        groundZNoise << 1e9, 1e9, sigma_z * sigma_z;
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx,
                            gtsam::Point3(poseTo.x(), poseTo.y(), groundZRef),
                            gtsam::noiseModel::Diagonal::Variances(groundZNoise)));
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);                
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

            pgTimeSaveStream << timeLaser << std::endl; // path
        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void performDistanceLoopClosure(void)
{
    double now_wall = rclcpp::Clock().now().seconds();
    if (g_lastNewKeyframeWallTime > 0.0 && now_wall - g_lastNewKeyframeWallTime > 30.0) return; // 2026-09-09:bag 结束后不再实时检测(最后一帧固定,防刷屏)
    const int curr_node_idx = int(keyframePoses.size()) - 1;
    if (curr_node_idx < 10)
        return;

    // 用优化后的位姿(被地面约束压平)做候选搜索,z 漂移不污染判据
    Pose6D pose_curr;
    mKF.lock();
    if (int(keyframePosesUpdated.size()) <= curr_node_idx) { mKF.unlock(); return; }
    pose_curr = keyframePosesUpdated.at(curr_node_idx);
    pcl::PointCloud<PointType>::Ptr poses(new pcl::PointCloud<PointType>());
    poses->resize(curr_node_idx + 1);
    for (int i = 0; i <= curr_node_idx; ++i) {
        poses->points[i].x = keyframePosesUpdated.at(i).x;
        poses->points[i].y = keyframePosesUpdated.at(i).y;
        poses->points[i].z = keyframePosesUpdated.at(i).z;
    }
    double t_curr = keyframeTimes.at(curr_node_idx);
    mKF.unlock();

    // 关键帧量级 2-3k,1Hz 重建 kd-tree 成本可忽略
    pcl::KdTreeFLANN<PointType> kd;
    kd.setInputCloud(poses);
    PointType q;
    q.x = pose_curr.x; q.y = pose_curr.y; q.z = pose_curr.z;
    std::vector<int> idx;
    std::vector<float> sqdis;
    kd.radiusSearch(q, loopDistanceRadius, idx, sqdis);
    if (idx.empty()) return;

    // 历史关键帧阈值:半径内、时间差满足要求、非自身,取最近的一帧
    int prev_node_idx = -1;
    double best_d = 1e9;
    for (size_t i = 0; i < idx.size(); ++i) {
        int id = idx[i];
        if (id >= curr_node_idx) continue;
        if (fabs(t_curr - keyframeTimes.at(id)) < loopDistanceTimeDiff) continue;
        double d = sqrt(sqdis[i]);
        if (d < best_d) { best_d = d; prev_node_idx = id; }
    }
    if (prev_node_idx < 0) return;

    cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx
         << " (dist " << best_d << " m)" << endl;

    // 去重:相同 (prev,curr) 只入队一次(静默,不刷屏)
    if (prev_node_idx == g_lastLoopPair.first && curr_node_idx == g_lastLoopPair.second) {
        return;
    }
    g_lastLoopPair = std::make_pair(prev_node_idx, curr_node_idx);

    mBuf.lock();
    scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
    mBuf.unlock();
} // performDistanceLoopClosure


void process_lcd(void)
{
    float loopClosureFrequency = 1.0; // can change 
    rclcpp::Rate rate(loopClosureFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        performDistanceLoopClosure();
        // 放包 30s 后一次性落盘(重定位数据库 + 全局地图),避免建图实时写盘
        // 与前端争抢 IO(2026-09-10)
        static bool scdbSavedFinal = false;
        if (!scdbSavedFinal && g_lastNewKeyframeWallTime > 0.0 &&
            rclcpp::Clock().now().seconds() - g_lastNewKeyframeWallTime > 30.0) {
            mKF.lock();
            // SC 描述子库:放包后批量生成(实时路径零 SC 开销,避免与前端争 CPU)
            for (size_t i = 0; i < keyframeLaserClouds.size(); ++i) {
                scManager.makeAndSaveScancontextAndKeys(*keyframeLaserClouds[i]);
            }
            scManager.saveToFile(save_directory + "sc_db.bin");
            // Scans/*.pcd:放包后批量写(relocalizer ICP 子图;实时小文件写盘
            // 的 fsync/目录元数据开销对前端不友好)
            for (size_t i = 0; i < keyframeLaserClouds.size(); ++i) {
                pcl::io::savePCDFileBinary(pgScansDirectory + padZeros(int(i)) + ".pcd",
                                           *keyframeLaserClouds[i]);
            }
            // 全局优化地图 PCD:关键帧点云 × 各自优化位姿摆全局,合并降采样
            pcl::PointCloud<PointType>::Ptr mapAll(new pcl::PointCloud<PointType>());
            for (size_t i = 0; i < keyframeLaserClouds.size(); ++i) {
                *mapAll += *local2global(keyframeLaserClouds[i], keyframePosesUpdated[i]);
            }
            pcl::PointCloud<PointType>::Ptr mapDS(new pcl::PointCloud<PointType>());
            downSizeFilterMapPGO.setInputCloud(mapAll);
            downSizeFilterMapPGO.filter(*mapDS);
            pcl::io::savePCDFileBinary(save_directory + "map.pcd", *mapDS);
            mKF.unlock();
            scdbSavedFinal = true;
            cout << "[SCDB] saved sc_db.bin + map.pcd (" << mapDS->size()
                 << " pts) to " << save_directory << endl;
        }
    }
} // process_lcd

void process_icp(void)
{
    while(rclcpp::ok())
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                RCLCPP_WARN(g_pgo_node->get_logger(), "Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                // runISAM2opt();
                mtxPosegraph.unlock();
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void process_viz_path(void)
{
    float hz = 10.0;
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
            // 2026-09-07 新增:当前帧点云按最新优化位姿发布(/aft_pgo_current_cloud),
            // 让实时点云"跟随优化后的位姿"(10Hz 循环,位姿本身 1Hz 刷新)
            pcl::PointCloud<PointType>::Ptr curCloud(new pcl::PointCloud<PointType>());
            mKF.lock();
            if (!keyframeLaserClouds.empty() &&
                keyframeLaserClouds.size() == keyframePosesUpdated.size())
            {
                int idx = int(keyframePosesUpdated.size()) - 1;
                *curCloud += *local2global(keyframeLaserClouds[idx], keyframePosesUpdated[idx]);
                mKF.unlock();
                sensor_msgs::msg::PointCloud2 curMsg;
                pcl::toROSMsg(*curCloud, curMsg);
                curMsg.header.frame_id = "odom";
                pubCurAftPGO->publish(curMsg);
            }
            else
            {
                mKF.unlock();
            }
        }
    }
}

void process_isam(void)
{
    float hz = 1;
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            bool hadFactors = (gtSAMgraph.size() > 0); // 2026-09-08:防刷屏判据——
            // 不能清 gtSAMgraphMade(它是"首节点已加"的一次性标志,prior 分支靠它判断,
            // 清了会让 prior 重复插入崩溃)。空图时只跳过打印/保存,优化照跑(空 update 无害)
            runISAM2opt();
            if (hadFactors) cout << "running isam2 optimization ..." << endl;
            mtxPosegraph.unlock();

            if (hadFactors) {
                saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
                saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
            }
        }
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 2; // sparse map visulalization to save computations 
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock(); 

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "odom";
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

void process_viz_map(void)
{
    float vizmapFrequency = 0.1; // 0.1 means run onces every 10s
    rclcpp::Rate rate(vizmapFrequency);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
} // pointcloud_viz


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("laserPGO");
    g_pgo_node = node;

	node->declare_parameter<std::string>("save_directory", "/");
    node->get_parameter<std::string>("save_directory", save_directory); // pose assignment every k m move
    node->declare_parameter<double>("sc_dist_thres", 0.2);
    node->get_parameter<double>("sc_dist_thres", scDistThres);
    node->declare_parameter<double>("sc_max_radius", 80.0);
    node->get_parameter<double>("sc_max_radius", scMaximumRadius);
    node->declare_parameter<double>("loop_fitness_score_thres", 0.3);
    node->get_parameter<double>("loop_fitness_score_thres", g_loop_fitness_score_thres);
    node->declare_parameter<double>("loop_icp_t_max", 5.0);
    node->get_parameter<double>("loop_icp_t_max", g_loop_icp_t_max);
    node->declare_parameter<double>("odom_trans_noise", 1e-6);
    node->get_parameter<double>("odom_trans_noise", odomTransNoise);
    node->declare_parameter<double>("loop_noise_score", 0.5);
    node->get_parameter<double>("loop_noise_score", g_loop_noise_score);
    node->declare_parameter<bool>("ground_z_en", false);
    node->get_parameter<bool>("ground_z_en", groundZEn);
    node->declare_parameter<double>("ground_z_sigma", 0.3);
    node->get_parameter<double>("ground_z_sigma", groundZSigma);
    node->declare_parameter<double>("ground_z_kappa", 1.0);
    node->get_parameter<double>("ground_z_kappa", groundZKappa);
    node->declare_parameter<double>("loop_distance_radius", 10.0);
    node->get_parameter<double>("loop_distance_radius", loopDistanceRadius);
    node->declare_parameter<double>("loop_distance_time_diff", 30.0);
    node->get_parameter<double>("loop_distance_time_diff", loopDistanceTimeDiff);
    pgKITTIformat = save_directory + "isam_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";
    auto unused = system((std::string("mkdir -p ") + save_directory).c_str()); // save_directory may not exist yet
    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out);
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);
    pgScansDirectory = save_directory + "Scans/";
    unused = system((std::string("exec rm -r ") + pgScansDirectory).c_str());
    unused = system((std::string("mkdir -p ") + pgScansDirectory).c_str());

	node->declare_parameter<double>("keyframe_meter_gap", 2.0);
    node->get_parameter<double>("keyframe_meter_gap", keyframeMeterGap); // pose assignment every k m move 
	node->declare_parameter<double>("keyframe_deg_gap", 10.0);
    node->get_parameter<double>("keyframe_deg_gap", keyframeDegGap); // pose assignment every k deg rot 
    keyframeRadGap = deg2rad(keyframeDegGap);


    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);


    float filter_size = 0.2; // 2026-09-08 0.4→0.2:Mid-360 室外点云稀疏(40m 量程+非重复扫描),
    // 关键帧入库降采样与 ICP 前降采样同源收紧,单帧 ~2000→~6000 点,协方差估计更稳
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
	node->declare_parameter<double>("mapviz_filter_size", 0.4);
    node->get_parameter<double>("mapviz_filter_size", mapVizFilterSize); // pose assignment every k frames 
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

	auto subLaserCloudFullRes = node->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered_local", rclcpp::QoS(10).best_effort(), laserCloudFullResHandler);
	auto subLaserOdometry = node->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);

	pubOdomAftPGO = node->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = node->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", 100);
	pubPathAftPGO = node->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = node->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);
	pubCurAftPGO = node->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_current_cloud", 100); // 2026-09-07 新增

    pubLoopScanLocal = node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal = node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);

	std::thread posegraph_slam {process_pg}; // pose graph construction
	std::thread lc_detection {process_lcd}; // loop closure detection 
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp 
	std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added. 

	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
	std::thread viz_path {process_viz_path}; // visualization - path (high frequency)

 	rclcpp::spin(node);

	// Join all threads before exit (avoids std::terminate from unjoined threads)
	posegraph_slam.join();
	lc_detection.join();
	icp_calculation.join();
	isam_update.join();
	viz_map.join();
	viz_path.join();

	rclcpp::shutdown();
	return 0;
}
