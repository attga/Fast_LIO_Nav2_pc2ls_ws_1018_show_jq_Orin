// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "body_output.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   extrinsic_est_en = true; // Upstream default; MID360 profile explicitly disables it.
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string lid_topic, imu_topic;
// 默认保持独立建图接口；正式定位通过 URDF 查询内置 IMU 到底盘的安装关系。
string output_base_frame = "base_link", output_imu_frame = "base_link", output_lidar_frame = "lidar";
Eigen::Isometry3d T_base_imu = Eigen::Isometry3d::Identity();

double g_match_gate_sq = 5.0; // correspondence search-distance gate (m^2)

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false, tf_pub_en = true;

vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());


// Precision-preserving seconds -> rclcpp::Time (avoids double truncation at epoch scale)
inline rclcpp::Time sec_to_time(double seconds)
{
    uint32_t sec = static_cast<uint32_t>(seconds);
    uint32_t nanosec = static_cast<uint32_t>((seconds - sec) * 1e9 + 0.5);
    if (nanosec >= 1000000000u) { nanosec -= 1000000000u; sec += 1; }
    return rclcpp::Time(sec, nanosec);
}

// Global node pointer and logger for free functions
rclcpp::Node::SharedPtr g_node;
static rclcpp::Logger g_logger = rclcpp::get_logger("laserMapping");

void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_WARN(g_logger, "catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        RCLCPP_ERROR(g_logger, "lidar loop back, clear buffer");
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_pushed = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds());
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
    // Keep bounded debug samples; localization must continue when the log is full.
    if (runtime_pos_log && scan_count < MAXN) {
        s_plot11[scan_count++] = omp_get_wtime() - preprocess_start_time;
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        RCLCPP_ERROR(g_logger, "lidar loop back, clear buffer");
        lidar_buffer.clear();
        time_buffer.clear();
        lidar_pushed = false;
    }
    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

    if (!time_sync_en && abs(last_timestamp_imu - lidar_end_time) > 10.0)
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar scan end time: %lf",last_timestamp_imu, lidar_end_time);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    if (runtime_pos_log && scan_count < MAXN) {
        s_plot11[scan_count++] = omp_get_wtime() - preprocess_start_time;
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(sensor_msgs::msg::Imu::SharedPtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        sec_to_time(timediff_lidar_wrt_imu + rclcpp::Time(msg_in->header.stamp).seconds());
    }

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_WARN(g_logger, "imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        if(meas.lidar->points.size() <= 1)
        {
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            RCLCPP_WARN(g_logger, "Drop sparse scan and its timestamp");
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        // Livox 点时间可能乱序；同一个结束时刻用于同步、传播和发布。
        const auto last_point = std::max_element(meas.lidar->begin(), meas.lidar->end(),
            [](const PointType& a, const PointType& b) { return a.curvature < b.curvature; });
        lidar_end_time = meas.lidar_beg_time + last_point->curvature / 1000.0;
        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = sec_to_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
    // 去畸变仍在原传感器系完成；这里只做雷达→IMU→底盘的刚体转换。
    PointCloudXYZI cloud_base = *feats_undistort;
    const Eigen::Matrix3d R_base_lidar = T_base_imu.linear() * state_point.offset_R_L_I.toRotationMatrix();
    const Eigen::Vector3d t_base_lidar = T_base_imu * state_point.offset_T_L_I;
    for (auto& point : cloud_base.points) {
        const Eigen::Vector3d transformed = R_base_lidar * Eigen::Vector3d(point.x, point.y, point.z) + t_base_lidar;
        point.x = transformed.x(); point.y = transformed.y(); point.z = transformed.z();
    }
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(cloud_base, laserCloudmsg);
    laserCloudmsg.header.stamp = sec_to_time(lidar_end_time);
    laserCloudmsg.header.frame_id = output_base_frame;
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_lidar(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_lidar)
{
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_undistort, laserCloudmsg);
    laserCloudmsg.header.stamp = sec_to_time(lidar_end_time);
    laserCloudmsg.header.frame_id = output_lidar_frame;
    pubLaserCloudFull_lidar->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = sec_to_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "odom";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
    sensor_msgs::msg::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = sec_to_time(lidar_end_time);
    laserCloudMap.header.frame_id = "odom";
    pubLaserCloudMap->publish(laserCloudMap);
}

// 不等待、不取最新动态 TF；缺少扫描时刻的安装关系时跳过相关发布。
bool prepare_output_transform(tf2_ros::Buffer& buffer)
{
    if (output_base_frame == output_imu_frame) {
        T_base_imu.setIdentity();
        return true;
    }
    try {
        const auto mount = buffer.lookupTransform(output_base_frame, output_imu_frame,
                                                   sec_to_time(lidar_end_time));
        const auto& r = mount.transform.rotation;
        const auto& t = mount.transform.translation;
        Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
        const Eigen::Vector3d translation(t.x, t.y, t.z);
        if (!q.coeffs().allFinite() || !translation.allFinite() ||
            std::abs(q.squaredNorm() - 1.0) > 1e-3) {
            RCLCPP_ERROR_THROTTLE(g_logger, *g_node->get_clock(), 5000,
                                 "invalid output mount TF; skip base pose/cloud");
            return false;
        }
        T_base_imu.linear() = q.normalized().toRotationMatrix();
        T_base_imu.translation() = translation;
        return true;
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(g_logger, *g_node->get_clock(), 5000,
                            "output mount TF %s <- %s unavailable; skip base pose/cloud: %s",
                            output_base_frame.c_str(), output_imu_frame.c_str(), error.what());
        return false;
    }
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = output_base_frame;
    odomAftMapped.header.stamp = sec_to_time(lidar_end_time);
    const Eigen::Matrix3d R_odom_imu = state_point.rot.toRotationMatrix();
    const auto T_odom_base = fast_lio::BasePose(R_odom_imu, state_point.pos, T_base_imu);
    const Eigen::Quaterniond orientation(T_odom_base.linear());
    auto& pose = odomAftMapped.pose.pose;
    pose.position.x = T_odom_base.translation().x();
    pose.position.y = T_odom_base.translation().y();
    pose.position.z = T_odom_base.translation().z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    // 包含安装偏移导致的位置/姿态互协方差；安装外参自身的不确定性暂不建模。
    const auto& P = kf.get_P();
    const auto J = fast_lio::BasePoseJacobian(R_odom_imu, T_base_imu);
    const fast_lio::PoseCovariance pose_covariance =
        J * P.block<6, 6>(0, 0) * J.transpose();
    for (int row = 0; row < 6; ++row)
    {
        for (int col = 0; col < 6; ++col)
        {
            odomAftMapped.pose.covariance[row * 6 + col] = pose_covariance(row, col);
        }
    }
    // Publish only after this scan's pose AND covariance have been populated.
    pubOdomAftMapped->publish(odomAftMapped);

    if (tf_pub_en)
    {
        static auto br = std::make_shared<tf2_ros::TransformBroadcaster>(*g_node);
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = odomAftMapped.header.stamp;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = output_base_frame;
        transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
        transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
        transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
        transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
        transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
        br->sendTransform(transformStamped);
    }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    msg_body_pose.pose = odomAftMapped.pose.pose;
    msg_body_pose.header.stamp = sec_to_time(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";
    path.header.stamp = msg_body_pose.header.stamp;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > g_match_gate_sq ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        RCLCPP_WARN(g_logger, "No effective points; skip LiDAR observation update");
        return;
    }
    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        // Match upstream FAST-LIO: known rigid IMU-LiDAR extrinsics are not estimated.
        if (extrinsic_est_en) {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("laserMapping");
    g_logger = g_node->get_logger();
    output_base_frame = g_node->declare_parameter<std::string>("output.base_frame", "base_link");
    output_imu_frame = g_node->declare_parameter<std::string>("output.imu_frame", "base_link");
    output_lidar_frame = g_node->declare_parameter<std::string>("output.lidar_frame", "lidar");
    if (output_base_frame.empty() || output_imu_frame.empty() || output_lidar_frame.empty()) {
        RCLCPP_ERROR(g_logger, "output frame names must not be empty");
        rclcpp::shutdown();
        return 1;
    }
    tf2_ros::Buffer output_tf_buffer(g_node->get_clock());
    // 共用现有 spin_some 回调循环，不新增线程，不阻塞前端处理。
    tf2_ros::TransformListener output_tf_listener(output_tf_buffer, g_node, false);

    g_node->declare_parameter<bool>("publish.scan_publish_en", true);
    g_node->get_parameter<bool>("publish.scan_publish_en", scan_pub_en);
    g_node->declare_parameter<bool>("publish.dense_publish_en", false);
    g_node->get_parameter<bool>("publish.dense_publish_en", dense_pub_en);
    g_node->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    g_node->get_parameter<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en);
    // odom->base_link TF 开关(默认开;EKF 接管 TF 时置 false)
    g_node->declare_parameter<bool>("publish.tf_publish_en", true);
    g_node->get_parameter<bool>("publish.tf_publish_en", tf_pub_en);
    g_node->declare_parameter<int>("max_iteration", 4);
    g_node->get_parameter<int>("max_iteration", NUM_MAX_ITERATIONS);
    g_node->declare_parameter<double>("match_gate_sq", 5.0);
    g_node->get_parameter<double>("match_gate_sq", g_match_gate_sq);
    g_node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    g_node->get_parameter<std::string>("common.lid_topic", lid_topic);
    g_node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    g_node->get_parameter<std::string>("common.imu_topic", imu_topic);
    g_node->declare_parameter<bool>("common.time_sync_en", false);
    g_node->get_parameter<bool>("common.time_sync_en", time_sync_en);
    g_node->declare_parameter<double>("filter_size_corner", 0.5);
    g_node->get_parameter<double>("filter_size_corner", filter_size_corner_min);
    g_node->declare_parameter<double>("filter_size_surf", 0.5);
    g_node->get_parameter<double>("filter_size_surf", filter_size_surf_min);
    g_node->declare_parameter<double>("filter_size_map", 0.5);
    g_node->get_parameter<double>("filter_size_map", filter_size_map_min);
    g_node->declare_parameter<double>("cube_side_length", 200.0);
    g_node->get_parameter<double>("cube_side_length", cube_len);
    g_node->declare_parameter<float>("mapping.det_range", 300.f);
    g_node->get_parameter<float>("mapping.det_range", DET_RANGE);
    g_node->declare_parameter<double>("mapping.fov_degree", 180.0);
    g_node->get_parameter<double>("mapping.fov_degree", fov_deg);
    g_node->declare_parameter<double>("mapping.gyr_cov", 0.1);
    g_node->get_parameter<double>("mapping.gyr_cov", gyr_cov);
    g_node->declare_parameter<double>("mapping.acc_cov", 0.1);
    g_node->get_parameter<double>("mapping.acc_cov", acc_cov);
    g_node->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    g_node->get_parameter<double>("mapping.b_gyr_cov", b_gyr_cov);
    g_node->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    g_node->get_parameter<double>("mapping.b_acc_cov", b_acc_cov);
    g_node->declare_parameter<double>("preprocess.blind", 0.01);
    g_node->get_parameter<double>("preprocess.blind", p_pre->blind);
    g_node->declare_parameter<int>("preprocess.lidar_type", AVIA);
    g_node->get_parameter<int>("preprocess.lidar_type", p_pre->lidar_type);
    g_node->declare_parameter<int>("preprocess.scan_line", 16);
    g_node->get_parameter<int>("preprocess.scan_line", p_pre->N_SCANS);
    g_node->declare_parameter<int>("point_filter_num", 2);
    g_node->get_parameter<int>("point_filter_num", p_pre->point_filter_num);
    g_node->declare_parameter<bool>("feature_extract_enable", false);
    g_node->get_parameter<bool>("feature_extract_enable", p_pre->feature_enabled);
    g_node->declare_parameter<bool>("runtime_pos_log_enable", false);
    g_node->get_parameter<bool>("runtime_pos_log_enable", runtime_pos_log);
    g_node->declare_parameter<bool>("pcd_save_enable", false);
    g_node->get_parameter<bool>("pcd_save_enable", pcd_save_en);
    extrinsic_est_en = g_node->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    RCLCPP_INFO(g_logger, "LiDAR->IMU extrinsic estimation: %s", extrinsic_est_en ? "ONLINE" : "FIXED");
    g_node->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
    g_node->get_parameter<std::vector<double>>("mapping.extrinsic_T", extrinT);
    g_node->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());
    g_node->get_parameter<std::vector<double>>("mapping.extrinsic_R", extrinR);
    if (extrinT.size() != 3 || extrinR.size() != 9 ||
        !std::all_of(extrinT.begin(), extrinT.end(), [](double v) { return std::isfinite(v); }) ||
        !std::all_of(extrinR.begin(), extrinR.end(), [](double v) { return std::isfinite(v); })) {
        RCLCPP_ERROR(g_logger, "mapping.extrinsic_T/R must contain 3/9 finite values");
        return 1;
    }
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    path.header.stamp    = g_node->now();
    path.header.frame_id ="odom";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    const std::string log_directory = g_node->declare_parameter<std::string>(
        "debug_log_directory", root_dir + "/Log");
    p_imu->log_directory = log_directory;
    string pos_log_dir = log_directory + "/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");
    if (!fp) {
        RCLCPP_ERROR(g_logger, "debug_log_directory must exist and be writable: %s", log_directory.c_str());
        return 1;
    }

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(log_directory + "/mat_pre.txt",ios::out);
    fout_out.open(log_directory + "/mat_out.txt",ios::out);
    fout_dbg.open(log_directory + "/dbg.txt",ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS2 subscribe initialization ***/
    rclcpp::SubscriptionBase::SharedPtr sub_pcl;
    if (p_pre->lidar_type == AVIA) {
        sub_pcl = g_node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, rclcpp::QoS(100).best_effort(), livox_pcl_cbk);
    } else {
        sub_pcl = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::QoS(100).best_effort(), standard_pcl_cbk);
    }
    auto sub_imu = g_node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(1000).best_effort(), imu_cbk);

    auto pubLaserCloudFull = g_node->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered", 100);
    auto pubLaserCloudFull_body = g_node->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered_body", 100);
    auto pubLaserCloudFull_lidar = g_node->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered_lidar", 100);
    auto pubLaserCloudEffect = g_node->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_effected", 100);
    auto pubLaserCloudMap = g_node->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/Laser_map", 100);
    auto pubOdomAftMapped = g_node->create_publisher<nav_msgs::msg::Odometry>
            ("/lidar_odometry", 100);
    auto pubPath = g_node->create_publisher<nav_msgs::msg::Path>
            ("/path", 100);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);
    while (rclcpp::ok())
    {
        if (flg_exit) break;
        rclcpp::spin_some(g_node);

        if(sync_packages(Measures))
        {
            if (flg_reset)
            {
                RCLCPP_WARN(g_logger, "reset when rosbag play back");
                p_imu->Reset();
                flg_reset = false;
                Measures.imu.clear();
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            // localization mode: the prebuilt map must NOT be segmented/deleted
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            if (feats_down_size < 5) {
                RCLCPP_WARN(g_logger, "Too few downsampled points; skip LiDAR update");
                continue;
            }

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            const bool output_ready = prepare_output_transform(output_tf_buffer);
            if (output_ready) publish_odometry(pubOdomAftMapped);
            // Preserve the target vehicle's low-latency cloud path for Nav2/registration.
            if (output_ready && scan_pub_en && scan_body_pub_en) {
                publish_frame_body(pubLaserCloudFull_body);
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

            /******* Publish points *******/
            if (output_ready) publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) {
              publish_frame_lidar(pubLaserCloudFull_lidar);
            }

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                fout_dbg << std::setprecision(12) << lidar_end_time << " "
                         << feats_down_size << " " << effct_feat_num << " "
                         << res_mean_last << " " << state_point.pos.transpose() << " "
                         << state_point.ba.transpose() << " " << state_point.grav << std::endl;
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                if (time_log_counter < MAXN) {
                    T1[time_log_counter] = Measures.lidar_beg_time;
                    s_plot[time_log_counter] = t5 - t0;
                    s_plot2[time_log_counter] = feats_undistort->points.size();
                    s_plot3[time_log_counter] = kdtree_incremental_time;
                    s_plot4[time_log_counter] = kdtree_search_time;
                    s_plot5[time_log_counter] = kdtree_delete_counter;
                    s_plot6[time_log_counter] = kdtree_delete_time;
                    s_plot7[time_log_counter] = kdtree_size_st;
                    s_plot8[time_log_counter] = kdtree_size_end;
                    s_plot9[time_log_counter] = aver_time_consu;
                    s_plot10[time_log_counter] = add_point_size;
                    time_log_counter ++;
                }
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        rate.sleep();
    }

    /**************** save map ****************/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = log_directory + "/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        if (!fp2) {
            RCLCPP_ERROR(g_logger, "Cannot write timing log: %s", log_dir.c_str());
            rclcpp::shutdown();
            return 1;
        }
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    rclcpp::shutdown();
    return 0;
}
