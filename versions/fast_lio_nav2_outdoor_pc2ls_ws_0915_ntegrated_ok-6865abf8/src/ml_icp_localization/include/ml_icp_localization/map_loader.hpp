#pragma once
// map_loader: 地图包加载(poses 文件 + Scans/*.pcd)与局部子图构造。
// 数据格式与建图产物约定(与 sc_pgo/alaserRelocalizer 一致,本包不含 SC):
//   - pose 文件(如 optimized_poses.txt):标准 KITTI 行主序,第 N 行 == 关键帧 N,
//     位姿 = T_map_body(body 系 = /cloud_registered_body 的 frame)
//   - Scans/%06d.pcd:body 系单帧点云,索引与 pose 行对齐
// 2026-09-10 实测确认文件为行主序(列主序解析会把 yaw 反号)。

#include "ml_icp_localization/point_types.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace ml_icp_loc
{


// KITTI 行主序一行 → 4x4(解析失败抛异常带行号,严禁 Identity 静默)
inline Eigen::Matrix4d KittiLineToMatrix4d(const std::string& line, int line_no)
{
    std::istringstream iss(line);
    std::vector<double> v(12);
    for (int i = 0; i < 12; i++) {
        if (!(iss >> v[i]))
            throw std::runtime_error("bad pose line " + std::to_string(line_no) + ": " + line);
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,0) = v[0]; T(0,1) = v[1]; T(0,2) = v[2]; T(0,3) = v[3];
    T(1,0) = v[4]; T(1,1) = v[5]; T(1,2) = v[6]; T(1,3) = v[7];
    T(2,0) = v[8]; T(2,1) = v[9]; T(2,2) = v[10]; T(2,3) = v[11];
    return T;
}

// 加载 pose 文件:poses_out[i] = 关键帧 i 的 T_map_body
inline bool LoadKeyframePoses(const std::string& pose_file,
                              std::vector<Eigen::Matrix4d>& poses_out)
{
    poses_out.clear();
    std::ifstream ifs(pose_file);
    if (!ifs.is_open()) return false;
    std::string line;
    int line_no = 0;
    while (std::getline(ifs, line)) {
        line_no++;
        if (line.empty()) continue;
        try {
            poses_out.push_back(KittiLineToMatrix4d(line, line_no));
        } catch (const std::exception& e) {
            std::cerr << "[icp_loc] pose DB parse failed: " << e.what() << std::endl;
            return false;
        }
    }
    return !poses_out.empty();
}

// 关键帧点云缓存:按需加载 + 无逐出(全地图 355 帧 × ~53KB ≈ 19MB)。
// 2Hz 配准相邻两轮子图 ~90% 帧重叠,缓存后稳态磁盘 I/O 归零。
class KeyframeCache
{
public:
    explicit KeyframeCache(const std::string& map_dir) : map_dir_(map_dir) {}

    // 返回关键帧 idx 的点云(body 系);未命中则从 Scans/%06d.pcd 加载并缓存。
    // 加载失败返回 nullptr 且不缓存(下次重试)。
    pcl::PointCloud<PointType>::Ptr Get(int idx)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto it = clouds_.find(idx);
        if (it != clouds_.end()) return it->second;

        std::stringstream ss;
        ss << map_dir_ << "/Scans/" << std::setw(6) << std::setfill('0') << idx << ".pcd";
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        if (pcl::io::loadPCDFile<PointType>(ss.str(), *cloud) == -1) return nullptr;
        clouds_[idx] = cloud;
        return cloud;
    }

private:
    std::string map_dir_;
    std::unordered_map<int, pcl::PointCloud<PointType>::Ptr> clouds_;
    std::mutex mtx_; // 防御性:当前单线程 executor,未来多线程下安全
};

// 候选帧 ±halfwidth 邻域合并成 map 系子图(各帧按自身 T_map_body 摆位),体素降采样
inline pcl::PointCloud<PointType>::Ptr BuildSubmapAround(
    KeyframeCache& cache,
    const std::vector<Eigen::Matrix4d>& poses,
    int center_idx, int halfwidth, double leaf)
{
    pcl::PointCloud<PointType>::Ptr submap(new pcl::PointCloud<PointType>());
    int start = std::max(0, center_idx - halfwidth);
    int end = std::min((int)poses.size() - 1, center_idx + halfwidth);
    for (int idx = start; idx <= end; idx++) {
        auto kf_cloud = cache.Get(idx);
        if (kf_cloud == nullptr) continue;
        pcl::PointCloud<PointType>::Ptr transformed(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*kf_cloud, *transformed, poses[idx]);
        *submap += *transformed;
    }
    if (submap->empty()) return submap;
    pcl::PointCloud<PointType>::Ptr ds(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(leaf, leaf, leaf);
    vg.setInputCloud(submap);
    vg.filter(*ds);
    return ds;
}

// 按预测位姿找最近关键帧索引(poses 线性扫描;规模几千时足够)
inline int NearestKeyframeIdx(const std::vector<Eigen::Matrix4d>& poses,
                              const Eigen::Matrix4d& query)
{
    int best = -1;
    double best_d = 1e18;
    for (size_t i = 0; i < poses.size(); i++) {
        double d = (poses[i].block<3,1>(0,3) - query.block<3,1>(0,3)).norm();
        if (d < best_d) { best_d = d; best = (int)i; }
    }
    return best;
}

// 地图包一次性加载 + 三库一致性校验(poses 行数 == Scans 文件数)。
// 失败返回 false,reason_out 说明原因(供启动即退出用)。
struct MapPackage
{
    std::vector<Eigen::Matrix4d> poses;
    int scan_count = 0;
};

inline bool LoadMapPackage(const std::string& map_dir,
                           const std::string& pose_file,
                           MapPackage& out, std::string& reason_out)
{
    out.poses.clear();
    out.scan_count = 0;
    std::string pose_path = map_dir + "/" + pose_file;
    if (!LoadKeyframePoses(pose_path, out.poses)) {
        reason_out = "failed to load pose file " + pose_path;
        return false;
    }
    for (size_t i = 0; i < out.poses.size(); i++) {
        std::stringstream ss;
        ss << map_dir << "/Scans/" << std::setw(6) << std::setfill('0') << i << ".pcd";
        std::ifstream f(ss.str());
        if (f.good()) out.scan_count++;
    }
    if ((int)out.poses.size() != out.scan_count) {
        reason_out = "DB index mismatch: poses " + std::to_string(out.poses.size())
                     + " vs scans " + std::to_string(out.scan_count);
        return false;
    }
    return true;
}

} // namespace ml_icp_loc
