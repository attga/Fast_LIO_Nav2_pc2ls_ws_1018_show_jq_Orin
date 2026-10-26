#!/usr/bin/env python3
# coding=utf8
import os
import sys
import time
import numpy as np
import open3d as o3d
import argparse

from scan_context import ScanContextManager, SCDatabase

def build_offline_database(map_path: str, db_path: str, sample_step: float = 3.0, crop_radius: float = 80.0):
    print(f"Loading map: {map_path}")
    if not os.path.exists(map_path):
        print(f"Error: Map file {map_path} does not exist.")
        return

    pcd = o3d.io.read_point_cloud(map_path)
    pts = np.asarray(pcd.points)
    print(f"Map loaded: {len(pts)} points")

    # 1. 过滤出靠近地面的点，用作虚拟关键帧的采样池
    # 假设 LiDAR 轨迹在 Z 轴 -1.0 到 1.0 之间
    print("Filtering ground points for virtual keyframe generation...")
    z_mask = (pts[:, 2] > -1.0) & (pts[:, 2] < 1.0)
    ground_pts = pts[z_mask]
    
    if len(ground_pts) == 0:
        print("Warning: No points found in Z range [-1.0, 1.0]. Using all points for sampling.")
        ground_pts = pts

    temp_pcd = o3d.geometry.PointCloud()
    temp_pcd.points = o3d.utility.Vector3dVector(ground_pts)

    # 2. 对地面点进行体素降采样，得到均匀分布的虚拟关键帧位置
    print(f"Downsampling to grid with step={sample_step}m...")
    down_pcd = temp_pcd.voxel_down_sample(voxel_size=sample_step)
    virtual_poses = np.asarray(down_pcd.points)
    num_poses = len(virtual_poses)
    print(f"Generated {num_poses} virtual keyframe locations.")

    # 构建全局地图的 KD-Tree 用于快速裁剪局部点云
    print("Building KD-Tree for global map...")
    map_kdtree = o3d.geometry.KDTreeFlann(pcd)

    sc_manager = ScanContextManager(max_radius=crop_radius)
    sc_db = SCDatabase(sc_manager)

    print("Building ScanContext database. This may take a few minutes...")
    start_time = time.time()
    
    for i, pose_pt in enumerate(virtual_poses):
        if i % 100 == 0:
            print(f"  Processing {i}/{num_poses} ({(i/num_poses)*100:.1f}%) ...")
            
        # 3. 裁剪局部点云 (半径 crop_radius)
        # 使用 KD-Tree 搜索半径内的点
        [k, idx, _] = map_kdtree.search_radius_vector_3d(pose_pt, crop_radius)
        if k < 100:
            continue
            
        local_pts = pts[idx]
        
        # 4. 转换到局部坐标系 (假设 Yaw=0, Roll=0, Pitch=0)
        # 因为 ScanContext 具有旋转不变性，虚拟关键帧的初始 Yaw 为 0 不影响检索，
        # 检索时会自动算出实际的 Yaw 偏移量。
        local_pts_shifted = local_pts - pose_pt

        # 5. 生成 ScanContext
        sc = sc_manager.ptcloud2sc(local_pts_shifted)
        
        # 6. 生成 4x4 变换矩阵 T_map_to_body
        T = np.eye(4, dtype=np.float32)
        T[0, 3] = pose_pt[0]
        T[1, 3] = pose_pt[1]
        T[2, 3] = pose_pt[2]
        
        sc_db.add(sc, T)

    sc_db.build_tree()
    
    print(f"Database built with {len(sc_db.sc_list)} nodes in {time.time()-start_time:.1f}s.")
    print(f"Saving database to {db_path} ...")
    sc_db.save(db_path)
    print("Done! \u2705")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Offline ScanContext Database Builder")
    parser.add_argument("--map", type=str, required=True, help="Path to GlobalMap.pcd")
    parser.add_argument("--db", type=str, default="sc_database.pkl", help="Path to save sc_database.pkl")
    parser.add_argument("--step", type=float, default=3.0, help="Grid sampling step (meters)")
    parser.add_argument("--radius", type=float, default=80.0, help="Crop radius for local map (meters)")
    
    args = parser.parse_args()
    
    build_offline_database(args.map, args.db, args.step, args.radius)
