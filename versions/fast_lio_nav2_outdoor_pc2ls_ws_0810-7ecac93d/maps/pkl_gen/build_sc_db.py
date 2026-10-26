#!/usr/bin/env python3
# coding=utf8
"""
ScanContext 离线数据库构建器 (多朝向版本)

修改说明 (相较旧版):
  旧版: 每个虚拟关键帧位置只生成 1 个 Yaw=0 的 SC 条目，
        方向信息完全依赖检索时的列移位推断，精度差且对称场景歧义大。

  新版: 对每个位置，按 N_ORIENTATIONS 个方向（默认 6 个，每 60°）
        分别将局部点云旋转到对应坐标系后生成 SC，并将真实旋转矩阵存入 T。
        检索时 query() 返回的 T_corrected 即包含真实的方向信息，
        sc_distance() 的列移位仅做 ±(360/N/2)° 范围内的微小修正。

  性能优化: 新增 --map_voxel 参数，可在建 KD-Tree 前预降采样地图。
        对室外大型地图（>2M 原始点）效果显著（从数小时压缩到分钟级）。

使用方法:
  # 室内场景 (小图，无需预降采样)
  python build_sc_db.py \\
      --map   /path/to/cloudGlobal.pcd \\
      --db    /path/to/sc_database_indoor_multiori.pkl \\
      --step 3.0 --radius 80.0 --orientations 6

  # 室外场景 (大图，必须用 --map_voxel 加速 KD-Tree)
  python build_sc_db.py \\
      --map       /path/to/GlobalMap.pcd \\
      --db        /path/to/sc_database_outdoor_multiori.pkl \\
      --step 5.0 --radius 80.0 --orientations 6 \\
      --map_voxel 0.3
"""

import os
import sys
import time
import numpy as np
import open3d as o3d
import argparse

from scan_context import ScanContextManager, SCDatabase


def _rot_z(yaw_deg: float) -> np.ndarray:
    """生成绕 Z 轴旋转 yaw_deg 度的 3×3 旋转矩阵 R(yaw)。"""
    rad = np.deg2rad(yaw_deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [ c, -s, 0],
        [ s,  c, 0],
        [ 0,  0, 1],
    ], dtype=np.float32)


def build_offline_database(
    map_path:       str,
    db_path:        str,
    sample_step:    float = 3.0,
    crop_radius:    float = 80.0,
    n_orientations: int   = 6,
    map_voxel_size: float = 0.0,
):
    """构建多朝向 ScanContext 数据库。

    Args:
        map_path:       全局地图 PCD 文件路径
        db_path:        输出 pkl 数据库路径
        sample_step:    虚拟关键帧采样间距 (m)
                          室内 3m / 室外 5m 推荐
        crop_radius:    局部地图裁剪半径 (m)
        n_orientations: 每个位置生成的朝向数量
                          默认 6 = 每 60° 一个（0°,60°,120°,180°,240°,300°）
        map_voxel_size: 建 KD-Tree 前预降采样体素尺寸 (m)
                          0.0 = 不降采样（室内小图）
                          0.3 = 推荐室外大图（4M点 → 19万点，建库加速 30x）
    """
    print(f"Loading map: {map_path}")
    if not os.path.exists(map_path):
        print(f"Error: Map file {map_path} does not exist.")
        return

    pcd_full = o3d.io.read_point_cloud(map_path)
    pts_full = np.asarray(pcd_full.points, dtype=np.float32)
    print(f"Map loaded: {len(pts_full)} points")

    # ── 0. （可选）预降采样，用于 KD-Tree 搜索加速 ────────────────────────────
    #   只影响 KD-Tree 搜索性能，SC 描述子质量略有变化但在可接受范围内
    if map_voxel_size > 0.0:
        print(f"Pre-downsampling map for KD-Tree with voxel_size={map_voxel_size}m ...")
        pcd_for_kdtree = pcd_full.voxel_down_sample(map_voxel_size)
        pts = np.asarray(pcd_for_kdtree.points, dtype=np.float32)
        print(f"  Downsampled: {len(pts_full)} -> {len(pts)} points "
              f"(ratio={len(pts)/len(pts_full)*100:.1f}%)")
    else:
        pcd_for_kdtree = pcd_full
        pts = pts_full
        print(f"  No pre-downsampling (map_voxel_size=0.0), using all {len(pts)} points.")

    # ── 1. 过滤靠近地面的点，用作虚拟关键帧的采样池 ────────────────────────────
    print("Filtering ground-level points for virtual keyframe generation...")
    z_mask    = (pts[:, 2] > -1.0) & (pts[:, 2] < 1.0)
    ground_pts = pts[z_mask]

    if len(ground_pts) == 0:
        print("Warning: No points in Z range [-1.0, 1.0]. Using all points for sampling.")
        ground_pts = pts

    temp_pcd       = o3d.geometry.PointCloud()
    temp_pcd.points = o3d.utility.Vector3dVector(ground_pts)

    # ── 2. 体素降采样得到均匀分布的虚拟关键帧位置 ───────────────────────────────
    print(f"Downsampling to grid with step={sample_step}m ...")
    down_pcd      = temp_pcd.voxel_down_sample(voxel_size=sample_step)
    virtual_poses = np.asarray(down_pcd.points, dtype=np.float32)
    num_poses     = len(virtual_poses)
    print(f"Generated {num_poses} virtual keyframe locations.")

    # ── 3. 构建 KD-Tree（使用预降采样后的地图）──────────────────────────────────
    print("Building KD-Tree for global map ...")
    map_kdtree = o3d.geometry.KDTreeFlann(pcd_for_kdtree)

    sc_manager = ScanContextManager(max_radius=crop_radius)
    sc_db      = SCDatabase(sc_manager)

    # ── 4. 计算多朝向旋转矩阵列表 ────────────────────────────────────────────────
    #
    #   旋转语义说明:
    #     对位置 pose_pt，将局部点云平移到以其为原点后，再绕 Z 轴旋转 yaw_deg。
    #     rotation: R(yaw) 将地图系向量转换到"机器人朝向 yaw_deg"的局部系。
    #
    #   存入 T_map_to_body:
    #     T[:3,:3] = R(yaw)  记录参考朝向
    #     T[:3, 3] = pose_pt 记录位置
    #
    #   检索时 query() 中:
    #     T_corrected[:3,:3] = cand_pose[:3,:3] @ R_shift
    #   其中 R_shift 是 sc_distance() 列移位对应的微小 Yaw 修正量，
    #   cand_pose[:3,:3] = R(yaw_k) 是数据库中存储的参考朝向。
    #   combined: R_body = R(yaw_k - shift * sector_step) → 真实机器人朝向。
    #
    yaw_step = 360.0 / n_orientations
    yaw_list  = [yaw_step * k for k in range(n_orientations)]
    R_list    = [_rot_z(y) for y in yaw_list]

    total_entries = num_poses * n_orientations
    print(f"Orientations per position : {n_orientations} "
          f"({', '.join(f'{y:.0f}' for y in yaw_list)} deg)")
    print(f"Total DB entries          : {num_poses} x {n_orientations} = {total_entries}")
    print("Building ScanContext database. This may take a few minutes ...")
    start_time = time.time()

    for i, pose_pt in enumerate(virtual_poses):
        if i % 50 == 0 and i > 0:
            elapsed = time.time() - start_time
            pct     = (i / num_poses) * 100
            eta     = (elapsed / i) * (num_poses - i)
            print(f"  [{pct:5.1f}%] pos {i}/{num_poses}  "
                  f"elapsed={elapsed:.0f}s  ETA={eta:.0f}s  "
                  f"db={len(sc_db.sc_list)}")
        elif i == 0:
            print(f"  [  0.0%] pos 0/{num_poses}  starting ...")

        # ── 3a. 裁剪局部点云 ─────────────────────────────────────────────────
        [k, idx, _] = map_kdtree.search_radius_vector_3d(pose_pt, crop_radius)
        if k < 100:
            continue

        local_pts          = pts[np.asarray(idx, dtype=np.int64)]   # (k, 3)
        local_pts_centered = local_pts - pose_pt                     # 平移到局部系

        # ── 3b. 对每个朝向生成一个 SC 条目 ─────────────────────────────────
        for yaw_deg, R_mat in zip(yaw_list, R_list):
            # 将点云旋转到"机器人朝向 yaw_deg"的坐标系
            local_pts_rotated = (R_mat @ local_pts_centered.T).T   # (k, 3)

            # 生成该朝向下的 SC 描述子
            sc = sc_manager.ptcloud2sc(local_pts_rotated)

            # 构造 4×4 T_map_to_body
            T          = np.eye(4, dtype=np.float32)
            T[:3, :3]  = R_mat        # 旋转：记录参考朝向
            T[0, 3]    = pose_pt[0]
            T[1, 3]    = pose_pt[1]
            T[2, 3]    = pose_pt[2]

            sc_db.add(sc, T)

    sc_db.build_tree()

    elapsed_total = time.time() - start_time
    print()
    print(f"Database built:")
    print(f"  Positions   : {num_poses}")
    print(f"  Orientations: {n_orientations}")
    print(f"  Total entries: {len(sc_db.sc_list)}")
    print(f"  Time elapsed: {elapsed_total:.1f}s  ({elapsed_total/60:.1f} min)")
    print(f"Saving to {db_path} ...")
    sc_db.save(db_path)
    print("Done! OK")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Offline ScanContext Database Builder (Multi-Orientation)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--map",  type=str, required=True,
                        help="Path to GlobalMap.pcd (or cloudGlobal.pcd)")
    parser.add_argument("--db",   type=str, default="sc_database.pkl",
                        help="Output path for sc_database.pkl")
    parser.add_argument("--step", type=float, default=3.0,
                        help="Virtual keyframe grid sampling step (m). "
                             "Recommended: 3.0 indoor / 5.0 outdoor")
    parser.add_argument("--radius", type=float, default=80.0,
                        help="Local map crop radius (m)")
    parser.add_argument("--orientations", type=int, default=6,
                        help="Orientations per position (6=every 60deg, 4=every 90deg)")
    parser.add_argument("--map_voxel", type=float, default=0.0,
                        help="Pre-downsample map before KD-Tree (m). "
                             "0.0=off (indoor). 0.3 strongly recommended for outdoor "
                             "maps >1M pts (reduces build time from hours to minutes).")

    args = parser.parse_args()

    build_offline_database(
        map_path       = args.map,
        db_path        = args.db,
        sample_step    = args.step,
        crop_radius    = args.radius,
        n_orientations = args.orientations,
        map_voxel_size = args.map_voxel,
    )
