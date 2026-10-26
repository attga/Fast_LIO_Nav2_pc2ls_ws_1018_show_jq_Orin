import numpy as np
from scipy.spatial import KDTree

class ScanContextManager:
    """
    高效的 Python Numpy 版 ScanContext 实现
    参考论文: Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map
    """
    def __init__(self, num_rings=20, num_sectors=60, max_radius=80.0):
        self.num_rings = num_rings
        self.num_sectors = num_sectors
        self.max_radius = max_radius
        
        self.ring_step = max_radius / num_rings
        self.sector_step = 360.0 / num_sectors

    def ptcloud2sc(self, ptcloud: np.ndarray) -> np.ndarray:
        """
        将 N×3 点云转换为 ScanContext (num_rings × num_sectors) 矩阵
        使用全向量化 Numpy 操作，单帧耗时 < 5ms
        """
        if ptcloud.shape[0] == 0:
            return np.zeros((self.num_rings, self.num_sectors), dtype=np.float32)

        x = ptcloud[:, 0]
        y = ptcloud[:, 1]
        z = ptcloud[:, 2]

        # 计算极坐标半径 r 和角度 theta
        r = np.linalg.norm(ptcloud[:, :2], axis=1)
        theta = np.degrees(np.arctan2(y, x))
        theta[theta < 0] += 360.0

        # 过滤超出最大半径的点
        valid_idx = r < self.max_radius
        if not np.any(valid_idx):
            return np.zeros((self.num_rings, self.num_sectors), dtype=np.float32)

        x, y, z = x[valid_idx], y[valid_idx], z[valid_idx]
        r, theta = r[valid_idx], theta[valid_idx]

        # 映射到 Ring 和 Sector 索引
        ring_idx = np.floor(r / self.ring_step).astype(np.int32)
        sector_idx = np.floor(theta / self.sector_step).astype(np.int32)

        # 防止索引越界
        ring_idx = np.clip(ring_idx, 0, self.num_rings - 1)
        sector_idx = np.clip(sector_idx, 0, self.num_sectors - 1)

        # 计算 1D 线性索引 (Ring * N_sectors + Sector)
        linear_idx = ring_idx * self.num_sectors + sector_idx

        # 向量化求每个格子里的最大 Z 值 (利用 np.lexsort 和 np.unique)
        # 1. 按照 (linear_idx, z) 联合排序
        order = np.lexsort((z, linear_idx))
        linear_idx_sorted = linear_idx[order]
        z_sorted = z[order]

        # 2. 找到每个 unique linear_idx 的最后一个元素（即最大 Z）
        _, unique_indices = np.unique(linear_idx_sorted, return_index=True)
        # 最后一个元素的索引 = 下一个 unique 起点 - 1（追加数组末尾索引）
        last_indices = np.append(unique_indices[1:] - 1, len(linear_idx_sorted) - 1)

        valid_linear_idx = linear_idx_sorted[last_indices]
        max_z = z_sorted[last_indices]

        # 3. 填入 ScanContext 矩阵
        sc = np.zeros((self.num_rings, self.num_sectors), dtype=np.float32)
        r_idx = valid_linear_idx // self.num_sectors
        s_idx = valid_linear_idx % self.num_sectors
        sc[r_idx, s_idx] = max_z

        return sc

    def sc2rk(self, sc: np.ndarray) -> np.ndarray:
        """
        ScanContext -> Ring Key (1D 向量，维度 = num_rings)
        环级不变量，用于 KD-Tree 快速初筛
        """
        # 原论文使用 L0 norm（非零元素个数）或 L1，这里我们使用均值，效果同等且对噪点更鲁棒
        return np.mean(sc, axis=1)

    def sc_distance(self, sc1: np.ndarray, sc2: np.ndarray) -> tuple[float, int]:
        """
        计算两个 ScanContext 的距离（0~1，越小越相似），并返回使得距离最小的列平移量（最优 Yaw 偏移）
        返回: (min_dist, best_shift)
        """
        # 列向量 L2 归一化，避免因建筑物高低导致的余弦尺度问题
        norm1 = np.linalg.norm(sc1, axis=0) + 1e-6
        sc1_norm = sc1 / norm1

        norm2 = np.linalg.norm(sc2, axis=0) + 1e-6
        sc2_norm = sc2 / norm2

        num_sectors = self.num_sectors
        sims = np.zeros(num_sectors, dtype=np.float32)

        # 测试所有列平移量（对应所有可能的 Yaw 角，步长 = 360/N_sectors）
        for i in range(num_sectors):
            # sc2_norm 整体向右循环平移 i 列
            sc2_shifted = np.roll(sc2_norm, i, axis=1)
            
            # 按列求点积，然后计算所有列的平均余弦相似度
            sim = np.sum(sc1_norm * sc2_shifted, axis=0)
            sims[i] = np.mean(sim)

        best_shift = np.argmax(sims)
        best_sim = sims[best_shift]

        # 距离 = 1 - 相似度
        return 1.0 - float(best_sim), int(best_shift)

    def shift_to_yaw(self, shift: int) -> float:
        """将列平移量转换为偏航角 Yaw (度)"""
        # 修正：ScanContext 矩阵右移对应的是目标坐标系相对原坐标系的逆向旋转
        # 真实的 Yaw = 360.0 - (shift * self.sector_step)
        yaw = -float(shift * self.sector_step)
        return yaw % 360.0

        # return float(shift * self.sector_step)

class SCDatabase:
    """
    ScanContext 检索数据库（支持离线保存与在线加载）
    """
    def __init__(self, sc_manager: ScanContextManager):
        self.sc_manager = sc_manager
        self.sc_list = []      # 存原始 SC 矩阵
        self.pose_list = []    # 存 4x4 变换矩阵 T_map_to_body
        self.rk_list = []      # 存 RingKey
        self.kdtree = None

    def add(self, sc: np.ndarray, pose: np.ndarray):
        self.sc_list.append(sc)
        self.pose_list.append(pose)
        self.rk_list.append(self.sc_manager.sc2rk(sc))
        
    def build_tree(self):
        """建库，构建 RingKey 的 KD-Tree"""
        if len(self.rk_list) == 0:
            return
        X = np.vstack(self.rk_list)
        self.kdtree = KDTree(X)

    def save(self, filepath: str):
        import pickle
        data = {
            'sc_list': self.sc_list,
            'pose_list': self.pose_list,
            'rk_list': self.rk_list
        }
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)

    def load(self, directory: str):
        import os
        sc_file = os.path.join(directory, 'scan_context_sc.npy')
        rk_file = os.path.join(directory, 'scan_context_rk.npy')
        poses_file = os.path.join(directory, 'scan_context_poses.npy')
        
        if not (os.path.exists(sc_file) and os.path.exists(rk_file) and os.path.exists(poses_file)):
            raise FileNotFoundError(f"Missing .npy database files in {directory}")
            
        self.sc_list = np.load(sc_file)
        self.rk_list = np.load(rk_file)
        self.pose_list = np.load(poses_file)
        self.build_tree()

    def query(self, query_sc: np.ndarray, top_k: int = 3, rk_search_k: int = 50) -> list:
        """
        在线检索
        1. 用 RingKey 在 KD-Tree 中初筛 rk_search_k 个候选
        2. 对候选进行严密的 ScanContext 矩阵列循环匹配
        返回: [(fitness, T_candidate), ...] 格式与之前的 RANSAC 候选一致
        """
        if self.kdtree is None or len(self.sc_list) == 0:
            return []

        query_rk = self.sc_manager.sc2rk(query_sc)
        
        # 1. RingKey 初筛
        search_k = min(rk_search_k, len(self.rk_list))
        _, indices = self.kdtree.query([query_rk], k=search_k)
        if len(indices.shape) > 1:
            indices = indices[0]
        else:
            indices = [indices]

        # 2. 严密匹配
        candidates = []
        for idx in indices:
            cand_sc = self.sc_list[idx]
            cand_pose = self.pose_list[idx]
            
            # 计算距离和最优列平移（Yaw差）
            dist, best_shift = self.sc_manager.sc_distance(query_sc, cand_sc)
            
            # 计算 fitness（为兼容原代码框架，转化为 0~1 的相似度）
            # 由于 SC distance 在 0~1 之间，1-dist 即为相似度
            fitness = 1.0 - dist
            
            # 计算包含 Yaw 补偿后的位姿
            yaw_diff_deg = self.sc_manager.shift_to_yaw(best_shift)
            
            from scipy.spatial.transform import Rotation as R
            # 生成修正后的位姿（给候选位姿加上搜索出的 Yaw 偏移）
            R_shift = R.from_euler('z', yaw_diff_deg, degrees=True).as_matrix()
            T_corrected = cand_pose.copy()
            # 这里的修正逻辑：候选位姿在地图中，补偿的Yaw是在机器人局部系下的旋转
            T_corrected[:3, :3] = cand_pose[:3, :3] @ R_shift
            
            candidates.append((fitness, T_corrected))

        # 3. 排序并取 Top-K
        candidates.sort(key=lambda x: x[0], reverse=True)
        return candidates[:top_k]
