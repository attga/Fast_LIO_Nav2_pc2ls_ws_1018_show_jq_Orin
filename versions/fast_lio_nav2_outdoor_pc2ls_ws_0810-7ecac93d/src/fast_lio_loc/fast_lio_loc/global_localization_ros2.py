#!/usr/bin/env python3
# coding=utf8
"""
FAST_LIO_LOCALIZATION - ROS2 适配版
原始项目: https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION
改写: 适配 ROS2 Humble / Python3 / open3d >= 0.13

工作原理:
  1. 加载先验 PCD 全局地图
  2. 订阅 FAST-LIO2 的 /cloud_registered (已转换到 odom 系的当前帧点云)
  3. 订阅 /Odometry (FAST-LIO2 输出的里程计)
  4. 等待 /initialpose 消息确定搜索初始值（或默认 identity）
  5. 多尺度 ICP：粗配准(scale=5) → 精配准(scale=1)
  6. 若配准 fitness > 阈值，更新 T_map_to_odom 并广播 TF: map -> odom
  7. 按固定频率持续执行全局定位
"""

from __future__ import annotations
import copy
import json
import logging
import os
import threading
import time

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from scan_context import ScanContextManager, SCDatabase
from tf2_ros import TransformBroadcaster


# ─────────────────────── 参数 ───────────────────────
MAP_VOXEL_SIZE    = 0.4   # 地图体素降采样尺寸 (m)
SCAN_VOXEL_SIZE   = 0.1    # 当前帧体素降采样尺寸 (m)
FREQ_LOCALIZATION = 0.5    # 全局定位频率 (Hz)
LOCALIZATION_TH   = 0.65   # ICP fitness 阈值（持续定位用，锁定后每帧校正）
# 初始全局定位（ScanContext → ICP）专用阈值，比持续定位阈值严格得多。
# 正确定位 fitness ≈ 0.997~1.000，错误落点 ≈ 0.77~0.87，0.95 可精准过滤假阳性。
INIT_LOCALIZATION_TH = 0.70
FOV               = 6.28   # 视野角 (rad)，Livox Mid-360 全向=2π
FOV_FAR           = 80.0   # 最远感知距离 (m)
SC_MIN_FITNESS    = 0.5    # SC 描述符相似度低于此值时候选不可信，ICP 收敛到错误极值风险高

# ── 合理性检查（Sanity Check）参数 ─────────────────────────────────────────────
# ICP 收敛后检查定位结果是否物理合理，拦截"翻转解"等明显异常
SANITY_Z_MAX      = 2.0   # 全局定位 Z 高度绝对值上限 (m)，室内地面机器人通常 |Z| < 0.3m
                           # 超过 0.5m 说明 SC/ICP 收敛到翻转或镜像解，必须拒绝
SANITY_ROLL_MAX   = 30.0   # Roll 绝对值上限 (deg)，正常行驶 Roll/Pitch < 15°
SANITY_PITCH_MAX  = 30.0   # Pitch 绝对值上限 (deg)

# ── IMU 交叉验证参数 ──────────────────────────────────────────────────────────
# 比较 ICP 结果的 Roll/Pitch 与 IMU 实测值，差异过大说明 ICP 收敛到错误姿态
# 比绝对值 Sanity Check 更精准：能捕获"ICP说倾斜20°但IMU测到3°"这类微妙错误
IMU_TOPIC              = '/livox/imu'   # Livox Mid-360 IMU 话题（可按实际修改）
IMU_ROLL_DIFF_MAX      = 15.0           # ICP 与 IMU 的 Roll 差异上限 (deg)
IMU_PITCH_DIFF_MAX     = 15.0           # ICP 与 IMU 的 Pitch 差异上限 (deg)
IMU_DATA_TIMEOUT_SEC   = 2.0            # IMU 数据超时阈值（超时则跳过 IMU 验证）

# ── Yesense AHRS IMU 绝对航向（Yaw）过滤参数 ────────────────────────────────
# Yesense IMU 内置磁力计，融合后输出绝对 Yaw（相对地磁北方向）
# 用于在 ScanContext 候选中过滤 Yaw 差异过大的候选，解决室内对称环境的角落混淆问题
#
# 原理：SE角 vs NW角的 Yaw 差 ≈ 180°，IMU Yaw 可以区分
# 首次成功定位后自动标定 map坐标系 与 磁北方向 的偏移量
YESENSE_IMU_TOPIC      = '/imu/data_raw'  # Yesense IMU 话题（含磁力计融合的绝对 Yaw）
YAW_DIFF_MAX           = 45.0             # SC 候选 Yaw 与 IMU Yaw 的最大允许差 (deg)
                                           # 45° 足够区分 SE/NW 混淆（差180°）
                                           # 室内磁力计误差通常 ±20°，45° 留有足够余量
YESENSE_TIMEOUT_SEC    = 2.0              # Yesense 数据超时阈值（超时则跳过 Yaw 过滤）
YESENSE_CHECK_DELAY_S  = 6.0             # 启动后等待多少秒再检查 Yesense 是否可用
IMU_WAIT_TIMEOUT_S     = 20.0            # Max seconds to wait for first Yesense IMU frame
                                          # before proceeding without Yaw filter
IMU_STABLE_DURATION_S  = 3.0             # 首帧到达后额外等待此时长让磁力计稳定（秒）
                                          # 避免刚上线的不稳定 Yaw 数据误导 SC 候选过滤

# ── Top-K ScanContext 参数 ──────────────────────────────────────────────────────
# 保留 fitness 最高的 K 个 SC 候选结果，各自做 ICP + 完整验证，取最优
# 比单一最优候选更鲁棒：在对称环境中正确解可能不是 SC fitness 最高的
SC_TOP_K               = 3              # 保留前 K 个 SC 候选做 ICP 精配准

# ── 边界与跟踪失败保护参数 ──────────────────────────────────────────────────
YAW_FILTER_FAIL_MAX    = 5              # 连续多少次 SC 候选全部被 Yaw 过滤掉时，触发全局搜索降级
ICP_OUT_OF_MAP_PTS_LIMIT = 30000        # 当视场内地图像素点少于此值时，警告驶出地图边界
ICP_LOW_MAP_PTS_THRESHOLD= 60000        # 接近边界警告阈值
ICP_TRACKING_FAIL_MAX  = 8              # 连续 ICP 校正失败多少次后，强制重新触发全局重定位

# ── 点云质量过滤参数 ────────────────────────────────────────────────────────────
# IMU 未完成初始化时，FAST-LIO 输出的点云帧几乎为空或点数极少（< 50 点）
# 用这类垃圾帧做 SC 检索会给出随机错误结果
SCAN_MIN_POINTS   = 200    # 低于此点数的帧视为 IMU 未就绪的无效帧，直接丢弃

# ── 热启动（Warm Start）参数 ──────────────────────────────────────────────────
WARM_START_FITNESS_TH = 0.90   # 热启动 ICP 专用阈值（比普通阈值 0.75 更严格）
                               # 低于此值 → 机器人可能被搬移 → 回退 ScanContext 全局重定位
POSE_CACHE_MAX_AGE_H  = 24.0  # 缓存过期时间（小时），超过则放弃热启动
POSE_CACHE_PATH_DEFAULT = '/home/nvidia/Fast_LIO_Localization_ws/last_pose_cache.json'


def msg_to_array(pc_msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → Nx3 numpy array"""
    pts = []
    for p in pc2.read_points(pc_msg, skip_nans=True):
        pts.append([p[0], p[1], p[2]])
    if len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return np.array(pts, dtype=np.float32)


def voxel_down_sample(pcd: o3d.geometry.PointCloud, voxel_size: float) -> o3d.geometry.PointCloud:
    return pcd.voxel_down_sample(voxel_size)


def registration_at_scale(pc_scan: o3d.geometry.PointCloud,
                           pc_map:  o3d.geometry.PointCloud,
                           initial: np.ndarray,
                           scale:   float):
    """多尺度 ICP 配准 (Point-to-Plane，对对称场景旋转歧义更鲁棒)

    Point-to-Plane 相比 Point-to-Point 的优势：
      - 利用法线方向约束旋转，在具有轴对称/平移对称结构的室内场景（走廊、矩形房间）
        中可大幅降低 ICP 陷入 180° 错误极值的概率
      - 通常收敛更快、精度更高（尤其对平面占主导的建筑环境）
    """
    scan_down = voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale)
    map_down  = voxel_down_sample(pc_map,  MAP_VOXEL_SIZE  * scale)

    # 估计法线（Point-to-Plane 必须，在降采样后的小点集上速度快）
    normal_radius = max(MAP_VOXEL_SIZE, SCAN_VOXEL_SIZE) * scale * 2.0
    normal_param  = o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30)
    scan_down.estimate_normals(normal_param)
    map_down.estimate_normals(normal_param)

    result = o3d.pipelines.registration.registration_icp(
        scan_down,
        map_down,
        2.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60),
    )
    return result.transformation, result.fitness


def inverse_se3(trans: np.ndarray) -> np.ndarray:
    inv = np.eye(4)
    inv[:3, :3] = trans[:3, :3].T
    inv[:3, 3]  = -trans[:3, :3].T @ trans[:3, 3]
    return inv


def pose_msg_to_mat(pose_msg) -> np.ndarray:
    """PoseWithCovarianceStamped.pose.pose → 4x4 变换矩阵"""
    from scipy.spatial.transform import Rotation
    p = pose_msg.pose.pose.position
    q = pose_msg.pose.pose.orientation
    T = np.eye(4)
    T[:3, 3]  = [p.x, p.y, p.z]
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return T


def odom_msg_to_mat(odom_msg: Odometry) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    p = odom_msg.pose.pose.position
    q = odom_msg.pose.pose.orientation
    T = np.eye(4)
    T[:3, 3]  = [p.x, p.y, p.z]
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return T


class FastLIOLocalizationNode(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')

        # ── 文件日志（最先初始化，后续所有日志均可写入）──────────────────────────
        self._setup_file_logger()

        # 参数
        self.declare_parameter('map_pcd_path', '')
        self.declare_parameter('localization_freq', FREQ_LOCALIZATION)
        self.declare_parameter('fitness_threshold', LOCALIZATION_TH)
        self.declare_parameter('map_voxel_size', MAP_VOXEL_SIZE)
        self.declare_parameter('scan_voxel_size', SCAN_VOXEL_SIZE)
        self.declare_parameter('fov_far', FOV_FAR)
        self.declare_parameter('odom_frame', 'camera_init')  # FAST-LIO2 默认 odom 系
        self.declare_parameter('pose_cache_path', POSE_CACHE_PATH_DEFAULT)
        # 热启动开关：室内对称环境建议关闭（False），室外/非对称环境可开启
        self.declare_parameter('enable_warm_start', False)
        # self.declare_parameter('sc_db_path', '/home/nvidia/Fast_LIO_Nav2_outdoor_pc2ls_ws_0810/maps/LOAM_outdoor/GlobalMap_sc_db.pkl')
        self.declare_parameter('sc_db_path', '/home/nvidia/Fast_LIO_Nav2_outdoor_pc2ls_ws_0810/maps/pkl_gen/sc_database_indoor_multiori.pkl')
        # Yaw 标定偏移手动参数（度）
        # 默认 -9999.0 表示未设置，由系统自动标定
        self.declare_parameter('yaw_map_offset_deg', -9999.0)
        self.declare_parameter('invert_imu_yaw', False)
        # 室外环境需要放宽 Z 轴限制，防止因地形起伏导致正常位姿被拒绝
        self.declare_parameter('sanity_z_max', SANITY_Z_MAX)

        map_pcd_path        = self.get_parameter('map_pcd_path').value
        self.freq           = self.get_parameter('localization_freq').value
        self.fitness_th     = self.get_parameter('fitness_threshold').value
        self.map_voxel      = self.get_parameter('map_voxel_size').value
        self.scan_voxel     = self.get_parameter('scan_voxel_size').value
        self.fov_far        = self.get_parameter('fov_far').value
        self.odom_frame     = self.get_parameter('odom_frame').value
        self.pose_cache_path    = self.get_parameter('pose_cache_path').value
        self.enable_warm_start  = self.get_parameter('enable_warm_start').value
        self.invert_imu_yaw     = self.get_parameter('invert_imu_yaw').value
        self.sanity_z_max       = self.get_parameter('sanity_z_max').value
        self.sc_db_path         = self.get_parameter('sc_db_path').value

        # Initialize ScanContext DB
        self.sc_manager = ScanContextManager(max_radius=FOV_FAR)
        self.sc_db = SCDatabase(self.sc_manager)
        if os.path.exists(self.sc_db_path):
            self.sc_db.load(self.sc_db_path)
            self.get_logger().info(f'✅ ScanContext Database loaded from {self.sc_db_path}')
        else:
            self.get_logger().error(f'❌ ScanContext Database NOT FOUND at {self.sc_db_path}! Please run build_sc_db.py first.')

        # 状态变量
        self.global_map: o3d.geometry.PointCloud | None = None
        self.global_map_down: o3d.geometry.PointCloud | None = None
        self.cur_scan:   o3d.geometry.PointCloud | None = None
        self.cur_odom:   Odometry | None = None
        self.cur_imu:    Imu | None = None          # 最新 IMU 消息（用于交叉验证）
        self._imu_stamp: float = 0.0                # 最新 IMU 消息的时间戳（秒）
        # Yesense AHRS IMU（绝对 Yaw，用于对称角落去歧义）
        self.yesense_imu: Imu | None = None         # 最新 Yesense IMU 消息
        self._yesense_stamp: float = 0.0            # Yesense 最新消息时间戳
        self._yesense_first_stamp: float = 0.0      # Yesense 首帧到达时间（用于稳定性判断）
        self._yaw_map_offset: float | None = None   # map坐标系X轴与磁北方向的偏移(deg)
                                                    # None=未标定，首次成功定位后自动计算
        self.T_map_to_odom = np.eye(4)
        self.initialized = False
        self.localization_locked = False   # True 后停止周期性 ICP，TF 保持固定，消除抖动
        self._lock = threading.Lock()
        self._cached_warm_T: np.ndarray | None = None  # 热启动缓存位姿（仅启动阶段使用）
        self._node_start_time: float = time.time()     # node startup time for IMU wait gate
        # ── 全局搜索降级状态 ──────────────────────────────────────────
        self._yaw_filter_fail_count:   int  = 0     # Yaw 过滤连续全拒帧计数
        self._global_search_mode:      bool = False  # 降级标志：True 时下次 RANSAC 跳过 Yaw 过滤
        self._icp_tracking_fail_count: int  = 0     # 锁定后连续 ICP 校正失败计数

        # TF 广播 (动态 TransformBroadcaster，配合 cb_odom 高频刷新时间戳)
        # 避免 Nav2 tf2_buffer 因时间戳 0 或滞后过大而报 Extrapolation Error
        self.tf_broadcaster = TransformBroadcaster(self)

        # 1. 声明多线程回调组
        # cb_group_fast 供高频实时传感器数据使用 (IMU, Odom)
        # cb_group_slow 供低频耗时计算使用 (RANSAC, 定时器 ICP)
        self.cb_group_fast = MutuallyExclusiveCallbackGroup()
        self.cb_group_slow = MutuallyExclusiveCallbackGroup()

        # 发布者
        self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)

        # 订阅者
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.sub_scan = self.create_subscription(
            PointCloud2, '/cloud_registered',
            self.cb_scan, sensor_qos, callback_group=self.cb_group_slow)
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry',
            self.cb_odom, sensor_qos, callback_group=self.cb_group_fast)
        self.sub_initial = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self.cb_initial_pose, 10, callback_group=self.cb_group_slow)
        # IMU 订阅：用于 ICP 结果的姿态交叉验证（Roll/Pitch）
        self.sub_imu = self.create_subscription(
            Imu, IMU_TOPIC,
            self.cb_imu, sensor_qos, callback_group=self.cb_group_fast)
        # Yesense AHRS IMU 订阅：用于绝对 Yaw 过滤（磁力计融合，区分对称角落）
        self.sub_yesense = self.create_subscription(
            Imu, YESENSE_IMU_TOPIC,
            self.cb_yesense_imu, sensor_qos, callback_group=self.cb_group_fast)
        # 延迟检查 Yesense 是否可用（避免在驱动未启动前误报）
        self._yesense_check_timer = self.create_timer(
            YESENSE_CHECK_DELAY_S, self._check_yesense_availability)

        # 加载地图
        if not map_pcd_path:
            self.get_logger().error('map_pcd_path parameter is empty! Please provide it via --ros-args -p map_pcd_path:=<path>')
            return
        self.get_logger().info(f'Loading map: {map_pcd_path}')
        self._load_global_map(map_pcd_path)

        # 定期全局定位定时器（等初始化完成后才有效）
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.timer_localization, callback_group=self.cb_group_slow)

        # ── 热启动：尝试加载上次关机缓存位姿 ─────────────────────────────────
        if self.enable_warm_start:
            self._cached_warm_T = self._load_pose_cache()
            if self._cached_warm_T is not None:
                self.get_logger().info(
                    '[WarmStart] Found valid cached pose, prioritizing warm start on first frame (skipping RANSAC)')
            else:
                self.get_logger().info(
                    '[WarmStart] No valid cache found, will use RANSAC for global relocalization')
        else:
            self._cached_warm_T = None
            self.get_logger().info(
                '[WarmStart] Warm start disabled (enable_warm_start=false), directly using RANSAC for global relocalization')

        # 加载 Yaw 标定偏移（与 pose cache 共享文件）
        self._load_yaw_offset()

        # 手动参数覆盖（优先级最高）
        _manual_offset = self.get_parameter('yaw_map_offset_deg').value
        if abs(_manual_offset - (-9999.0)) > 0.1:   # 用户确实设置了该参数
            self._yaw_map_offset = float(_manual_offset)
            self.get_logger().info(
                f'[YawCalib] ✅ Manual Yaw offset set: {self._yaw_map_offset:.2f}°\n'
                f'  → Yaw filtering will be active from first cold start, no need to wait for auto-calibration')

        self.get_logger().info('FastLIO Localization node started, waiting for initial pose /initialpose ...')
        self.get_logger().info('  If no /initialpose is sent, ICP will be performed directly with identity as initial value when the first frame arrives.')

    # -- File Logger ----------------------------------------------------------
    def _setup_file_logger(self):
        """Create a timestamped log file on each startup; never overwrites old files.

        Log directory : ~/Fast_LIO_Localization_ws/logs/
        File pattern  : localization_YYYYMMDD_HHMMSS.log
        """
        import datetime
        log_dir = os.path.expanduser(
            '~/Fast_LIO_Localization_ws/logs')
        os.makedirs(log_dir, exist_ok=True)

        ts  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(log_dir, f'localization_{ts}.log')

        self._file_logger = logging.getLogger(f'fastlio_loc_{ts}')
        self._file_logger.setLevel(logging.DEBUG)
        handler = logging.FileHandler(log_path, encoding='utf-8')
        handler.setFormatter(logging.Formatter(
            '[%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'))
        self._file_logger.addHandler(handler)
        self._log_path = log_path

        # Session header
        import datetime as _dt
        self._file_logger.info('=' * 70)
        self._file_logger.info('FastLIO Localization - Session Start')
        self._file_logger.info(f'Time    : {_dt.datetime.now().isoformat()}')
        self._file_logger.info(f'Log file: {log_path}')
        self._file_logger.info('=' * 70)

        self.get_logger().info(f'[FileLog] Log saved to: {log_path}')

    def _flog(self, level: str, msg: str, also_ros: bool = False,
              ros_level: str = 'info'):
        """Write to file log (always) and optionally also to the ROS2 terminal.

        Args:
            level:     File log level: 'info' / 'warning' / 'error' / 'debug'
            msg:       Message string
            also_ros:  If True, also emit via ROS2 logger (default False).
            ros_level: ROS2 log level: 'info' / 'warn' / 'error'
        """
        if not hasattr(self, '_file_logger'):
            return
        getattr(self._file_logger, level)(msg)
        if also_ros:
            getattr(self.get_logger(), ros_level)(msg)

    # ── 地图加载 ──────────────────────────────────
    def _load_global_map(self, path: str):
        pcd = o3d.io.read_point_cloud(path)
        if len(pcd.points) == 0:
            self.get_logger().error(f'Map is empty or path is incorrect: {path}')
            return
        pcd = voxel_down_sample(pcd, self.map_voxel)
        self.global_map = pcd
        self.get_logger().info(f'Map loaded: {len(pcd.points)} points (downsampled)')
        self.get_logger().info('✅ FPFH precomputation removed. Using ScanContext instead.')

    # ── 回调 ──────────────────────────────────────
    def cb_odom(self, msg: Odometry):
        with self._lock:
            self.cur_odom = msg

        # 每次收到里程计时以里程计频率高频广播动态 TF，防止 Nav2 报 Extrapolation Error
        if self.initialized:
            self._broadcast_map_to_odom(self.T_map_to_odom)

    def cb_imu(self, msg: Imu):
        """接收 IMU 消息，缓存最新的姿态数据（用于 ICP 结果交叉验证）。"""
        with self._lock:
            self.cur_imu = msg
            self._imu_stamp = time.time()

    def cb_yesense_imu(self, msg: Imu):
        """接收 Yesense AHRS IMU 消息（含磁力计融合的绝对 Yaw）。"""
        with self._lock:
            self.yesense_imu = msg
            now = time.time()
            self._yesense_stamp = now
            if self._yesense_first_stamp == 0.0:  # 记录首帧到达时间（仅一次）
                self._yesense_first_stamp = now

    def _check_yesense_availability(self):
        """启动后延迟检查 Yesense IMU 是否正常发布数据（只运行一次）。"""
        self._yesense_check_timer.cancel()  # 只触发一次
        age = time.time() - self._yesense_stamp
        if self._yesense_stamp == 0.0 or age > YESENSE_CHECK_DELAY_S:
            self.get_logger().warn(
                f'[YesenseYaw] ⚠️  No {YESENSE_IMU_TOPIC} data received!'
                f'(Still no data after waiting {YESENSE_CHECK_DELAY_S:.0f}s)\n'
                f'  → Symmetric corner disambiguation disabled, RANSAC candidates will not be Yaw filtered\n'
                f'  → Please check if yesense_std_ros2 node is running')
        else:
            imu_yaw = self._get_imu_yaw()
            yaw_str = f"{imu_yaw:.1f}°" if imu_yaw is not None else "超时/未知"
            self.get_logger().info(
                f'[YesenseYaw] ✅ Yesense IMU data is normal'
                f'(Latest data {age:.1f}s ago, current Yaw={yaw_str})\n'
                f'  → RANSAC candidates will be filtered by Yaw to solve symmetric corner confusion\n'
                + (f'  → Yaw calibration offset loaded: {self._yaw_map_offset:.1f}°'
                   if self._yaw_map_offset is not None
                   else '  → Yaw calibration offset unknown, will auto-calibrate after first successful localization'))

    @staticmethod
    def _normalize_yaw(angle: float) -> float:
        """归一化角度到 [-180, 180] 度"""
        while angle > 180.0:  angle -= 360.0
        while angle < -180.0: angle += 360.0
        return angle

    def _get_imu_yaw(self) -> 'float | None':
        """从最新 Yesense IMU 消息中提取绝对 Yaw（度），超时返回 None。"""
        from scipy.spatial.transform import Rotation as R
        with self._lock:
            msg   = self.yesense_imu
            age   = time.time() - self._yesense_stamp
        if msg is None or age > YESENSE_TIMEOUT_SEC:
            return None
        q = msg.orientation
        rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
        raw_yaw = rpy[2]
        if self.invert_imu_yaw:
            raw_yaw = -raw_yaw
        return self._normalize_yaw(raw_yaw)

    @staticmethod
    def _yaw_from_matrix(T: np.ndarray) -> float:
        """从 4x4 变换矩阵中提取 Yaw 角（度）。"""
        from scipy.spatial.transform import Rotation as R
        return R.from_matrix(T[:3, :3].copy()).as_euler('xyz', degrees=True)[2]

    def _filter_candidates_by_yaw(self,
                                   candidates: list,
                                   ) -> list:
        """用 Yesense 绝对 Yaw 过滤 RANSAC 候选，丢弃 Yaw 差异 > YAW_DIFF_MAX 的候选。

        工作条件：
          1. Yesense IMU 数据新鲜（< YESENSE_TIMEOUT_SEC）
          2. Yaw 标定偏移已知（self._yaw_map_offset is not None）
        任一不满足 → 跳过过滤，返回原始列表（优雅降级）

        Args:
          candidates: [(ransac_fitness, T_4x4), ...] 已按 fitness 降序排列
        Returns:
          过滤后的候选列表（可能长度不变）
        """
        imu_yaw = self._get_imu_yaw()
        if imu_yaw is None:
            self.get_logger().debug('[YawFilter] Yesense data unavailable, skip Yaw filter')
            return candidates
        if self._yaw_map_offset is None:
            self.get_logger().debug('[YawFilter] Yaw offset not calibrated yet, skip Yaw filter')
            return candidates

        expected_yaw = self._normalize_yaw(imu_yaw + self._yaw_map_offset)
        # --- DEBUG LOGGING ---
        cand_yaws_str = []
        for fit, T in candidates:
            cand_yaws_str.append(f"{self._yaw_from_matrix(T):.1f}°")
        self.get_logger().info(f"[YAW-DEBUG] Expected: {expected_yaw:.1f}°. Candidates: {', '.join(cand_yaws_str)}")
        self._flog('info', f"[YAW-DEBUG] Expected: {expected_yaw:.1f}°. Candidates: {', '.join(cand_yaws_str)}")
        # ---------------------

        filtered = []
        for fit, T in candidates:
            cand_yaw  = self._yaw_from_matrix(T)
            diff      = abs(self._normalize_yaw(cand_yaw - expected_yaw))
            if diff <= YAW_DIFF_MAX:
                filtered.append((fit, T))
                self.get_logger().info(
                    f'[YawFilter] KEEP   candidate Yaw={cand_yaw:.1f}° | '
                    f'expected={expected_yaw:.1f}° | diff={diff:.1f}°')
            else:
                self.get_logger().warn(
                    f'[YawFilter] REJECT candidate Yaw={cand_yaw:.1f}° | '
                    f'expected={expected_yaw:.1f}° | diff={diff:.1f}° > {YAW_DIFF_MAX}°')

        if not filtered:
            self.get_logger().warn(
                '[YawFilter] ⚠️ All RANSAC candidates rejected by Yaw filter! '
                '(Possible symmetric corner confusion). Waiting for next frame.')
            return []  # 坚决不回退！返回空列表，迫使系统等待下一帧重新采样

        self.get_logger().info(
            f'[YawFilter] {len(candidates)} → {len(filtered)} candidates '
            f'after Yaw filter (IMU Yaw={imu_yaw:.1f}°, offset={self._yaw_map_offset:.1f}°)')
        return filtered

    def _imu_cross_check(self, T_map_to_body: 'np.ndarray') -> 'tuple[bool, str]':
        """IMU 交叉验证：比较 ICP 结果的 Roll/Pitch 与 IMU 实测值。

        与现有 Sanity Check 的分工：
          - Sanity Check（绝对值）：拦截 |Roll|>30° 的明显翻转解，无需 IMU
          - IMU 交叉验证（相对值）：拦截差异>15° 的微妙错误，需要 IMU 话题可用

        Returns:
          (ok: bool, reason: str)  ok=True 表示通过验证
        """
        from scipy.spatial.transform import Rotation as R
        with self._lock:
            imu_msg  = self.cur_imu
            imu_age  = time.time() - self._imu_stamp

        if imu_msg is None or imu_age > IMU_DATA_TIMEOUT_SEC:
            return True, f'[IMUCheck] IMU data not available (age={imu_age:.1f}s), skip'

        # 从 IMU 四元数提取实测 Roll/Pitch
        q = imu_msg.orientation
        rpy_imu = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)
        imu_roll, imu_pitch = rpy_imu[0], rpy_imu[1]

        # 从 ICP 结果提取 Roll/Pitch
        rpy_icp = R.from_matrix(T_map_to_body[:3, :3].copy()).as_euler('xyz', degrees=True)
        icp_roll, icp_pitch = rpy_icp[0], rpy_icp[1]

        roll_diff  = abs(icp_roll  - imu_roll)
        pitch_diff = abs(icp_pitch - imu_pitch)

        self.get_logger().debug(
            f'[IMUCheck] ICP R={icp_roll:.1f}deg P={icp_pitch:.1f}deg | '
            f'IMU R={imu_roll:.1f}deg P={imu_pitch:.1f}deg | '
            f'diff R={roll_diff:.1f}deg P={pitch_diff:.1f}deg')

        if roll_diff > IMU_ROLL_DIFF_MAX:
            return False, (f'[IMUCheck] REJECTED: Roll diff={roll_diff:.1f}deg > '
                           f'{IMU_ROLL_DIFF_MAX}deg (ICP={icp_roll:.1f} vs IMU={imu_roll:.1f})')
        if pitch_diff > IMU_PITCH_DIFF_MAX:
            return False, (f'[IMUCheck] REJECTED: Pitch diff={pitch_diff:.1f}deg > '
                           f'{IMU_PITCH_DIFF_MAX}deg (ICP={icp_pitch:.1f} vs IMU={imu_pitch:.1f})')

        return True, f'[IMUCheck] OK (roll_diff={roll_diff:.1f}deg pitch_diff={pitch_diff:.1f}deg)'


    def cb_scan(self, msg: PointCloud2):
        pts = msg_to_array(msg)
        if pts.shape[0] == 0:
            return

        # ── IMU 就绪检查：过滤稀疏无效帧 ─────────────────────────────────────
        # FAST-LIO 在 IMU 未完成初始化时仍会发布点云，但点数极少（通常 < 50 点）。
        # 若用这类帧触发 RANSAC 全局定位，会得到完全随机的错误结果。
        if pts.shape[0] < SCAN_MIN_POINTS:
            self.get_logger().warn(
                f'[ScanFilter] Point cloud frame too sparse ({pts.shape[0]} points < {SCAN_MIN_POINTS}), '
                f'IMU might not be initialized, skipping this frame',
                throttle_duration_sec=2.0)
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
        with self._lock:
            self.cur_scan = pcd
        # 若还没初始化，执行定位初始化（热启动优先，失败则回退 RANSAC）
        if not self.initialized and self.global_map is not None:

            # ══════════════════════════════════════════════════════════════
            # 阶段 A：热启动（Warm Start）
            #   条件：存在有效缓存位姿
            #   流程：直接用缓存 T 做 ICP → fitness >= 0.90 则锁定
            #         否则说明机器人被搬移 → 清除缓存 → 进入阶段 B
            # ══════════════════════════════════════════════════════════════
            if self._cached_warm_T is not None:
                self.get_logger().info(
                    '[WarmStart] Executing ICP directly using cached pose (skipping RANSAC)...')
                warm_success = self._try_warm_start(pcd, self._cached_warm_T)
                self._cached_warm_T = None  # 无论成败，缓存只尝试一次
                if warm_success:
                    self.initialized = True
                    self.localization_locked = True
                    self.get_logger().info(
                        '[WarmStart] Warm start successful! Localization locked, RANSAC bypassed.\n'
                        '  To relocate, please provide a new initial pose using "2D Pose Estimate" in RViz.')
                    return
                else:
                    self.get_logger().warn(
                        '[WarmStart] Warm start ICP fitness insufficient '
                        f'(Threshold={WARM_START_FITNESS_TH}, robot might have been moved), '
                        'falling back to RANSAC global relocalization...')

            # ══════════════════════════════════════════════════════════════
            # 阶段 B：RANSAC 全局重定位（冷启动 或 热启动失败后回退）
            #   多次独立运行取最佳：N 次尝试成功率 = 1-(1-p)^N
            #   例：单次 p=0.5 → 5次后 P=96.9%
            # ══════════════════════════════════════════════════════════════

            # -- IMU readiness gate -------------------------------------------
            # Problem: RANSAC runs ~2s after node startup, but Yesense IMU data
            # typically arrives 8-16s later. Without IMU, Yaw filter is inactive
            # and RANSAC may converge to a false-positive area (e.g. X≈-55).
            #
            #   - MUST wait for at least one Yesense IMU frame to arrive, 
            #     even if Yaw offset is manually provided (e.g. -130.5).
            #     This guarantees Yaw filter can activate for RANSAC!
            #   - Warm Start bypasses this gate because it doesn't need RANSAC.
            # -----------------------------------------------------------------
            if self._yesense_stamp == 0.0:
                elapsed = time.time() - self._node_start_time
                if elapsed < IMU_WAIT_TIMEOUT_S:
                    self.get_logger().info(
                        f'[IMUGate] Waiting for first Yesense IMU frame before RANSAC '
                        f'({elapsed:.1f}s elapsed / {IMU_WAIT_TIMEOUT_S:.0f}s timeout). '
                        f'Yaw filter must be ready to prevent false-positive localizations.',
                        throttle_duration_sec=3.0)
                    self._flog('info',
                        f'[IMU-GATE] holding RANSAC, waiting for IMU '
                        f'elapsed={elapsed:.1f}s timeout={IMU_WAIT_TIMEOUT_S:.0f}s')
                    return   # skip frame, try again on next scan
                else:
                    # IMU never arrived — degrade gracefully
                    self.get_logger().warn(
                        f'[IMUGate] No Yesense IMU data received after {elapsed:.1f}s. '
                        f'Proceeding with RANSAC in degraded mode (no Yaw filter). '
                        f'Check that yesense_std_ros2 node is running and publishing '
                        f'on {YESENSE_IMU_TOPIC}.')
                    self._flog('warning',
                        f'[IMU-GATE] timeout={elapsed:.1f}s, '
                        f'running RANSAC without Yaw filter (degraded)')

            # ── IMU 稳定性门控：首帧到达后等待磁力计预热 ────────────────────
            # Yesense 磁力计融合在刚上线的前几秒 Yaw 可能不稳定，
            # 过早触发 RANSAC 会让 Yaw 过滤基于不可靠数据。
            if (self._yaw_map_offset is None
                    and self._yesense_first_stamp > 0.0
                    and (time.time() - self._yesense_first_stamp) < IMU_STABLE_DURATION_S):
                stable_so_far = time.time() - self._yesense_first_stamp
                self.get_logger().info(
                    f'[IMUGate] Yesense IMU received but stabilizing '
                    f'({stable_so_far:.1f}s / {IMU_STABLE_DURATION_S:.0f}s), '
                    f'holding RANSAC...',
                    throttle_duration_sec=1.0)
                self._flog('info',
                    f'[IMU-GATE] IMU stabilizing '
                    f'({stable_so_far:.1f}s / {IMU_STABLE_DURATION_S:.0f}s)')
                return  # 等待磁力计预热完成

            self.get_logger().info(
                f'Received point cloud frame, starting ScanContext global relocalization...')

            # ── Top-K ScanContext：保留前 K 个候选，各自做 ICP + 完整验证，取最优 ──
            # ── 全局搜索降级模式：Yaw 过滤连续失败后临时跳过 Yaw 约束 ────────
            _skip_yaw_filter = self._global_search_mode
            if self._global_search_mode:
                self.get_logger().warn(
                    '[GlobalSearchFallback] Global search mode active: '
                    f'Yaw filter=BYPASSED\n'
                    f'  Reason: Yaw filter rejected all candidates {YAW_FILTER_FAIL_MAX} times.')
                self._flog('warning',
                    f'[GLOBAL-SEARCH-FALLBACK] Yaw filter bypassed')

            self.get_logger().info('[ScanContext] Querying database...')
            self._flog('info', '[SC-START] querying ScanContext database')
            
            pts_array = np.asarray(pcd.points)
            query_sc = self.sc_db.sc_manager.ptcloud2sc(pts_array)
            top_k_candidates = self.sc_db.query(query_sc, top_k=SC_TOP_K)
            
            if not top_k_candidates:
                self.get_logger().warn('[ScanContext] Query returned empty results.')
                self._flog('warning', '[SC-FAIL] No candidates found in database.')
                return
                
            self.get_logger().info(
                f'[ScanContext] Top-{len(top_k_candidates)} candidates found: '
                + ', '.join(f'{f:.4f}' for f, _ in top_k_candidates))
            self._flog('info',
                f'[SC-DONE] '
                + ' '.join(f'cand{i+1}={f:.4f}' for i, (f, _) in enumerate(top_k_candidates)))

            # -- Yaw filter: reject candidates whose Yaw deviates from IMU heading --
            if _skip_yaw_filter:
                # 全局搜索降级模式：跳过 Yaw 过滤，直接进入 ICP 验证
                self.get_logger().warn(
                    '[GlobalSearchFallback] Yaw filter BYPASSED -- proceeding directly to ICP validation')
                self._flog('warning',
                    '[GLOBAL-SEARCH-FALLBACK] Yaw filter bypassed, proceeding to ICP')
            else:
                n_before = len(top_k_candidates)
                top_k_candidates = self._filter_candidates_by_yaw(top_k_candidates)
                if len(top_k_candidates) < n_before:
                    self._flog('info',
                        f'[YAW-FILTER] {n_before} -> {len(top_k_candidates)} candidates kept '
                        f'(offset={self._yaw_map_offset})')

                if not top_k_candidates:
                    # 累计连续 Yaw 过滤全拒次数，达阈值后触发全局搜索降级
                    self._yaw_filter_fail_count += 1
                    self.get_logger().warn(
                        f'  Yaw filter rejected all candidates '
                        f'({self._yaw_filter_fail_count}/{YAW_FILTER_FAIL_MAX}). '
                        f'Waiting for next frame...')
                    self._flog('warning', '[YAW-FILTER] All candidates rejected by Yaw filter')
                    if self._yaw_filter_fail_count >= YAW_FILTER_FAIL_MAX:
                        self._global_search_mode    = True
                        self._yaw_filter_fail_count = 0
                        self.get_logger().warn(
                            f'[GlobalSearchFallback] Yaw filter failed '
                            f'{YAW_FILTER_FAIL_MAX} times consecutively!\n'
                            f'  Activating global search mode: Yaw filter BYPASSED.\n'
                            f'  Will retry without Yaw constraint on next frame.\n'
                            f'  Tip: Check yaw_map_offset_deg is correct for this location.')
                        self._flog('warning',
                            f'[GLOBAL-SEARCH-FALLBACK] activated after '
                            f'{YAW_FILTER_FAIL_MAX} consecutive Yaw-filter all-rejected frames')
                    return

            if top_k_candidates[0][0] < SC_MIN_FITNESS:
                self.get_logger().warn(
                    f'[ScanContext] Best fitness={top_k_candidates[0][0]:.4f} too low '
                    f'(< {SC_MIN_FITNESS}). Please use RViz 2D Pose Estimate.')
                self._flog('warning',
                    f'[SC-LOW] best_fitness={top_k_candidates[0][0]:.4f} min={SC_MIN_FITNESS}')

            # Try each Top-K candidate with ICP + full validation
            success = False
            for rank, (sc_fit, sc_T) in enumerate(top_k_candidates, start=1):
                self.get_logger().info(
                    f'  Trying candidate #{rank}/{len(top_k_candidates)} '
                    f'(ScanContext fitness={sc_fit:.4f}) with ICP...')
                self._flog('info', f'[CANDIDATE] trying rank={rank} sc_fitness={sc_fit:.4f}')

                # ── 坐标系修正：SC query 返回 T_map_to_body，
                # 而 _global_localization 期望的是 T_map_to_odom 作为 ICP 初值。
                # 关系：T_map_to_odom = T_map_to_body @ inv(T_odom_to_body)
                # 开机时 odom≈0 两者近似，但移动后重定位必须正确转换，
                # 否则 ICP 初值误差 = 累计里程，导致收敛失败。
                with self._lock:
                    cur_odom_for_sc = self.cur_odom
                if cur_odom_for_sc is not None:
                    T_odom_to_body = odom_msg_to_mat(cur_odom_for_sc)
                    T_map_to_odom_init = sc_T @ inverse_se3(T_odom_to_body)
                else:
                    # 极罕见：odom 还未就绪，此时 body≈odom，直接使用 sc_T 近似
                    T_map_to_odom_init = sc_T
                    self.get_logger().warn(
                        '[ScanContext] cur_odom not ready, using sc_T directly as T_map_to_odom '
                        '(OK only at boot when odom≈0)')

                success = self._global_localization(T_map_to_odom_init)
                if success:
                    self.get_logger().info(
                        f'  Candidate #{rank} accepted after ICP + validation.')
                    self._flog('info', f'[CANDIDATE-OK] rank={rank} accepted')
                    break
                else:
                    self.get_logger().warn(
                        f'  Candidate #{rank} rejected (ICP or validation failed).')
                    self._flog('warning', f'[CANDIDATE-FAIL] rank={rank} rejected')

            if success:
                self.initialized = True
                self.localization_locked = True
                # 成功后重置所有降级状态
                self._global_search_mode      = False
                self._yaw_filter_fail_count   = 0
                self._icp_tracking_fail_count = 0
                self.get_logger().info(
                    '✅ Auto relocalization succeeded! Localization locked.\n'
                    '  Use RViz 2D Pose Estimate to re-localize if needed.')
                self._flog('info', '[LOCKED] Relocalization succeeded - localization locked.')
            else:
                self.get_logger().warn(
                    f'All {len(top_k_candidates)} RANSAC candidates failed ICP/validation. '
                    'Please use RViz 2D Pose Estimate.')
                self._flog('warning',
                    f'[FAILED] All {len(top_k_candidates)} RANSAC candidates failed.')
                # 若在降级模式（跳过 Yaw 过滤）下依然全部失败，重置降级标志等待下轮触发
                if _skip_yaw_filter:
                    self._flog('warning',
                        '[GLOBAL-SEARCH-FALLBACK] Global search also failed. '
                        'Possible causes: robot outside map coverage, '
                        'or yaw_map_offset_deg is significantly wrong.')
                    self._global_search_mode = False  # 重置，等下轮 Yaw 失败再次触发


    def cb_initial_pose(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info('Received /initialpose, executing global relocalization (Point-to-Plane ICP)...')
        # 手动给定初始位姿时，先解锁允许重新定位
        self.localization_locked = False
        initial_T = pose_msg_to_mat(msg)
        success = self._global_localization(initial_T)
        if success:
            self.initialized = True
            self.localization_locked = True   # 🔒 重定位成功后再次锁定
            self.get_logger().info('✅ Relocalization successful! Localization locked.')
        else:
            self.get_logger().warn('❌ Relocalization failed to converge, fitness below threshold, please adjust initial pose orientation and try again')

    # ── 定时全局定位 ──────────────────────────────
    def timer_localization(self):
        # 先验地图始终持续发布（供 RViz 显示白色地图点云）
        if self.global_map is not None:
            self._publish_submap()

        if not self.initialized or self.global_map is None:
            return

        # 持续实时 ICP 校正（Point-to-Plane 模式，对旋转歧义更鲁棒）
        # 每 2s 执行一次 scan-to-map ICP，更新 T_map_to_odom，持续校正里程计漂移
        success = self._global_localization(self.T_map_to_odom)

        # ── 失锁自动恢复：连续 ICP 失败后触发 ScanContext 重定位 ──────────────
        if not success:
            self._icp_tracking_fail_count += 1
            self._flog('warning',
                f'[TRACKING] ICP failed '
                f'({self._icp_tracking_fail_count}/{ICP_TRACKING_FAIL_MAX})')
            if self._icp_tracking_fail_count >= ICP_TRACKING_FAIL_MAX:
                self.get_logger().warn(
                    f'[TrackingFail] ICP failed {ICP_TRACKING_FAIL_MAX} consecutive times!\n'
                    '  → Unlocking localization, next scan will trigger ScanContext re-localization.')
                self._flog('warning',
                    f'[TRACKING-FAIL] {ICP_TRACKING_FAIL_MAX} consecutive failures, '
                    'unlocking for SC re-localization')
                self.initialized    = False   # 解锁，cb_scan 将重新触发 SC 全局重定位
                self.localization_locked = False
                self._icp_tracking_fail_count = 0
        else:
            if self._icp_tracking_fail_count > 0:
                self.get_logger().info(
                    f'[Tracking] ICP recovered after '
                    f'{self._icp_tracking_fail_count} failure(s).')
            self._icp_tracking_fail_count = 0   # 成功则重置计数


    def _publish_submap(self):
        import std_msgs.msg
        import builtin_interfaces.msg
        pts = np.asarray(self.global_map.points)
        header = std_msgs.msg.Header()
        header.stamp = builtin_interfaces.msg.Time(sec=0, nanosec=0)
        header.frame_id = 'map'
        msg = pc2.create_cloud_xyz32(header, pts)
        self.pub_submap.publish(msg)

    # ── 核心：全局 ICP 定位 ───────────────────────
    def _global_localization(self, pose_estimation: np.ndarray) -> bool:
        with self._lock:
            if self.cur_scan is None:
                self.get_logger().warn(
                    '[ICP] cur_scan not ready, skipping localization (waiting for point cloud topic)',
                    throttle_duration_sec=2.0)
                return False
            if self.cur_odom is None:
                self.get_logger().warn(
                    '[ICP] cur_odom not ready, skipping localization (FAST-LIO odometry not initialized, suggest increasing startup delay)',
                    throttle_duration_sec=2.0)
                return False
            scan = copy.copy(self.cur_scan)

        self.get_logger().debug('Executing scan-to-map ICP global localization...')
        tic = time.time()

        # 裁剪 FOV 范围内的地图子集（加速 ICP）
        submap = self._crop_map_in_fov(pose_estimation)
        n_submap = len(submap.points)
        if n_submap < 100:
            msg = f'FOV map points too few ({n_submap} pts), skip localization'
            self.get_logger().warn(f'Too few map points in FOV ({n_submap} points), skipping localization')
            self._flog('warning', f'[ICP-SKIP] {msg}')
            return False

        # ── 地图边界监控：早期预警驶出地图（根据第 15 次室外测试失败根因添加）──
        # fov_map_pts 从 ~112k 骤降至 ~53k 时 ICP 全部失败，本段做趋势预警
        if self.initialized:
            if n_submap < ICP_OUT_OF_MAP_PTS_LIMIT:
                self._flog('warning',
                    f'[MAP-BOUNDARY] CRITICAL: fov_map_pts={n_submap} '
                    f'< {ICP_OUT_OF_MAP_PTS_LIMIT} -- robot likely outside map coverage!')
                self.get_logger().error(
                    f'[MapBoundary] CRITICAL: fov_map_pts={n_submap} '
                    f'(< {ICP_OUT_OF_MAP_PTS_LIMIT}). Robot may be outside map!\n'
                    f'  Consider stopping navigation and returning to mapped area.',
                    throttle_duration_sec=5.0)
            elif n_submap < ICP_LOW_MAP_PTS_THRESHOLD:
                self._flog('warning',
                    f'[MAP-BOUNDARY] WARNING: fov_map_pts={n_submap} '
                    f'< {ICP_LOW_MAP_PTS_THRESHOLD} -- approaching map boundary')
                self.get_logger().warn(
                    f'[MapBoundary] fov_map_pts={n_submap} approaching map boundary '
                    f'(threshold={ICP_LOW_MAP_PTS_THRESHOLD})',
                    throttle_duration_sec=10.0)

        # 多尺度 ICP：粗 → 精
        T_coarse, _ = registration_at_scale(scan, submap, pose_estimation, scale=5)
        T_fine, fitness = registration_at_scale(scan, submap, T_coarse, scale=1)
        toc = time.time()

        # 初始定位阶段（RANSAC 后首次 ICP）使用更严格的阈值，防止错误落点被接受。
        # 定位锁定后恢复普通阈值，确保持续校正不受影响。
        if not self.initialized:
            if self._yaw_map_offset is None:
                # 当 Yaw 过滤未激活时，RANSAC 极易受到对称环境（如 180° 镜像）欺骗。
                # 必须实施极其严格的阈值以确保万无一失。
                active_th = max(INIT_LOCALIZATION_TH, 0.94)
            else:
                # Yaw 过滤已激活，对称解会被角度直接否决，因此可以安全地放宽 ICP 分数要求。
                active_th = INIT_LOCALIZATION_TH
        else:
            active_th = self.fitness_th

        icp_summary = (f'elapsed={toc-tic:.2f}s fitness={fitness:.4f} '
                       f'threshold={active_th} fov_map_pts={n_submap}')
        self.get_logger().info(
            f'ICP elapsed: {toc-tic:.2f}s | fitness={fitness:.4f} | threshold={active_th}')
        if not self.initialized and self._yaw_map_offset is None and fitness < active_th and fitness >= INIT_LOCALIZATION_TH:
            self.get_logger().warn(
                f'Since Yaw filter is inactive, strict threshold ({active_th}) applied, rejecting possible score ({fitness:.4f}). '
                f'If starting in open environment, please provide yaw_map_offset_deg via launch file!')
        
        self._flog('info', f'[ICP] {icp_summary}')

        if fitness >= active_th:
            # ── Sanity Check: reject flipped/mirrored solutions ──────────────
            from scipy.spatial.transform import Rotation as R
            with self._lock:
                cur_odom_snap = self.cur_odom
            T_odom_to_body = pose_msg_to_mat(cur_odom_snap) if cur_odom_snap else np.eye(4)
            T_map_to_body_check = T_fine @ T_odom_to_body
            z_global  = T_map_to_body_check[2, 3]
            rpy_check = R.from_matrix(T_map_to_body_check[:3, :3].copy()).as_euler('xyz', degrees=True)

            if abs(z_global) > self.sanity_z_max:
                msg = (f'[SanityCheck] REJECTED: Z={z_global:.3f}m exceeds {self.sanity_z_max}m '
                       f'(flipped solution suspected), fitness={fitness:.4f}')
                self.get_logger().warn(msg)
                self._flog('warning', msg)
                return False
            if abs(rpy_check[0]) > SANITY_ROLL_MAX:
                msg = (f'[SanityCheck] REJECTED: Roll={rpy_check[0]:.1f}deg exceeds '
                       f'{SANITY_ROLL_MAX}deg (flipped solution suspected), fitness={fitness:.4f}')
                self.get_logger().warn(msg)
                self._flog('warning', msg)
                return False
            if abs(rpy_check[1]) > SANITY_PITCH_MAX:
                msg = (f'[SanityCheck] REJECTED: Pitch={rpy_check[1]:.1f}deg exceeds '
                       f'{SANITY_PITCH_MAX}deg (flipped solution suspected), fitness={fitness:.4f}')
                self.get_logger().warn(msg)
                self._flog('warning', msg)
                return False

            # ── IMU 交叉验证（精细拦截，比绝对值 Sanity Check 更敏感）──────────
            imu_ok, imu_msg_str = self._imu_cross_check(T_map_to_body_check)
            if not imu_ok:
                self.get_logger().warn(f'{imu_msg_str}, fitness={fitness:.4f}')
                self._flog('warning', f'[IMU-CrossCheck] REJECTED: {imu_msg_str}, fitness={fitness:.4f}')
                return False
            self.get_logger().debug(imu_msg_str)

            # Sanity check passed - accept this localization result
            with self._lock:
                self.T_map_to_odom = T_fine
            self._broadcast_map_to_odom(T_fine)

            # ── Save warm-start cache: must store T_map_to_body, NOT T_map_to_odom ──
            # TF chain:
            #   T_map_to_odom:  map -> camera_init (odom origin, resets to 0 on each boot)
            #   T_odom_to_body: camera_init -> body (FAST-LIO odometry)
            #   T_map_to_body = T_map_to_odom @ T_odom_to_body  <- true global position
            #
            # Bug: saving only T_map_to_odom means after reboot, when odom resets to 0,
            # the old T_map_to_odom no longer points to the robot real stop position.
            # Fix: save T_map_to_body = robot pose in map frame at shutdown time.
            if self.enable_warm_start:
                self._save_pose_cache(T_map_to_body_check)

            # ── Yaw 标定：首次成功定位后计算 map坐标系与磁北方向的偏移量 ─────────
            if self._yaw_map_offset is None:
                imu_yaw_now = self._get_imu_yaw()
                if imu_yaw_now is not None:
                    icp_yaw = self._yaw_from_matrix(T_map_to_body_check)
                    self._yaw_map_offset = self._normalize_yaw(icp_yaw - imu_yaw_now)
                    yaw_msg = (f'[YawCalib] Yaw offset calibrated: '
                               f'map_yaw={icp_yaw:.1f}deg imu_yaw={imu_yaw_now:.1f}deg '
                               f'offset={self._yaw_map_offset:.1f}deg')
                    self.get_logger().info(
                        f'[YawCalib] ✅ Yaw offset calibration completed: map_yaw={icp_yaw:.1f}°, '
                        f'imu_yaw={imu_yaw_now:.1f}°, offset={self._yaw_map_offset:.1f}°\n'
                        f'  → Subsequent RANSAC will auto-filter symmetric corner confusion using Yaw')
                    self._flog('info', yaw_msg)
                    self._save_yaw_offset()
                else:
                    warn_msg = ('[YawCalib] First localization succeeded but Yesense IMU unavailable; '
                                'Yaw calibration skipped. Yaw filtering disabled this session.')
                    self.get_logger().warn(
                        '[YawCalib] First localization successful, but Yesense IMU data unavailable, '
                        'cannot complete Yaw calibration. Yaw filtering will be unavailable in this session.')
                    self._flog('warning', warn_msg)

            # --- 打印并记录位姿对比 ---
            t_odom = T_odom_to_body[:3, 3]
            e_odom = R.from_matrix(T_odom_to_body[:3, :3].copy()).as_euler('xyz', degrees=True)
            t_map  = T_map_to_body_check[:3, 3]
            e_map  = rpy_check

            # IMU Yaw（调试用）
            imu_yaw_dbg = self._get_imu_yaw()
            imu_yaw_str = f'{imu_yaw_dbg:.1f}' if imu_yaw_dbg is not None else 'N/A'

            self.get_logger().info(
                f'\n==================== Pose Comparison ====================\n'
                f'[Local Odom] (FAST-LIO, relative to boot origin):\n'
                f'  Pos -> X: {t_odom[0]:8.2f}, Y: {t_odom[1]:8.2f}, Z: {t_odom[2]:8.2f} (m)\n'
                f'  Att -> R: {e_odom[0]:8.2f}, P: {e_odom[1]:8.2f}, Y: {e_odom[2]:8.2f} (deg)\n'
                f'[Global ICP] (relative to map origin):\n'
                f'  Pos -> X: {t_map[0]:8.2f}, Y: {t_map[1]:8.2f}, Z: {t_map[2]:8.2f} (m)\n'
                f'  Att -> R: {e_map[0]:8.2f}, P: {e_map[1]:8.2f}, Y: {e_map[2]:8.2f} (deg)\n'
                f'========================================================='
            )
            # 文件日志：结构化记录，便于后续脚本解析
            self._flog('info',
                f'[POSE] '
                f'local_x={t_odom[0]:.3f} local_y={t_odom[1]:.3f} local_z={t_odom[2]:.3f} '
                f'local_yaw={e_odom[2]:.2f} '
                f'map_x={t_map[0]:.3f} map_y={t_map[1]:.3f} map_z={t_map[2]:.3f} '
                f'map_roll={e_map[0]:.2f} map_pitch={e_map[1]:.2f} map_yaw={e_map[2]:.2f} '
                f'icp_fitness={fitness:.4f} '
                f'imu_yaw={imu_yaw_str} '
                + (f'yaw_offset={self._yaw_map_offset:.2f}'
                   if self._yaw_map_offset is not None else 'yaw_offset=None')
            )
            return True
        else:
            rejected_msg = (f'ICP fitness={fitness:.4f} < threshold {active_th}, '
                            f'localization rejected')
            self.get_logger().warn(rejected_msg)
            self._flog('warning', f'[ICP-REJECT] fitness={fitness:.4f} threshold={active_th}')
            return False


    # ── 热启动专用方法 ─────────────────────────────
    def _try_warm_start(self, pcd: o3d.geometry.PointCloud,
                        cached_T: np.ndarray) -> bool:
        """用缓存位姿直接做 ICP，使用更严格的 fitness 阈值判断是否成功。

        高阈值（0.90）的作用：
          - 机器人原地重启 → 点云与缓存位置高度吻合 → fitness 接近 1.0 → 通过
          - 机器人被搬移   → 点云与缓存位置严重偏离 → fitness 很低    → 拒绝 → 触发 ScanContext 全局重定位

        注意：不再检查 cur_odom 是否就绪。
          - _crop_map_in_fov 在 cur_odom=None 时自动返回全局地图（已有 fallback）
          - _broadcast_map_to_odom 在 cur_odom=None 时使用系统时钟（已有 fallback）
          - 因此热启动不依赖里程计，消除点云帧先于里程计到达时的竞态问题
        """
        self.get_logger().info(
            f'[WarmStart] Cached initial position: X={cached_T[0,3]:.3f}, '
            f'Y={cached_T[1,3]:.3f}, Z={cached_T[2,3]:.3f} m')

        submap = self._crop_map_in_fov(cached_T)
        if len(submap.points) < 100:
            self.get_logger().warn('[WarmStart] Too few map points in FOV, warm start skipped')
            return False

        T_coarse, _ = registration_at_scale(pcd, submap, cached_T, scale=5)
        T_fine, fitness = registration_at_scale(pcd, submap, T_coarse, scale=1)

        self.get_logger().info(
            f'[WarmStart] ICP fitness={fitness:.4f} | Warm start threshold={WARM_START_FITNESS_TH}')

        if fitness >= WARM_START_FITNESS_TH:
            # Sanity check also applies to warm-start results
            # (cached pose itself might have been from a previous bad localization)
            from scipy.spatial.transform import Rotation as R
            z_ws  = abs(T_fine[2, 3])
            rpy_ws = R.from_matrix(T_fine[:3, :3].copy()).as_euler('xyz', degrees=True)
            if z_ws > SANITY_Z_MAX or abs(rpy_ws[0]) > SANITY_ROLL_MAX or abs(rpy_ws[1]) > SANITY_PITCH_MAX:
                self.get_logger().warn(
                    f'[WarmStart][SanityCheck] REJECTED: '
                    f'Z={T_fine[2,3]:.2f}m R={rpy_ws[0]:.1f}deg P={rpy_ws[1]:.1f}deg '
                    f'exceeds limits. Falling back to RANSAC.')
                return False
            # IMU cross-check also applies to warm-start
            imu_ok, imu_msg_str = self._imu_cross_check(T_fine)
            if not imu_ok:
                self.get_logger().warn(f'[WarmStart]{imu_msg_str}, falling back to RANSAC')
                return False
            self.get_logger().debug(f'[WarmStart]{imu_msg_str}')
            with self._lock:
                self.T_map_to_odom = T_fine
            self._broadcast_map_to_odom(T_fine)
            # At warm-start, odom just reset to 0, so body ~ odom origin: T_map_to_body ~ T_fine
            self._save_pose_cache(T_fine)
            return True
        return False

    def _save_yaw_offset(self):
        """yaw_offset 不持久化到文件——每次 session 重新校准，避免错误定位污染下次 session。

        设计决策：
          yaw_offset 是首次成功定位后由 (map_yaw - imu_yaw) 计算的当次标定值。
          若本次定位落在错误位置，该值即为错误偏差，持久化会导致下次 session 继承错误。
          重新校准代价极低（~3s 静止等待），不值得为此冒跨 session 污染的风险。

          若需要固定已知的传感器安装角，请通过 launch 参数 yaw_map_offset_deg 设置，
          该参数优先级最高（见 __init__ 第 294~300 行）。
        """
        if self._yaw_map_offset is not None:
            self.get_logger().debug(
                f'[YawCalib] Yaw offset calibrated (valid only for this session, not persisted): '
                f'{self._yaw_map_offset:.2f}°')

    def _load_yaw_offset(self):
        """yaw_offset 不从缓存加载——每次 session 重新校准（见 _save_yaw_offset 说明）。

        注意：如需在 session 间保留固定安装角，请使用 launch 参数 yaw_map_offset_deg，
        该参数优先级最高（见 __init__ 第 294~300 行）。
        """
        pass  # 有意不读取文件，self._yaw_map_offset 保持 None 直到本 session 首次 YawCalib

    def _load_pose_cache(self) -> 'np.ndarray | None':
        """从 JSON 文件加载上次保存的 T_map_to_odom（4×4 矩阵）。

        返回 None 的情况：
          - 缓存文件不存在（首次运行）
          - 缓存已过期（超过 POSE_CACHE_MAX_AGE_H 小时）
          - 文件损坏或格式错误
        """
        path = self.pose_cache_path
        if not os.path.exists(path):
            self.get_logger().info(f'[WarmStart] Cache file does not exist: {path}')
            return None
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            age_h = (time.time() - data['timestamp']) / 3600.0
            if age_h > POSE_CACHE_MAX_AGE_H:
                self.get_logger().warn(
                    f'[WarmStart] Cache expired (last shutdown {age_h:.1f}h ago > '
                    f'{POSE_CACHE_MAX_AGE_H}h), abandoning warm start')
                return None
            # Support both old field name and new field name
            key = 'T_map_to_odom' if 'T_map_to_odom' in data else 'T_map_to_body'
            T = np.array(data[key], dtype=np.float64)
            if T.shape != (4, 4):
                raise ValueError(f'Matrix shape anomaly: {T.shape}')
            self.get_logger().info(
                f'[WarmStart] Cache loaded successfully (last shutdown {age_h:.2f}h ago)\n'
                f'  Cached position: X={T[0,3]:.3f}, Y={T[1,3]:.3f}, Z={T[2,3]:.3f} m')
            return T
        except Exception as e:
            self.get_logger().warn(f'[WarmStart] Cache load failed ({e}), will use RANSAC')
            return None

    def _save_pose_cache(self, T: np.ndarray):
        """将 T_map_to_odom（4×4 矩阵）序列化为 JSON 写入缓存文件。

        双保险写入策略：
          1. 运行期间每次 ICP 成功均写入（防异常断电）
          2. 正常关机时在 main() finally 块中再写一次（最终位姿）
        """
        path = self.pose_cache_path
        try:
            # Read existing file first to preserve extra fields (e.g. yaw_map_offset)
            data = {}
            tmp_path = path + '.tmp'
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        data = json.load(f)
                except Exception:
                    data = {}   # corrupted file – start fresh
            data['T_map_to_odom'] = T.tolist()
            data['timestamp']     = time.time()
            # Preserve yaw_map_offset from current session if available
            if self._yaw_map_offset is not None:
                data['yaw_map_offset'] = float(self._yaw_map_offset)
            # Atomic write: write to .tmp then rename
            with open(tmp_path, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp_path, path)   # POSIX atomic rename
        except Exception as e:
            self.get_logger().warn(f'[WarmStart] Cache write failed: {e}')

    def _crop_map_in_fov(self, T_map_to_odom: np.ndarray) -> o3d.geometry.PointCloud:
        """从全局地图中裁剪当前 FOV 范围内的点云子集"""
        with self._lock:
            if self.cur_odom is None:
                return self.global_map
            T_odom_to_base = odom_msg_to_mat(self.cur_odom)

        T_map_to_base = T_map_to_odom @ T_odom_to_base
        T_base_to_map = inverse_se3(T_map_to_base)

        map_pts = np.asarray(self.global_map.points)
        map_pts_h = np.hstack([map_pts, np.ones((len(map_pts), 1))])
        pts_in_base = (T_base_to_map @ map_pts_h.T).T

        # 距离过滤
        dist = np.linalg.norm(pts_in_base[:, :3], axis=1)
        idx = dist < self.fov_far

        submap = o3d.geometry.PointCloud()
        submap.points = o3d.utility.Vector3dVector(map_pts[idx])
        return submap

    # ── TF 广播 ───────────────────────────────────
    def _broadcast_map_to_odom(self, T: np.ndarray):
        from scipy.spatial.transform import Rotation
        t = TransformStamped()
        with self._lock:
            if self.cur_odom is not None:
                t.header.stamp = self.cur_odom.header.stamp
            else:
                t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = self.odom_frame   # FAST-LIO2 的 odom 系
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        q = Rotation.from_matrix(np.copy(T[:3, :3])).as_quat()  # [x,y,z,w]
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalizationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 关机时最后一次写入缓存（双保险之二：正常关机的最终位姿）
        if node.initialized:
            if node.cur_odom is not None:
                T_odom_to_body = pose_msg_to_mat(node.cur_odom)
                T_map_to_body = node.T_map_to_odom @ T_odom_to_body
                node._save_pose_cache(T_map_to_body)
            else:
                # 若完全没有里程计，退化保存 T_map_to_odom
                node._save_pose_cache(node.T_map_to_odom)
            node.get_logger().info(
                f'[WarmStart] Shutdown cache written: {node.pose_cache_path}')
        node.destroy_node()
        # ── 修复 Ctrl+C 双重关闭报错 ───────────────────────────────────────
        # 原因：ROS2 的 SIGINT 信号处理器（C++ 层）会提前调用 rcl_shutdown()，
        #       Python 的 finally 块再调用 rclpy.shutdown() 时会重复关闭，
        #       抛出 RCLError: rcl_shutdown already called。
        #       用 try-except 静默忽略这个预期中的重复关闭异常即可。
        try:
            rclpy.shutdown()
        except Exception:
            pass



if __name__ == '__main__':
    main()
