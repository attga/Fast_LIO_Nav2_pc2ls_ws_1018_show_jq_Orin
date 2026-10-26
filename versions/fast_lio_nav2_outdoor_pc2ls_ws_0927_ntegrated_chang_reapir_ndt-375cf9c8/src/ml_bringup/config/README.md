# 定位参数索引

模块参数归各自包管理；bringup 仅保存双 EKF 与 IMU 集成配置。下表路径相对于工作区 src。

全部由 `localization.launch.py` 加载；节点名和命名空间必须匹配 YAML 的节点键。

| 文件 | 配置内容 |
|---|---|
| `fast_lio/config/mid360.yaml` | MID360 前端、固定外参和点云过滤 |
| `ml_bringup/config/ekf/local_ekf.yaml` | Local：FAST-LIO 位姿 + wheel vx/wz，30 Hz、二维 |
| `ml_bringup/config/ekf/global_ekf.yaml` | Global：NDT 位姿 + Local x/y/yaw 位姿差分，30 Hz、二维、2 秒迟到历史 |
| `ml_bringup/config/ekf/imu.yaml` | 仅 Local EKF 使用的 Yesense 角速度叠加配置（加速度关闭） |
| `ml_bringup/config/ekf/imu_measurements.yaml` | 原始 IMU 校验及未知协方差填值 |
| `ml_ndt_localization/config/localization.yaml` | NDT、子图窗口、质量门限、LOST 策略 |
| `ml_localization_manager/config/manager.yaml` | 单次启动 SC 初始化、set_pose 和交接确认 |
| `ml_relocalization/config/relocalization.yaml` | SC 候选搜索、NDT 验证及连续结果确认 |

SC 配置仅由重定位包维护；独立重定位入口与正式定位入口加载同一份 YAML。
`use_sim_time`、地图路径、安装 TF、是否启用 Yesense、日志目录和 RViz 由 launch 参数决定。

## 当前约定

Local 不接 NDT/Global 位姿；Global 融合 Local x/y/yaw 位姿差分，不再直接融合 FAST-LIO、wheel 或 IMU。Global 仍依赖 Local TF 计算 map→odom。
Wheel 仅融合有效的 `twist.linear.x` 和 `twist.angular.z`，不融合积分位姿，`differential=false`。
两套 EKF 均以 `two_d_mode=true` 约束 z/roll/pitch；Local 的 IMU 输入仅选择 wx/wy/wz，二维约束下主要有效量为 wz；独立 IMU 加速度暂不融合。

Yesense 原始加速度含重力。适配节点不去重力，EKF 设置 `imu0_remove_gravitational_acceleration=true`，该设置仅在启用加速度融合时生效；当前不融合 IMU 姿态或加速度。全零未知协方差填为角速度方差 0.0025、加速度方差 1.0；尚未实车标定。
过程噪声与初始协方差统一使用完整 225 项矩阵格式，兼容 Humble/Jazzy；保留既有对角线权重，不使用 15 项简写。

NDT 使用 200 m × 200 m XY 窗口，距任一边界不大于 80 m 时更新。整图常驻，窗口切换时建立 NDT 体素分布，后续帧复用。默认 2 Hz、2 线程、scan/map 体素 0.4 m、NDT 分辨率 2 m、NVTL 下限 2.3；LOST 后仍尝试新扫描。

FAST-LIO 固定外参 `T_IMU_Lidar` 的平移为 [-0.011,-0.02329,0.04412] m，旋转为单位阵；在线外参估计关闭。保留 point_filter_num=1 / max_iteration=5，未宣称最优。本移植版 `preprocess.blind=0.5` 是水平半径平方且仅作用于 z<2 m，约等于 0.707 m 水平近场掩码，不能直接按上游径向盲区理解。

正式入口将 Local EKF 输出重映射为 `/local_odometry/filter`，Global EKF 输出为 `/global_odometry/filter`。这些是 topic 名称；TF 仍由两节点分别发布 odom→base_link 和 map→odom。

## 参数单位与覆盖

NDT 的 `position_stddev` / `orientation_stddev` 为标准差（m / rad），发布前平方。NDT 的 NVTL 越大越好，不是距离误差；SC 重定位也使用 NDT NVTL，另有覆盖率与修正量门控，不再使用 ICP fitness。

`submap_halfwidth` 仅由 SC 使用，是关键帧索引半宽，不是米；NDT 包已移除该参数。200 m 窗口 / 80 m 边界裕量对应相对中心沿任一 XY 轴移动达到 20 m 时更新。
`failure_count_before_relocalization` 只设置连续拒绝次数门限，不负责触发 SC。

正式 launch 强制覆盖：FAST-LIO 关闭 TF 并开启 body scan；NDT 等待初始化；SC 不自动搜索、等待 Manager trigger。仅修改这些 YAML 字段不会改变对应的强制覆盖行为。历史实车建图入口仍有内嵌专用参数，不随此配置统一调整。

Local 融合 LiDAR 位姿、wheel vx/wz 和 Yesense 角速度；Global 融合 Local x/y/yaw 位姿差分 + NDT。验收范围见 docs/KNOWN_ISSUES.md。

## 正式入口的配置加载顺序

`ml_bringup/launch/localization.launch.py` 按下面顺序组装参数；同名参数由后项覆盖。

| 节点 | 加载顺序 |
|---|---|
| FAST-LIO | `fastlio_config` → `use_sim_time` → 发布开关、日志目录等集成覆盖 |
| Local EKF | `local_ekf.yaml` → 启用 Yesense 时叠加 `imu.yaml` → `use_sim_time` |
| Global EKF | `global_ekf.yaml` → `use_sim_time` |
| IMU 适配 | `imu_measurements.yaml` → `use_sim_time`；仅启用 Yesense 时启动 |
| NDT | `ndt_config` → `map_directory` → `use_sim_time` → 等待初始化 |
| SC 重定位 | `relocalization_config` → `map_directory` / `pose_file` → `use_sim_time` → 关闭自动搜索 |
| Manager | `manager_config` → `use_sim_time` |

`enable_yesense` 控制 IMU 适配、安装 TF 和 Local 的 IMU 配置叠加，不启动 Yesense 硬件驱动。
各 YAML 中未显式列出的参数仍使用节点代码默认值；正式 YAML 中的现场配置值不必等于代码默认值。
目前保留参数名及默认值，不将标准差参数改名为方差，也不修改历史建图入口的算法参数。

机器人安装参数位于 `config/robot/rear_axle.yaml`；URDF 位于 `urdf/robot.urdf.xacro`，说明见 `urdf/README.md`。未知平移暂取零近似，已知 Livox +90° 保留；正式定位默认使用新车后轴中心配置；旧车配置已删除，历史 bag 需另行提供正确的安装配置。


## 集成入口

只使用 NDT。pose_file 仅传给 SC 重定位，不传给持续 NDT 节点。
安装参数位于 robot/rear_axle.yaml，lidar_mount_yaw 可覆盖 Livox 安装 yaw。
