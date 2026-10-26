# 定位参数索引

模块参数归各自包管理；bringup 仅保存双 EKF 与 IMU 集成配置。下表路径相对于工作区 src。

全部由 `localization.launch.py` 加载；节点名和命名空间必须匹配 YAML 的节点键。

| 文件 | 配置内容 |
|---|---|
| `fast_lio/config/mid360.yaml` | MID360 前端、固定外参和点云过滤 |
| `ml_bringup/config/ekf/local_ekf.yaml` | Local：FAST-LIO 位姿 + wheel vx/wz，30 Hz、二维 |
| `ml_bringup/config/ekf/global_ekf.yaml` | Global：ICP 位姿 + Local x/y/yaw 位姿差分，30 Hz、二维、2 秒迟到历史 |
| `ml_bringup/config/ekf/imu.yaml` | 仅 Local EKF 使用的 Yesense 角速度叠加配置（加速度关闭） |
| `ml_bringup/config/ekf/imu_measurements.yaml` | 原始 IMU 校验及未知协方差填值 |
| `ml_icp_localization/config/localization.yaml` | FastGICP、子图窗口、质量门限、LOST 策略 |
| `ml_localization_manager/config/manager.yaml` | 单次启动 SC 初始化、set_pose 和交接确认 |

SC 参数单独保存在 `ml_relocalization/config/relocalization.yaml`，避免维护两份。
`use_sim_time`、地图路径、安装 TF、是否启用 Yesense、日志目录和 RViz 由 launch 参数决定。

## 当前约定

Local 不接 ICP/Global 位姿；Global 融合 Local x/y/yaw 位姿差分，不再直接融合 FAST-LIO、wheel 或 IMU。Global 仍依赖 Local TF 计算 map→odom。
Wheel 仅融合有效的 `twist.linear.x` 和 `twist.angular.z`，不融合积分位姿，`differential=false`。
两套 EKF 均以 `two_d_mode=true` 约束 z/roll/pitch；共享 IMU 仅选择 wx/wy/wz，二维约束下主要有效量为 wz；独立 IMU 加速度暂不融合。

Yesense 原始加速度含重力。适配节点不去重力，EKF 设置 `imu0_remove_gravitational_acceleration=true`，该设置仅在启用加速度融合时生效；当前不融合 IMU 姿态或加速度。全零未知协方差填为角速度方差 0.0025、加速度方差 1.0；尚未实车标定。
本机 robot_localization 3.8.3 支持协方差的 15 项对角线简写；移植其他版本需检查支持情况，必要时等价展开为 225 项。

ICP 使用 200 m × 200 m 的 XY 窗口，距离边界小于 80 m 时更新，整图常驻并预计算 FastGICP 目标协方差。保持 2 Hz、2 个配准线程、0.4 m scan 体素、3 m 对应距离、0.3 fitness 门限；LOST 后继续尝试新扫描。

FAST-LIO 固定外参 `T_IMU_Lidar` 的平移为 [-0.011,-0.02329,0.04412] m，旋转为单位阵；在线外参估计关闭。保留 point_filter_num=1 / max_iteration=5，未宣称最优。本移植版 `preprocess.blind=0.5` 是水平半径平方且仅作用于 z<2 m，约等于 0.707 m 水平近场掩码，不能直接按上游径向盲区理解。

正式入口将 Local EKF 输出重映射为 `/local_odometry/filter`，Global EKF 输出为 `/global_odometry/filter`。这些是 topic 名称；TF 仍由两节点分别发布 odom→base_link 和 map→odom。

## 参数单位与覆盖

ICP `pose_covariance` / `orientation_covariance` 沿用历史名称，实际填标准差（m / rad），发布前平方为方差；四个相关参数现已显式写出，数值与原 C++ 默认值一致。
ICP 和 SC 的 fitness 均为最近邻距离平方均值（m²），但持续 ICP 限制统计距离为 3 m，SC 使用无距离上限评分，不能直接横向比较相同门限。

`submap_halfwidth` 是关键帧索引半宽，不是米；持续 ICP 的 window 模式忽略它，SC 仍使用它。200 m 窗口 / 80 m 边界裕量对应相对中心沿任一 XY 轴移动达到 20 m 时更新。
`failure_count_before_relocalization` 只设置连续拒绝次数门限，不负责触发 SC。

正式 launch 强制覆盖：FAST-LIO 关闭 TF 并开启 body scan；ICP 等待初始化；SC 不自动搜索、等待 Manager trigger。仅修改这些 YAML 字段不会改变对应的强制覆盖行为。历史实车建图入口仍有内嵌专用参数，不随此配置统一调整。

Local 融合 LiDAR 位姿、wheel vx/wz 和 Yesense 角速度；Global 融合 Local x/y/yaw 位姿差分 + ICP。验收范围见 docs/KNOWN_ISSUES.md。
