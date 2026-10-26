# 定位参数索引

模块参数归各自包管理；bringup 仅保存双 EKF 与 IMU 集成配置。下表路径相对于工作区 src。

全部由 `localization.launch.py` 加载；节点名和命名空间必须匹配 YAML 的节点键。

| 文件 | 配置内容 |
|---|---|
| `fast_lio/config/mid360.yaml` | MID360 前端、固定外参和点云过滤 |
| `ml_bringup/config/ekf/local_ekf.yaml` | Local：FAST-LIO 绝对位姿 + Yesense 角速度，30 Hz、二维 |
| `ml_bringup/config/ekf/global_ekf.yaml` | Global：NDT 位姿 + Local x/y/yaw 位姿差分 + Yesense 角速度，30 Hz、二维、2 秒迟到历史 |
| `ml_bringup/config/ekf/imu.yaml` | 两层 EKF 共用的 Yesense 角速度叠加配置（加速度关闭） |
| `ml_bringup/config/ekf/imu_measurements.yaml` | 原始 IMU 校验及未知协方差填值 |
| `ml_ndt_localization/config/localization.yaml` | NDT、子图窗口、质量门限、LOST 策略 |
| `ml_localization_manager/config/manager.yaml` | 单次启动 SC 初始化、set_pose 和交接确认 |
| `ml_relocalization/config/relocalization.yaml` | SC 候选搜索、NDT 验证及连续结果确认 |

SC 配置仅由重定位包维护；独立重定位入口与正式定位入口加载同一份 YAML。
`use_sim_time`、地图路径、安装 TF、是否启用 Yesense、日志目录和 RViz 由 launch 参数决定。

## 当前约定

Local 不接 NDT/Global 位姿；融合 FAST-LIO 的 odom 系绝对位姿（`odom0_differential=false`）。Global 融合 Local x/y/yaw 位姿差分、NDT 绝对位姿以及直接 Yesense 角速度，仍依赖 Local TF 计算 map→odom。
定位双 EKF 不再订阅 `/wheel_odom`；建图轮速因子与安装 TF 不受此改动影响。
两层 EKF 均以 `two_d_mode=true` 约束 z/roll/pitch；IMU 输入选择 wx/wy/wz，二维约束下主要有效量为 wz；不融合 IMU 姿态、磁航向或加速度。
Global 的 Local 差分输入已经包含 Yesense 的影响，与直接 IMU 输入相关。当前 robot_localization 配置没有显式表达跨传感器相关项，不能把它们当成两份独立证据或据此宣称协方差更准确。

Yesense 原始加速度含重力。适配节点不去重力，EKF 设置 `imu0_remove_gravitational_acceleration=true`，该设置仅在启用加速度融合时生效；当前不融合 IMU 姿态或加速度。全零未知协方差填为角速度方差 0.0025、加速度方差 1.0；尚未实车标定。
过程噪声与初始协方差统一使用完整 225 项矩阵格式，兼容 Humble/Jazzy；保留既有对角线权重，不使用 15 项简写。

NDT当前YAML使用300 m × 300 m XY窗口，距任一边界不大于100 m时更新。整图常驻，窗口切换时建立 NDT 体素分布，后续帧复用。默认 2 Hz、2 线程、scan/map 体素 0.4 m、NDT 分辨率 2 m、NVTL 下限 2.3；LOST 后仍尝试新扫描。

FAST-LIO 固定外参 `T_IMU_Lidar` 的平移为 [-0.011,-0.02329,0.04412] m，旋转为单位阵；在线外参估计关闭。保留 point_filter_num=1 / max_iteration=5，未宣称最优。本移植版 `preprocess.blind=0.5` 是水平半径平方且仅作用于 z<2 m，约等于 0.707 m 水平近场掩码，不能直接按上游径向盲区理解。

正式入口将 Local EKF 输出重映射为 `/local_odometry/filter`，Global EKF 输出为 `/global_odometry/filter`。这些是 topic 名称；TF 仍由两节点分别发布 odom→base_link 和 map→odom。

## 参数单位与覆盖

NDT 的 `position_stddev` / `orientation_stddev` 为标准差（m / rad），发布前平方。NDT 的 NVTL 越大越好，不是距离误差；SC 重定位也使用 NDT NVTL，另有覆盖率与修正量门控，不再使用 ICP fitness。

`submap_halfwidth` 仅由 SC 使用，是关键帧索引半宽，不是米；NDT 包已移除该参数。300 m窗口 / 100 m边界裕量对应相对中心沿任一XY轴移动达到50 m时更新。
`max_consecutive_failures=40`设置连续拒绝升级LOST的门限；Manager当前startup_only=true，启动后LOST不会自动重新初始化。

正式 launch 强制覆盖：FAST-LIO 关闭 TF 并开启 body scan；NDT 等待初始化；SC 不自动搜索、等待 Manager trigger。仅修改这些 YAML 字段不会改变对应的强制覆盖行为。建图入口独立使用同一 FAST-LIO YAML 与 sc_pgo/config/mapping.yaml，不加载定位融合参数。

Local 融合 LiDAR 绝对位姿和 Yesense 角速度；Global 融合 Local x/y/yaw 位姿差分 + NDT + Yesense 角速度。验收范围见 docs/KNOWN_ISSUES.md。

## 正式入口的配置加载顺序

`ml_bringup/launch/localization.launch.py` 按下面顺序组装参数；同名参数由后项覆盖。

| 节点 | 加载顺序 |
|---|---|
| FAST-LIO | `fastlio_config` → `use_sim_time` → 发布开关、日志目录等集成覆盖 |
| Local EKF | `local_ekf.yaml` → 启用 Yesense 时叠加 `imu.yaml` → `use_sim_time` |
| Global EKF | `global_ekf.yaml` → 启用 Yesense 时叠加 `imu.yaml` → `use_sim_time` |
| IMU 适配 | `imu_measurements.yaml` → `use_sim_time`；仅启用 Yesense 时启动 |
| NDT | `ndt_config` → `map_directory` → `use_sim_time` → 等待初始化 |
| SC 重定位 | `relocalization_config` → `map_directory` / `pose_file` → `use_sim_time` → 关闭自动搜索 |
| Manager | `manager_config` → `use_sim_time` |

`enable_yesense` 控制 IMU 适配、安装 TF 和两层 EKF 的 IMU 配置叠加，不启动 Yesense 硬件驱动。
各 YAML 中未显式列出的参数仍使用节点代码默认值；正式 YAML 中的现场配置值不必等于代码默认值。
目前保留参数名及默认值，不将标准差参数改名为方差，建图参数以 sc_pgo/config/mapping.yaml 为准。

机器人安装参数位于 `config/robot/rear_axle.yaml`；URDF 位于 `urdf/robot.urdf.xacro`，说明见 `urdf/README.md`。未知平移暂取零近似，已知 Livox +90° 保留；正式定位默认使用新车后轴中心配置；旧车配置已删除，历史 bag 需另行提供正确的安装配置。

建图统一使用 `mapping.launch.py`：实机显式设置 `start_livox:=true use_sim_time:=false`；回放设置 `start_livox:=false use_sim_time:=true`。旧 `mapping_mid360s*` 转发入口已移除。

## Manager 临时启动延迟

`localization.launch.py` 默认 `manager_start_delay:=10.0`，仅 Manager 延迟 10 个实际秒数启动，其他节点照常启动；不依赖 `/clock`，回放暂停也会到期。可设 `manager_start_delay:=0` 立即启动，或按地图加载时间增加延迟。参数须为非负有限数。

这是重定位服务就绪竞态的临时绕过措施，不是服务就绪保证。Manager 在服务未就绪时提前消耗 120 秒请求间隔的问题仍待正式修复；本次未修改 Manager 状态机。历史 delivery 快照不随源码修改自动更新。
