> 当前目录是 2026-09-23 的 0915 + qq 迁入候选版。已完成离线检查，并修复 FAST-LIO 日志数组越界；完整 ROS 编译、bag 回放和实车验收尚未完成。下文的 0915 历史测试结论不代表本候选版的验收结果。

# MID360 园区定位

供同事接手和继续实测的工程基线。当前验证环境为 Ubuntu 24.04 / ROS 2 Jazzy / x86_64；Humble 和 Jetson Orin NX 尚未验收。

正式定位入口：`ros2 launch ml_bringup localization.launch.py map_directory:=<地图绝对路径>`。
本机完整多终端命令见 [RUN_COMMANDS.txt](RUN_COMMANDS.txt)。bag 回放必须加 `use_sim_time:=true` 并发布 `/clock`。

日常实车启动：工作区构建完成后，在根目录运行 `bash src/start_robot.sh`，统一启动底盘、`msg_MID360s_launch.py` 雷达、Yesense 和定位。Nav2 始终独立：定位初始化完成后，在另一终端运行 `bash src/start_nav2.sh`。两个脚本自动加载工作区环境、使用绝对地图路径，不自动下发导航目标。此次更新构建 `fast_lio`、`my_nav2_config` 和 `ml_bringup`，具体见 RUN_COMMANDS.txt。先停止已有同类节点；退出时先停止 Nav2，再停止驱动/定位。脚本尚待车上联调。

2026-09-24 确认雷达在建图后相对车头向左旋转 90°（俯视）。FAST-LIO C++ 参数默认值、mid360.yaml、localization.launch.py 和 robot.launch.py 均默认 +90°，不依赖 shell 脚本传角度。FAST-LIO 输出点云按 `R_base_imu` 转到车体系，输出朝向按 `R_odom_imu * R_base_imu.transpose()` 转换；前端内部状态、世界点云及地图坐标保持原定义。雷达与内置 IMU 外参不改，Yesense 未随雷达调整。保留原位置原点假设，此参数只处理安装 yaw，不代替雷达相对底盘的平移/倾角标定。旧地图和航点无需为此手动旋转，重新定位后的对齐效果仍需实车核验。旧安装 bag 必须显式传 `lidar_mount_yaw:=0.0`；如果绕过定位入口直接启动 FAST-LIO，则覆盖 `publish.imu_to_base_yaw:=0.0`。恢复原雷达安装时使用 `bash src/start_robot.sh lidar_mount_yaw:=0.0`。

## 数据链

```text
Livox LiDAR + 内置 IMU → FAST-LIO → /lidar_odometry → Local EKF → odom → base_link
Wheel 速度 + Yesense IMU 角速度 ─────────→ Local EKF
Local 位姿差分（x/y/yaw）+ ICP ──→ Global EKF → map → odom
body scan + 扫描时刻 TF + PCD地图 → ICP → map 系绝对位姿
SC 重定位 → Manager → Global EKF set_pose → 输出确认 → ICP start_tracking
```

Manager 仅负责启动初始化；完成后 ICP 即使 LOST 也继续用新扫描和 EKF 的 TF 尝试自恢复。GPS 未融合。定位入口不启动硬件驱动或播放 bag。

## 文件索引

| 包/文件 | 职责 |
|---|---|
| `fast_lio` | LiDAR + 内置 IMU 前端，定位入口关闭其 TF |
| `ml_icp_localization` | TF 初值、窗口子图、CPU FastGICP、质量判断与绝对位姿 |
| `ml_relocalization` / `ml_scan_context` | 独立 SC 重定位 / 公共描述子库 |
| `ml_localization_manager` | 启动结果验证及 EKF/ICP 交接 |
| `ml_bringup/launch/localization.launch.py` | 唯一正式定位入口，含可选 RViz |
| `ml_bringup/config/ekf` | 双 EKF 与 IMU 集成配置（其余参数归各模块 config） |
| `sc_pgo` | 原有建图后端；不参与持续定位 |
| `yesense_std_ros2` / `yesense_interface` / `serial_ros2` | Yesense 驱动及依赖 |
| `nmea_navsat_driver` | 保留 GPS 驱动，当前未接定位链 |

CPU FastGICP 已内置于 `ml_icp_localization/third_party/fast_gicp`，无需外部 fast_gicp 包。保留上游许可证、提交号和文件校验信息。

- [部署与启动参数](docs/DEPLOYMENT.md)
- [接口、TF 和时间约定](docs/INTERFACES.md)
- [已知问题及待验收项](docs/KNOWN_ISSUES.md)
- [参数索引](ml_bringup/config/README.md)

交付根目录只有 `map_jeck/` 和 `src/`。原 0915 定位地图完整保留在 `src/ml_bringup/map/`，二维导航地图及禁行区保留在 `map_jeck/`；不包含 bag。保留包内构建所需测试和许可证。

默认使用新版合法参数结构及 0915 原门限；各包 config 中带 `_qq` 后缀的配置用于 qq 门限对照，启动方式见 RUN_COMMANDS.txt。

Nav2 已补齐控制器的 `/local_odometry/filter` 订阅，并让点云转扫描和 waypoint_follower 跟随启动参数 `use_sim_time`。前进速度上限现为 0.8 m/s（MPPI 与速度平滑器同步），保留 4 秒预测时长，局部地图扩大为 8×8 米，`prune_distance=4.0` 米；最近路径点搜索距离显式保持原先的 2.5 米。控制频率、倒车/转向上限、原有代价权重、地图坐标系和点云高度过滤保持原值。局部地图栅格数由 100×100 增至 160×160，需在车上确认更新耗时能满足 5 Hz。尚未完成 ROS 节点或实车 0.8 m/s 验收；更新后重新构建 `my_nav2_config`、source 工作区并重启 Nav2。

新增 `CruiseSpeedCritic`，为正常行驶增加目标 0.8 m/s 的有界速度偏好：仅在局部路径足够长、与车头接近对齐、转向角速度较小，且预测通道的代价栅格全为已知零代价时启用；通道覆盖车体外接半径、0.15 米余量及允许的路径横向偏差。还要求最近 `/scan` 不超过 0.3 秒。转弯、接近路径终点、障碍/未知区域、扫描超时或倒车时退出偏好，保留原 MPPI 决策。该插件不发布或覆写 `/cmd_vel`，不设最低速度，也不修改其他碰撞代价。它促使控制器选择接近目标速度的轨迹，不保证实车精确匀速；CPU 延迟、底盘能力和定位误差仍需实测。可将 `FollowPath.CruiseSpeedCritic.enabled` 设为 `false` 做对照。`my_nav2_config` 现在包含 C++ 插件，必须构建后部署；本地完成策略与适配层隔离测试，目标 ROS 编译、插件加载和实车测试尚未完成。

底盘现仅在收到有效电机轮速后发布 `/wheel_odom`，实际话题频率随反馈到达率变化；断流不再把缓存轮速当新测量。`max_odom_gap=0.5` 秒限制轮速位姿积分跨越的缺失时段，首次/长缺口后的样本重新建立积分时间，正常新轮速仍发布；这不是电机停车超时。轨迹点仍按有效轮速保留，`path_publish_rate=2.0` Hz 仅降低 `/trajectory` 整条历史轨迹的显示发布频率。高频终端打印改为限频 DEBUG 日志。

多航点巡航脚本已去掉去程/回程各自的 wp202、wp203 局部折返，改走 wp201 与 wp204 之间的通道，并更新相邻朝向；其余航点和两端目标位姿保留。新连接按车体外接半径加 0.15 米余量核对了当前静态地图及禁行区，没有穿越障碍或未知栅格。`NavigateThroughPoses` 行为树的航点移除半径从 3.5 米改为 1.0 米，减少在约 2.4 米点距下提前跳过转弯点的情况。此半径不是终点到达容差；重规划频率和恢复动作保持原值。离线几何/配置检查通过，尚未验证实车摆头是否改善；仅在使用该巡航脚本和此行为树时生效，外部下发的航点不会自动改变。

FAST-LIO 的 `/cloud_registered_body` 提前到里程计之后、地图插入和显示发布之前，沿用原点云转换、坐标系和时间戳。新回归检查覆盖轮速断流/恢复、时钟回退、直行指令对称性、轨迹限频和点云发布顺序；尚未测得实车端到端延迟或直行误差的改善幅度。
