> 当前交付：2026-09-15 集成测试版。Global 使用 Local 位姿差分 + ICP；已知限制见 docs/KNOWN_ISSUES.md。

# MID360 园区定位

供同事接手和继续实测的工程基线。当前验证环境为 Ubuntu 24.04 / ROS 2 Jazzy / x86_64；Humble 和 Jetson Orin NX 尚未验收。

正式定位入口：`ros2 launch ml_bringup localization.launch.py map_directory:=<地图绝对路径>`。
本机完整多终端命令见 [RUN_COMMANDS.txt](RUN_COMMANDS.txt)。bag 回放必须加 `use_sim_time:=true` 并发布 `/clock`。

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

历史实验入口、项目测试脚本、临时参考源码和重复命令已移出交付源码；地图和 bag 独立提供，不放入源码压缩包。保留上游驱动自带的测试和许可证。
