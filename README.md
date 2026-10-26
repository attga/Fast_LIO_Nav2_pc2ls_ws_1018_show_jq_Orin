# Fast_LIO_Nav2_pc2ls_ws_1018_show_jq_Orin

ROS 2 Humble 定位与 Nav2 导航工作空间，包含 FAST-LIO、Polar 重定位、NDT 定位、双 EKF、Livox / Yesense 驱动和底盘接口。

本仓库由本地同名 ZIP 快照整理，作为个人私有副本保存。原压缩包的两层包装目录已移除，源码和地图保持原始文件内容；不包含组织仓库的 Git 提交历史。

## 内容

| 路径 | 说明 |
| --- | --- |
| `src/` | 12 个 ROS 2 包、启动脚本和原始项目文档 |
| `maps/map_1012/` | 三维定位地图、关键帧、Polar / Scan Context 数据 |
| `map_jeck/` | 二维导航栅格地图 |

上传保留 821 个原始文件，约 65.64 MB；排除了 222 个编译产物与缓存文件。未运行归档中的程序，也未重新进行构建或实车验证。

## 文档入口

- [原项目工作空间说明](src/README.md)
- [运行命令](src/RUN_COMMANDS.txt)
- [待验收事项](src/docs/KNOWN_ISSUES.md)
- [定位参数索引](src/ml_bringup/config/README.md)
- [定位管理器](src/ml_localization_manager/README.md)
- [NDT 定位](src/ml_ndt_localization/README.md)
- [重定位](src/ml_relocalization/README.md)

## 源码数据链

```text
Livox LiDAR / IMU → FAST-LIO → Local EKF
Polar 重定位 → Localization Manager → Global EKF 初始化
NDT 持续配准 + Local EKF → Global EKF → map → odom
点云转 LaserScan + 二维地图 → Nav2 → 底盘接口
```

具体话题、参数、硬件接口与启动顺序以原项目文档和配置文件为准。

## 使用前检查

压缩包没有包含启动脚本引用的顶层 `localization_config/` 目录。按现有 `src/start_robot.sh` 启动前，需要补齐并核对 `mounts.yaml`、`manager.yaml`、`relocalization.yaml` 等现场配置。

`map_jeck/` 未包含禁行区掩膜。原项目文档还列出了二维与三维地图对齐、安装参数标定及目标车辆验证等待验收事项，请保留这些实际状态，不将源码快照视为已完成整车验收的证明。

Livox-SDK2、ROS 2 / Nav2 依赖与设备配置需在目标 Linux 机器单独安装和设置；详见原始文档。

## 快照来源

- 文件：`Fast_LIO_Nav2_pc2ls_ws_1018_show_jq_Orin.zip`
- SHA-256：`743a43a9d8a3ec1027cc8d6bca8578e1857dca3fd233928e9eae938eef7a81b4`
- 保留各组件原有许可证及第三方说明；使用或分发前需核对其授权。

