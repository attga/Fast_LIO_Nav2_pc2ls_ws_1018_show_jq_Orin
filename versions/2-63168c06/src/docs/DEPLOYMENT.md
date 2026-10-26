# 部署

从本工作区根目录执行 [RUN_COMMANDS.txt](../RUN_COMMANDS.txt) 中的构建与启动命令。首次启动前检查 `localization_config/mounts.yaml`、Livox 网络地址、Yesense 串口及底盘 `can0`。安装参数来自 `jq_reloc` 的独立定位包，Livox yaw 默认 +90°；恢复旧安装时应修改 YAML 并重新启动。

定位源文件在 `src/fast_lio`、`src/ml_scan_context`、`src/ml_relocalization`、`src/ml_ndt_localization`、`src/ml_localization_manager`、`src/ml_bringup`。数据在 `maps/map_1012`。定位用 `/cloud_registered_body` 供 Nav2 生成 `/scan`，Local EKF 用 `/local_odometry/filter` 供 Nav2 控制器等节点消费。

`bash src/start_robot.sh` 启动驱动和定位。等待 `/ml_localization/manager_state` 出现 `startup_complete=1` 后，再执行 `bash src/start_nav2.sh`。该脚本只启动导航服务器，不自动下发目标。检查 `map → odom → base_link`、`/scan` 和二维地图坐标后，再做低速路线验证。

当前 `map_jeck` 没有禁行区掩膜，Nav2 默认无 KeepoutFilter；需要禁行区时，提供与 `map.yaml` 同坐标系的 `keepout_mask.yaml` 与 `keepout_mask.pgm`，并设置 `USE_KEEPOUT=true`。地图和航点与 map1012 的精确对齐尚未现场验证。
