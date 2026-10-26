# map1012 定位 + Nav2 导航工作区

此工作区将 `jq_reloc` 的六个定位源码包与现有底盘、Livox、Yesense 驱动和 Nav2 配置放在同一个 ROS 2 Humble 工作区。

## 数据链

`/livox/lidar` + `/livox/imu` → FAST-LIO → `/lidar_odometry`、`/cloud_registered_body`；Local EKF 发布 `odom → base_link` 和 `/local_odometry/filter`；Polar 重定位给 Global EKF 初值，NDT 持续配准，Global EKF 发布 `map → odom` 和 `/global_odometry/filter`。Nav2 从 `/cloud_registered_body` 生成 `/scan`，控制器与速度平滑器订阅 `/local_odometry/filter`，规划使用 `map_jeck/map.yaml`。

定位地图为 `maps/map_1012/`（`map.pcd`、`isam_poses.txt`、`Scans/`、`polar.mlpc` 等）；实车安装与定位门限在 `localization_config/`。`map_jeck/` 是二维导航地图，两类地图必须处于同一个 `map` 坐标系。当前文件有坐标范围重叠，但尚无实车证据证明栅格、禁区和航点与 map1012 精确对齐。

## 构建和启动

ROS 2 Humble 机器上安装 `colcon`、PCL、Eigen、OpenCV、`robot_localization`、`pointcloud_to_laserscan`、Nav2 及各包 `package.xml` 所列依赖。进入本目录执行：

```bash
source /opt/ros/humble/setup.bash
export LIVOX_SDK_ROOT=/path/to/Livox-SDK2/install
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential --packages-up-to ml_bringup my_nav2_config chassis_driver yesense_std_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DLIVOX_LIDAR_SDK_LIBRARY="$LIVOX_SDK_ROOT/lib/liblivox_lidar_sdk_shared.so" -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$LIVOX_SDK_ROOT/include"
source install/setup.bash
bash src/start_robot.sh
```

启动脚本统一启动底盘、MID360s、Yesense 和定位，不发送导航目标。硬件接口沿用驱动配置中的 `can0`、雷达网段及 IMU 串口。不要同时启动另一套驱动或定位。看到 `/ml_localization/manager_state` 中 `startup_complete=1` 后，在另一个已 source 同一工作区的终端执行：

```bash
bash src/start_nav2.sh
```

Nav2 脚本自动检查 `map_jeck`：若 `keepout_mask.yaml` 和 `keepout_mask.pgm` 都存在则启用禁行区；当前目录缺少这两个文件，因此默认无禁行区运行，代价地图仍使用静态地图和实时 `/scan`。强制要求禁行区可设 `USE_KEEPOUT=true`，缺文件时脚本会报错退出。地图/航点对齐和禁行区确认前，不应下发实车导航目标。启动命令不会自动发送目标。

详见 [RUN_COMMANDS.txt](RUN_COMMANDS.txt) 及各定位包的 README。修改安装角度时编辑 `localization_config/mounts.yaml`，不要使用旧版 `lidar_mount_yaw` 启动参数。日志写入 `logs/localization/` 和 `logs/nav2/`。
