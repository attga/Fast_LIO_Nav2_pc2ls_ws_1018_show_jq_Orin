# 部署与操作

## 环境与依赖

已验证 Ubuntu 24.04、ROS 2 Jazzy、robot_localization 3.8.3、x86_64 CPU。Humble / Orin NX 需要独立构建和完整路线测试；本源码不是跨平台预编译包。

先安装 ROS 2、colcon、rosdep，并准备已构建的 `livox_ros_driver2`。本机 Livox overlay 位于 `/home/jackie/Desktop/ws_livox/install`；此驱动不在本交付源码中。
核心依赖包括 robot_localization、PCL、Eigen、OpenMP、OpenCV、tf2、RViz；建图后端 sc_pgo 另需 GTSAM 开发库及可发现的 CMake 配置。可在新工作区运行 `rosdep install --from-paths src --ignore-src -r -y`，但仍需检查外部 Livox/GTSAM 是否已满足。

将源码放入新工作区 src，source ROS 与 Livox 后构建：

```bash
colcon build --symlink-install --executor sequential \
  --packages-select fast_lio ml_scan_context ml_relocalization \
  ml_icp_localization ml_localization_manager sc_pgo ml_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

上述命令同时构建 `sc_pgo`，满足 bringup 保留的建图入口依赖。新工作区不需要旧 `temp_bag` 或独立 fast_gicp 的 install。硬件驱动按设备需要单独构建、启动。当前定位入口不会替你连接串口或启动 Livox。

## 地图

```text
map_directory/
  map.pcd
  isam_poses.txt
  sc_db.bin
  Scans/000000.pcd ...
```

位姿文件为 KITTI 每行 12 个数的 `T_map_body`；Scans 是 body 系单帧，描述子编号、位姿行号和扫描编号必须对应。地图和 bag 均需独立交付。不要混用不同地图的文件。
本机使用 `/home/jackie/Desktop/fast_lio_slam_ws/map`，bag 位于 `/home/jackie/Desktop/rubu/bags`。源码中历史 `ml_bringup/map` 数据保留在本机，但不打包、不作为正式默认地图。

## 正式启动参数

| 参数 | 默认值 / 含义 |
|---|---|
| `map_directory` | 必填，外部地图目录 |
| `pose_file` | `isam_poses.txt`，相对地图目录 |
| `use_sim_time` | false；bag 回放必须 true 并使用 --clock |
| `rviz` | true；无桌面环境设 false |
| `enable_yesense` | true；bag0909 缺少 /imu/data_raw，应设 false |
| `imu_mount` | `[0,0,0,0,0,0]`，T_base_link_gyro_link |
| `wheel_mount` | `[0,0,0,0,0,0]`，T_base_link_body |
| `ekf_config_directory` | ml_bringup/config/ekf，双 EKF 与 IMU 配置 |
| `fastlio_config` | fast_lio/config/mid360.yaml |
| `icp_config` | ml_icp_localization/config/localization.yaml |
| `manager_config` | ml_localization_manager/config/manager.yaml |
| `relocalization_config` | ml_relocalization 的 config/relocalization.yaml |
| `fastlio_log_directory` | 自动创建 ROS_LOG_DIR（或 ~/.ros/log）下带时间的子目录 |

安装位姿顺序 x/y/z/roll/pitch/yaw，单位米/弧度；单位变换是现有安装的近似，不是标定结果。
数值算法参数在 YAML 中修改，不再提供一长串重复 launch 参数。用 symlink-install 修改源码配置后重新启动；复制安装或交接新机器时重新构建安装。

## 启动与停止

1. source 环境，启动正式 launch；实车尽量静止等待初始化。
2. 启动硬件数据或按 RUN_COMMANDS.txt 回放指定 topic。不要播放 bag 内 TF 或旧定位输出。
3. 等待 Manager `NORMAL_TRACKING startup_complete=1`，确认 `/ml_localization/icp_pose` 和双 EKF 输出。
4. 完整路线检查 LOST、TF 连续性、ICP 拒绝和地图片段变化。短段成功只说明链路连通。
5. 先停止 bag/数据输入，再退出 launch。重放、倒带、更换地图时重启整个链路。

一次只运行一套定位入口。初始化失败应先读日志再重启；当前没有运行中人工重新初始化入口，也没有 LOST 后自动 SC 恢复。GPS 尚未接入。

## 保留的辅助入口

- `rviz.launch.py`：独立 RViz，bag 时加 `use_sim_time:=true`。
- `fast_lio2.launch.py`：独立前端调试，不能与正式定位同时启动。
- `mapping_mid360s.launch.py` / `mapping_mid360s_bag_pgo.launch.py`：历史实车/离线建图流程，保留原策略，未纳入本次定位交付验收。必须显式指定新输出目录，避免覆盖地图。
- `ml_relocalization/relocalization.launch.py`：独立 SC 调试，默认 pose_file 为 `isam_poses.txt`，与正式入口一致，不能与正式入口重复启动 SC。

历史实验入口不再提供。旧安装目录可能残留已删除的文件，升级时应清理旧包安装目录或使用新工作区，不能凭旧 launch 仍能补全就继续使用。
