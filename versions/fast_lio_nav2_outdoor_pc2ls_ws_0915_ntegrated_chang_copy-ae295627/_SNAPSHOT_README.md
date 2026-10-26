# Snapshot: Fast_LIO_Nav2_outdoor_pc2ls_ws_0915_ntegrated_chang_copy

此目录是 **Fast-LIO + Nav2 Localization Workspace** 的一个只读源码快照，供版本追溯、差异比较与迁移参考。

## 快照信息

- 归档目录：`versions/fast_lio_nav2_outdoor_pc2ls_ws_0915_ntegrated_chang_copy-ae295627/`
- 原始位置标识：`Fast_LIO_Nav2_outdoor_pc2ls_ws_0915_ntegrated_chang_copy`
- 保留文件：**492** 个，其中识别为源码/构建描述的文件 **140** 个
- 排除项：**6** 个（编译产物、缓存、日志、运行时大数据或本机配置）
- 内容指纹：`bb814bb640ff0b7e8c164f42ae600861fb9637a0223658c7d40bfa5297e3946b`

## 识别到的模块

- 包/模块：`chassis_driver`, `lio_sam`, `livox_ros_driver2`, `my_nav2_config`, `serial`, `yesense_interface`, `yesense_std_ros2`
- 可执行入口：`chassis_driver/chassis_driver_node`, `lio_sam/test_nav2_velocity_estimator`, `yesense_std_ros2/yesense_node_publisher`, `yesense_std_ros2/yesense_node_subscriber`
- 声明依赖：`Eigen`, `GTSAM`, `OpenCV`, `PCL`, `ament_cmake`, `ament_cmake_auto`, `ament_lint_auto`, `ament_lint_common`, `apr`, `boost`, `geometry_msgs`, `git`, `libpcl-all-dev`, `livox_ros_driver2`, `nav2_bringup`, `nav2_bt_navigator`, `nav2_common`, `nav2_controller`, `nav2_costmap_2d`, `nav2_lifecycle_manager` 等 50 项

## 构建参考

```bash
source /opt/ros/<ros2-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 上述命令是根据目录结构给出的通用入口，不代表该历史快照已在当前环境重新构建。请先阅读本目录内原始 README、package.xml、CMakeLists.txt 或工程文件。

## 安全与兼容性

- 真实车辆、CAN、串口、电机、相机和激光雷达接入前，应先在隔离环境核对设备号、协议、坐标系、单位及限位。
- 本次归档未修改原始源码，也未把构建产物、缓存和大体积运行数据作为源码上传。
- 本目录中的第三方代码、模型说明和许可证仍按其各自条款使用。
