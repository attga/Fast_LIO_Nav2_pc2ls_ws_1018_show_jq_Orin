# Snapshot: 2

此目录是 **Fast-LIO + Nav2 Localization Workspace** 的一个只读源码快照，供版本追溯、差异比较与迁移参考。

## 快照信息

- 归档目录：`versions/2-63168c06/`
- 原始位置标识：`2`
- 保留文件：**336** 个，其中识别为源码/构建描述的文件 **221** 个
- 排除项：**485** 个（编译产物、缓存、日志、运行时大数据或本机配置）
- 内容指纹：`c9b686cc6d0072b69a798b06536b167b8cd1aba7813ac8c8f6721d88ffda137b`

## 识别到的模块

- 包/模块：`chassis_driver`, `fast_lio`, `livox_ros_driver2`, `ml_bringup`, `ml_localization_manager`, `ml_ndt_localization`, `ml_relocalization`, `ml_scan_context`, `my_nav2_config`, `serial`, `yesense_interface`, `yesense_std_ros2`
- 可执行入口：`chassis_driver/chassis_driver_node`, `fast_lio/fastlio_mapping`, `ml_localization_manager/ml_localization_manager`, `ml_ndt_localization/ml_ndt_localization_node`, `ml_ndt_localization/replay_registration`, `ml_relocalization/relocalization_node`, `ml_relocalization/replay_relocalization`, `ml_scan_context/build_polar_database`, `ml_scan_context/polar_database_test`, `ml_scan_context/preprocessing_test`, `yesense_std_ros2/yesense_node_publisher`, `yesense_std_ros2/yesense_node_subscriber`
- 声明依赖：`ament_cmake`, `ament_cmake_auto`, `ament_cmake_gtest`, `ament_cmake_pytest`, `ament_cmake_python`, `ament_index_python`, `ament_lint_auto`, `ament_lint_common`, `apr`, `boost`, `diagnostic_msgs`, `eigen`, `fast_lio`, `geometry_msgs`, `git`, `launch`, `launch_ros`, `libopencv-dev`, `libpcl-all-dev`, `livox_ros_driver2` 等 65 项

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
