# Snapshot: Fast_LIO_Nav2_outdoor_pc2ls_ws_0915_b2b_masker_lio

此目录是 **Fast-LIO + Nav2 Localization Workspace** 的一个只读源码快照，供版本追溯、差异比较与迁移参考。

## 快照信息

- 归档目录：`versions/fast_lio_nav2_outdoor_pc2ls_ws_0915_b2b_masker_lio-2be2938e/`
- 原始位置标识：`Fast_LIO_Nav2_outdoor_pc2ls_ws_0915_b2b_masker_lio`
- 保留文件：**1183** 个，其中识别为源码/构建描述的文件 **187** 个
- 排除项：**30** 个（编译产物、缓存、日志、运行时大数据或本机配置）
- 内容指纹：`e2c2f396192522e80d6ec94ca519124cf52f247e43faf830983dbe7806f829ab`

## 识别到的模块

- 包/模块：`autorccar_interfaces`, `chassis_driver`, `lio_sam`, `livox_ros_driver2`, `my_nav2_config`, `robot_localization`, `serial`, `yesense_interface`, `yesense_std_ros2`
- 可执行入口：`chassis_driver/chassis_driver_node`, `robot_localization/ekf_node`, `robot_localization/ukf_node`, `robot_localization/navsat_transform_node`, `robot_localization/robot_localization_listener_node`, `yesense_std_ros2/yesense_node_publisher`, `yesense_std_ros2/yesense_node_subscriber`
- 声明依赖：`Eigen`, `GTSAM`, `OpenCV`, `PCL`, `ament_cmake`, `ament_cmake_auto`, `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`, `angles`, `apr`, `autorccar_interfaces`, `boost`, `builtin_interfaces`, `diagnostic_msgs`, `diagnostic_updater`, `eigen`, `geographic_msgs`, `geographiclib`, `geometry_msgs` 等 62 项

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
