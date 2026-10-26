# Snapshot: Fast_LIO_Nav2_outdoor_pc2ls_ws_0915_ntegrated

此目录是 **Fast-LIO + Nav2 Localization Workspace** 的一个只读源码快照，供版本追溯、差异比较与迁移参考。

## 快照信息

- 归档目录：`versions/fast_lio_nav2_outdoor_pc2ls_ws_0915_ntegrated-ab8cb8f3/`
- 原始位置标识：`Fast_LIO_Nav2_outdoor_pc2ls_ws_0915_ntegrated`
- 保留文件：**327** 个，其中识别为源码/构建描述的文件 **198** 个
- 排除项：**2377** 个（编译产物、缓存、日志、运行时大数据或本机配置）
- 内容指纹：`9bb2b0e909a6d4ea6f6e867283b98d179f53bf9b9c2b1c104c20bf57a1e7f707`

## 识别到的模块

- 包/模块：`chassis_driver`, `fast_lio`, `livox_ros_driver2`, `ml_bringup`, `ml_icp_localization`, `ml_localization_manager`, `ml_relocalization`, `ml_scan_context`, `my_nav2_config`, `nmea_navsat_driver`, `sc_pgo`, `serial`, `yesense_interface`, `yesense_std_ros2`
- 可执行入口：`chassis_driver/chassis_driver_node`, `fast_lio/fastlio_mapping`, `ml_icp_localization/ml_icp_localization_node`, `ml_localization_manager/ml_localization_manager`, `ml_relocalization/relocalization_node`, `sc_pgo/alaserPGO`, `yesense_std_ros2/yesense_node_publisher`, `yesense_std_ros2/yesense_node_subscriber`
- 声明依赖：`ament_cmake`, `ament_cmake_auto`, `ament_cmake_gtest`, `ament_index_python`, `ament_lint_auto`, `ament_lint_common`, `apr`, `boost`, `chassis_driver`, `diagnostic_msgs`, `eigen`, `fast_lio`, `geometry_msgs`, `git`, `gtsam`, `launch`, `launch_ros`, `libopencv-dev`, `libpcl-all-dev`, `livox_ros_driver2` 等 68 项

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
