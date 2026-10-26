# Snapshot: Fast_LIO_Localization_ws

此目录是 **Fast-LIO + Nav2 Localization Workspace** 的一个只读源码快照，供版本追溯、差异比较与迁移参考。

## 快照信息

- 归档目录：`versions/fast_lio_localization_ws-74896732/`
- 原始位置标识：`Fast_LIO_Localization_ws`
- 保留文件：**288** 个，其中识别为源码/构建描述的文件 **142** 个
- 排除项：**40** 个（编译产物、缓存、日志、运行时大数据或本机配置）
- 内容指纹：`a17ef52dcfd16d6232d950589f16005a76c708b790649c30edc048fe47787250`

## 识别到的模块

- 包/模块：`fast_lio_loc`, `fast_lio`, `aloam_velodyne`, `livox_ros_driver2`, `serial`, `yesense_interface`, `yesense_std_ros2`
- 可执行入口：`fast_lio/fastlio_mapping`, `aloam_velodyne/ascanRegistration`, `aloam_velodyne/alaserOdometry`, `aloam_velodyne/alaserMapping`, `aloam_velodyne/alaserPGO`, `aloam_velodyne/kittiHelper`, `yesense_std_ros2/yesense_node_publisher`, `yesense_std_ros2/yesense_node_subscriber`
- 声明依赖：`ament_cmake`, `ament_cmake_auto`, `ament_copyright`, `ament_flake8`, `ament_lint_auto`, `ament_lint_common`, `ament_pep257`, `apr`, `boost`, `catkin`, `common_interfaces`, `geometry_msgs`, `git`, `image_transport`, `libpcl-all-dev`, `livox_ros_driver2`, `nav_msgs`, `pcl_conversions`, `pcl_ros`, `python3-pytest` 等 37 项

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
