# 底盘与传感器描述

固定安装关系由 `urdf/robot.urdf.xacro` 描述，`robot_state_publisher` 唯一发布。
不发布 map→odom 或 odom→base_link，不修改点云、里程计或 IMU 数据。
仅有固定关节，不需要 joint_state_publisher。当前不包含碰撞体、惯性或外观模型。

## 文件分工

- `robot.urdf.xacro`：入口参数、固定关节宏、明确列出的父子坐标树。
- `../config/robot/*.yaml`：安装位置与角度的唯一配置来源；Xacro 用 `xacro.load_yaml` 读取，不在 XML 内再保存一份安装数值。
- `../ml_bringup/description.py`：校验 YAML 和旧命令行覆盖，调用 Xacro；不构造坐标树、不计算外参。
- launch：将展开结果交给 robot_state_publisher。

当前只有四条固定边，保留一个 `fixed_frame` 宏和一份模型文件。
宏也用于 wheel 的 body 别名，因此采用“坐标系”命名而不是“传感器”命名。
frame、joint 名称及参数单位保持原样。无需为当前规模拆出多层 include 或新增 ROS 包。

Xacro 默认读取相对路径 `../config/robot/rear_axle.yaml`，也可显式传入 mounts_file。
旧 imu_mount/wheel_mount 六元列表由 Python 转为空格分隔字符串，仅显式设置时覆盖 YAML；日常安装调整优先修改 YAML。
直接使用 Xacro 时这两个覆盖参数采用空格分隔的六个数；正式 launch 保持原来的列表语法和校验。

## 安装配置

- `config/robot/rear_axle.yaml`：新车临时配置，base_link 为后轴中心、X 前 Y 左 Z 上。
  Livox X 朝左且 Z 朝上时 yaw=+π/2；Yesense 轴向与底盘一致；wheel body 与后轴中心重合。
  按用户决定，Livox/Yesense 未知平移暂取零；这不是安装标定。Livox 已知 +90° 保留，Yesense/wheel 使用单位变换。取得实测前/左/上距离后更新；显式 null 或非有限值仍拒绝加载。
- xyz 单位米，rpy 单位弧度，T_parent_child 将子系点变换到父系。
- livox_imu_link→livox_lidar_link 使用当前 FAST-LIO 内部固定外参；设备整体旋转不改变它。
  后续若修改雷达—IMU 内部标定，须同步核对 FAST-LIO 配置；这里不重新估计该外参。

TF 树：base_link→body、base_link→gyro_link（可关闭）、base_link→livox_imu_link→livox_lidar_link。
正式入口将去畸变雷达系点云的 frame 设为 livox_lidar_link；独立建图入口默认仍为 lidar。

## 启动边界

正式 localization.launch.py 默认读取 rear_axle.yaml，可用 robot_mounts_file 显式选择；旧 imu_mount、wheel_mount 可显式覆盖。
原 wheel_odom_frame、yesense_mount 静态发布进程已移除，不要额外启动同名 TF 边。
独立 description.launch.py 用 mounts_file 指定安装文件，只用于检查固定 TF。
不要与正式入口同时启动两个 robot_state_publisher。
前端和重定位均已支持安装变换；rear_axle 是未知平移取零的新车近似配置，默认启用，仍需完成实车验证。不要给历史 bag 套用新车安装关系。

依赖：robot_state_publisher、xacro、python3-yaml。测试命令集中在工作区源码根目录 RUN_COMMANDS.txt。

## 参考实现与取舍

- [ROS 官方 Xacro 教程](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.rst)：使用属性和参数化宏集中常量、减少重复。
- [TurtleBot3 Humble 描述](https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble/turtlebot3_description/urdf/turtlebot3_burger.urdf)：传感器通过清楚的固定关节连接到底盘。
- [Jackal 描述](https://github.com/jackal/jackal/blob/noetic-devel/jackal_description/urdf/jackal.urdf.xacro)：通过可复用部件宏组织更复杂的模型；这里参考组织方式，未引入其 ROS1 启动写法或仿真内容。

2026-09-24 整理仅改变描述组织方式。8 种配置（新旧安装×Yesense 开关×有无旧参数覆盖）
展开后的 link/joint/安装数值与整理前完全一致。未调整定位算法和传感器外参。

2026-09-24 按用户要求删除旧车 legacy.yaml，两个 launch 与 Xacro 默认均改为 rear_axle.yaml。
旧 bag 不再具备内置安装配置；回放前须另外提供与录制车辆一致的安装 YAML。此前新旧配置对照属于删除前的历史验证。
