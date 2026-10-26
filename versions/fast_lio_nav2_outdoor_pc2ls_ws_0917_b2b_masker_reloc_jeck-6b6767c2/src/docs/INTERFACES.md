# 定位接口与时间约定

`T_A_B` 将 B 系坐标转换到 A 系。当前 base_link 对应 MID360 内置 IMU body。

## TF 权威

| TF | 唯一发布者 |
|---|---|
| map → odom | Global EKF |
| odom → base_link | Local EKF |
| base_link → body | launch 静态 TF，wheel 轴向近似 |
| base_link → gyro_link | launch 静态 TF，启用 Yesense 时 |

FAST-LIO 和 ICP 均不发布定位 TF；不存在单独 map→base_link 发布者。ICP 在 scan 时间查询完整 TF 链，得到 `T_map_base_init = T_map_odom × T_odom_base`。

## Topic / Service

| 接口 | 类型 | 数据流 |
|---|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg | 驱动/bag → FAST-LIO |
| /livox/imu | sensor_msgs/msg/Imu | MID360 内置 IMU → FAST-LIO |
| /imu/data_raw | sensor_msgs/msg/Imu | Yesense → IMU 适配器，含重力 |
| /imu/measurements | sensor_msgs/msg/Imu | 适配器 → Local EKF，当前仅融合角速度 |
| /wheel_odom | nav_msgs/msg/Odometry | wheel vx/wz → Local EKF，twist 在 body 系 |
| /lidar_odometry | nav_msgs/msg/Odometry | FAST-LIO → Local EKF，仅位姿 |
| /cloud_registered_body | sensor_msgs/msg/PointCloud2 | FAST-LIO → ICP、SC，base_link 系 |
| /local_odometry/filter | nav_msgs/msg/Odometry | Local EKF 输出，odom 系；Manager 用于启动时间推进 |
| /global_odometry/filter | nav_msgs/msg/Odometry | Global EKF 输出，map 系；Manager 确认 set_pose |
| /ml_localization/icp_pose | geometry_msgs/msg/PoseWithCovarianceStamped | ICP → Global EKF，map 系绝对位姿 |
| /ml_localization/icp_path | nav_msgs/msg/Path | ICP 调试轨迹 |
| /ml_localization/status | std_msgs/msg/String | ICP 状态及质量、输入时效 → Manager/监控 |
| /relocalized_pose | geometry_msgs/msg/PoseWithCovarianceStamped | SC → Manager，map 系绝对位姿 |
| /trigger_relocalization | std_srvs/srv/Trigger | Manager 触发 SC |
| /global/set_pose | robot_localization/srv/SetPose | Manager 设置 Global EKF 初值 |
| /ml_localization/start_tracking | std_srvs/srv/Trigger | Manager 确认 EKF 后放行 ICP，不携带位姿 |
| /ml_localization/manager_state | std_msgs/msg/String | 启动状态、startup_complete、initialization_error |

双 EKF 输出统一为 `/local_odometry/filter` 和 `/global_odometry/filter`，旧 `/odometry/filtered` 与 `/global_odometry/filtered` 不再由正式入口发布。

FAST-LIO 原 `/Odometry` 已统一更名为 `/lidar_odometry`，不同时发布旧话题；外部订阅者需更新。消息类型、frame 和时间戳保持原样。

旧 `/ml_localization/reset` 已更名，不再使用。正式 set_pose 模式不发布 `/ml_localization/reloc_pose`，避免同一重定位结果重复融合。ICP 不订阅 Odometry；Global 融合 Local 输出的 x/y/yaw 位姿差分，不直接订阅 FAST-LIO、wheel 或 IMU。

## 启动交接

Manager 请求 SC → 校验结果 → 用 Local 位姿增量推进结果到当前局部样本时间 → 一次 set_pose → 服务应答及两条不同时间 Global 输出确认 → start_tracking 应答 → 新扫描 TRACKING → 启动完成。

推进关系：`T_map_base(t_i) = T_map_base(t_s) × inverse(T_odom_base(t_s)) × T_odom_base(t_i)`。当前使用最近样本、0.2 秒同步容差，并按平面恢复处理。
启动完成后 Manager 只监控；ICP LOST 后继续从 TF 取初值、尝试新 scan，通过质量门限即可恢复。不盲目重复注入 EKF 初值。

## 时间与质量

FAST-LIO 点云与位姿使用扫描末端时间，ICP/SC 观测保留对应扫描时间。IMU 适配不更改时间戳。
ICP 用待处理/最新两个槽限制积压；历史 TF 缺失时等待最多 1 秒 steady time，超时丢弃且不计为配准失败。正常路径不使用最新 TF 代替扫描时刻 TF，同一最近已消费时间戳不再处理。
输入年龄基于 ROS 时间；bag 暂停且 /clock 停止时不会按墙钟判断断流。回放倒带不保证全历史去重，需要重启。
Global EKF 保留 2 秒历史处理迟到观测，并预测到当前时间；30 Hz 是目标发布率，不是算力不足时的实时保证。

ICP 有质量门限而非固定“融合权重”：协方差和 EKF 的状态不确定性共同决定修正。当前平移标准差为 max(0.3, 3×fitness)，旋转标准差为 max(0.05, 0.5×fitness)，再平方成方差；尚未做统计标定。
