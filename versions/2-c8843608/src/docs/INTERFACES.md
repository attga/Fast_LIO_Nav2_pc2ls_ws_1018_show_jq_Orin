# 集成定位与导航接口

`jq_reloc` 的 map1012 定位链统一使用 `map → odom → base_link`。Global EKF 独占发布 `map → odom`，Local EKF 独占发布 `odom → base_link`。FAST-LIO 关闭动态 TF；`robot_state_publisher` 发布 `base_link → livox_imu_link → livox_lidar_link` 和可选的 `base_link → gyro_link` 固定关系。

| 接口 | 生产者 | 消费者 |
|---|---|---|
| `/livox/lidar`、`/livox/imu` | Livox 驱动 | FAST-LIO |
| `/imu/data_raw` | Yesense 驱动 | IMU 适配器 |
| `/lidar_odometry`、`/cloud_registered_body` | FAST-LIO | Local EKF；NDT、Polar 和 Nav2 点云转扫描 |
| `/local_odometry/filter` | Local EKF | Global EKF、Manager、Nav2 控制器、BT、速度平滑器 |
| `/global_odometry/filter` | Global EKF | Manager 与监控 |
| `/ml_localization/ndt_pose` | NDT | Global EKF |
| `/relocalized_pose` | Polar 重定位 | Manager |
| `/ml_localization/manager_state` | Manager | 启动检查 |
| `/scan` | pointcloud_to_laserscan | Nav2 代价地图和 CruiseSpeedCritic |
| `/cmd_vel` | Nav2 速度平滑器 | 底盘驱动 |

定位地图 `maps/map_1012` 与二维地图 `map_jeck/map.yaml` 共用 `map` 坐标系的前提尚需现场核验。`map_jeck` 当前缺少禁行区掩膜，启动脚本会禁用 KeepoutFilter。
