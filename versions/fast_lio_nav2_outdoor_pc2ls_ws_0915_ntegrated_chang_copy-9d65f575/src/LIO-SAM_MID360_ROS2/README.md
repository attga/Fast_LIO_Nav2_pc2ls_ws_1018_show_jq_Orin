# LIO-SAM_MID360_ROS2


### Related Package

  - [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
  - [LIO-SAM-MID360](https://github.com/nkymzsy/LIO-SAM-MID360)

## Nav2 闭环接入（重定位模式）

运行 `relocalization.launch.py` 后，本包独占发布 `map -> odom` 和
`odom -> base_link`。`/odom` 是可靠 QoS 的高频局部里程计，位姿与
`odom -> base_link` TF 使用同一份滤波结果，twist 是 `base_link` 坐标系下
由局部位姿时间窗口估计的角速度；纵向速度优先使用时间戳有效的
`/wheel_odom`。轮速缺失或超时才回退到位姿估计。原有 `/odometry/imu` 和
`/odometry/global` 继续发布；`/lidar_odom` 的 10 Hz 全局校正位姿不能
用作 Nav2 的速度反馈。

Nav2 参数文件中至少对齐以下设置（合并到现有参数文件，不要覆盖控制器插件配置）：

```yaml
controller_server:
  ros__parameters:
    odom_topic: /odom
    odom_duration: 0.1

velocity_smoother:
  ros__parameters:
    feedback: CLOSED_LOOP
    odom_topic: /odom
    smoothing_frequency: 20.0
    odom_duration: 0.1
    max_velocity: [0.6, 0.0, 1.0]
    min_velocity: [-0.6, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.5]
    max_decel: [-2.5, 0.0, -3.5]
    deadband_velocity: [0.0, 0.0, 0.0]

local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
```

不要同时运行发布 `map -> odom` 的 AMCL、SLAM 或双 EKF，以及发布
`odom -> base_link` 的另一个轮速里程计节点。若现有系统必须占用 `/odom`，
把 `nav2_odom_topic` 设成空字符串，并把 Nav2 的 `odom_topic` 指向
`/odometry/imu`。一个 TF 边只应有一个发布者。

速度参数在 `config/params.yaml` 中：`nav2_wheel_odom_topic` 和
`nav2_wheel_twist_frame` 必须匹配真实底盘输出；这里的 `body` 是 1012_bag 的
`child_frame_id`，其 X 轴需与 `base_link` 前进方向一致。若没有轮速，
将话题设为空字符串。`nav2_velocity_window` 是位姿回归窗口，
`nav2_velocity_time_constant` 是速度低通时间常数，`nav2_max_*` 是反馈的
物理上限。上述 Nav2 配置下，定位侧 `nav2_max_linear_accel=2.5`、
`nav2_max_angular_accel=3.5`，避免反馈被低于命令上限的限幅器持续压低。
这些数值仅在底盘实测能够达到时适用；若底盘加减速能力更低，应同时
降低 Nav2 命令限值。定位侧速度上限 1.2 m/s、2.0 rad/s 留有测量余量，
无需机械地压到 Nav2 的命令上限 0.6 m/s、1.0 rad/s。窗口增大会降低噪声，
也会增加反馈延迟；`nav2_linear_deadband=0.02` 与
`nav2_angular_deadband=0.03` 是反馈去噪，不等同于 smoother 的命令死区。
若需要准确控制极低速倒车，须用真实倒车数据重新标定定位死区和滤波延迟。
底盘如有可靠轮速，
优先用轮速或轮速与 IMU 融合得到的速度反馈；仅凭激光位姿不能保证轮胎
打滑时的瞬时速度准确。轮速同样不能检测轮胎打滑，需要另行做运动一致性检查。

启动后先检查：

```bash
ros2 topic info /odom -v               # 唯一 publisher、RELIABLE QoS
ros2 topic hz /odom                    # 应接近 IMU 里程计频率
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

低速直行、原地转向、停车和倒车时录制 `/odom`、`/tf`、`/cmd_vel`，检查
`twist.linear.x`、`twist.angular.z` 与实际运动方向一致，且变化率不超过
配置的加速度上限。若机器人不允许倒车，同时在所用 Nav2 控制器中禁用
负向速度采样或倒车轨迹，并让底盘驱动执行速度与加速度限幅。定位链路
无法替代底盘侧的执行安全约束。

