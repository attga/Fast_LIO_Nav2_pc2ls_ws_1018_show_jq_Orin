# 变更清单 / Change List

**日期**: 2026-09-11  
**问题**: 小车导航过程中突然加速失控（线速度 ~2 m/s，角速度 ~10 rad/s）  
**根本原因**: `velocity_smoother` 工作在 `CLOSED_LOOP` 模式，将 LIO-SAM IMU 预积分产生的瞬时速度尖峰正反馈放大，导致失控。

---

## 文件 1：`src/my_nav2_config/config/nav2_params.yaml`

### 修改内容

| 参数 | 修改前 | 修改后 | 说明 |
|------|--------|--------|------|
| `velocity_smoother.feedback` | `CLOSED_LOOP` | `OPEN_LOOP` | 切断速度反馈回路，防止 odom 尖峰被正反馈放大 |
| `velocity_smoother.max_velocity` | `[0.5, 0.0, 1.0]` | `[0.4, 0.0, 1.0]` | 线速度上限降至 0.4 m/s |
| `velocity_smoother.max_accel` | `[2.5, 0.0, 3.2]` | `[1.0, 0.0, 2.0]` | 限制加速度，避免急加速 |
| `velocity_smoother.max_decel` | `[-2.5, 0.0, -3.2]` | `[-2.0, 0.0, -3.0]` | 限制减速度 |

---

## 文件 2：`src/LIO-SAM_MID360_ROS2/src/relocalization.cpp`

### 修改内容

在 `hrOdomThread` 函数发布 `/local_lidar_odom` 之前，新增速度硬限幅：

```cpp
// [安全补丁] 限幅：防止 IMU 预积分异常值通过 odom 话题传递给 Nav2
const double MAX_LINEAR_VEL  = 1.5;  // m/s
const double MAX_ANGULAR_VEL = 2.5;  // rad/s

linear_x  = std::clamp(linear_x,  -MAX_LINEAR_VEL,  MAX_LINEAR_VEL);
angular_z = std::clamp(angular_z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
```

**位置**: `hrOdomThread` 函数内，`nav_msgs::msg::Odometry` 消息赋值之前。

---

## 文件 3：`src/LIO-SAM_MID360_ROS2/launch/relocalization.launch.py`

### 修改内容

| 修改类型 | 说明 |
|----------|------|
| 新增参数 `use_lidar_driver` | 允许通过命令行控制是否启动雷达驱动 |
| 新增参数 `use_rviz` | 允许通过命令行控制是否启动 RViz |
| 新增 `livox_driver_node` 节点 | 当 `use_lidar_driver:=true` 时自动启动 MID360 驱动 |
| 修正地图默认路径 | 移除硬编码的 `/home/syj/` 前缀，改用包内相对路径 |

**修改前报错**:
```
ros2 launch lio_sam relocalization.launch.py use_lidar_driver:=false use_rviz:=false
# 报错: use_lidar_driver and use_rviz are not defined
```

**修改后正常启动**:
```bash
ros2 launch lio_sam relocalization.launch.py use_lidar_driver:=false use_rviz:=false
```

---

## 文件 4：`start_navigation_with_rosbag.sh`（新增文件）

**作用**: 自动化录制 rosbag，包含完整且经过校验的话题订阅列表。

**录制话题列表**（话题名称已全部审查修正）：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/local_lidar_odom` | `nav_msgs/Odometry` | LIO-SAM 输出里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 输出速度指令 |
| `/cmd_vel_smoothed` | `geometry_msgs/Twist` | 平滑后速度指令 |
| `/odom` | `nav_msgs/Odometry` | 底盘里程计 |
| `/tf` | `tf2_msgs/TFMessage` | TF 变换 |
| `/tf_static` | `tf2_msgs/TFMessage` | 静态 TF |
| `/livox/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/plan` | `nav_msgs/Path` | 全局路径 |
| `/local_plan` | `nav_msgs/Path` | 局部路径 |
| `/costmap_raw` | `nav2_msgs/Costmap` | 原始代价地图 |
| `/costmap_updates` | `map_msgs/OccupancyGridUpdate` | 代价地图增量更新 |
| `/filter_mask` | `nav_msgs/OccupancyGrid` | Keepout Mask 话题 |

---

## 文件 5：`start_navigation_with_log.sh`（新增文件）

**作用**: 一键启动导航系统，监控进程健康状态，崩溃时自动告警并记录日志。

**功能**：
- 检测各关键节点是否存活
- 任一节点崩溃时输出告警日志
- 完整运行日志写入 `navigation_log_<timestamp>.txt`

---

## 安全链路对比

```
【修改前 — 危险链路】
  IMU 尖峰
    → LIO-SAM 预积分
    → /local_lidar_odom（含速度毛刺，峰值 ~2 m/s）
    → velocity_smoother [CLOSED_LOOP]  ← 正反馈放大！
    → /cmd_vel（失控，~2 m/s）
    → 小车飞出

【修改后 — 安全链路】
  IMU 尖峰
    → LIO-SAM 预积分
    → [硬限幅 ±1.5 m/s，±2.5 rad/s]（relocalization.cpp）
    → /local_lidar_odom（干净）
    → velocity_smoother [OPEN_LOOP]（nav2_params.yaml）
    → /cmd_vel（平稳，≤ 0.4 m/s）
    → 小车正常行驶
```

---

## 下一步操作建议

```bash
# 1. 重新编译（必须！relocalization.cpp 已修改）
cd /path/to/0910_mask
colcon build --symlink-install

# 2. 加载环境
source install/setup.bash

# 3. 启动定位（不带雷达，用 rosbag 替代）
ros2 launch lio_sam relocalization.launch.py \
  use_lidar_driver:=false use_rviz:=false

# 4. 启动导航
ros2 launch my_nav2_config nav2_bringup.launch.py \
  map:=src/my_nav2_config/maps/lio_sam_map/map_edited_cleaned.yaml \
  mask:=src/my_nav2_config/maps/lio_sam_map/keepout_mask.yaml \
  use_keepout:=true use_ekf:=false

# 5. 回放数据包验证修复效果
ros2 bag play rosbag2_2026_09_11-16_43_06 --clock
```
