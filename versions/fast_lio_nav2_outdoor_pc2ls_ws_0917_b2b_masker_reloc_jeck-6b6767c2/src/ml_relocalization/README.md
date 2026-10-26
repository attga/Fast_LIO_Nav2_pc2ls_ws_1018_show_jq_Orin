# 重定位模块

独立包 `ml_relocalization`；可执行文件 `relocalization_node`。暂时保留 ROS 节点名
`alaserRelocalizer`，兼容现有参数查询与监控。旧命令 `ros2 run sc_pgo alaserRelocalizer`
通过薄启动脚本转发，不包含第二份实现。新启动文件直接使用独立包。

## 代码结构

- `ml_scan_context`：从 SC-PGO 提取的公共 Scan Context 算法与二进制数据库读写。
  建图和重定位链接同一份库；没有引入 GTSAM 依赖，保留上游版权。
- `keyframe_map`：KITTI 位姿、索引检查、PCD 惰性缓存、候选关键帧邻域子图。
- `relocalizer`：SC 候选检索、yaw 初值、子图与精配准串联。
- `registration`：PCL ICP 与原质量门限，输出绝对 `T_map_body`。
- `confirmation`：有效配准结果的位置/yaw 一致性计数。
- `relocalization_node`：ROS 参数/输入输出、单工作线程、去重、触发代次与空闲状态。

线程所有权：回调更新最新扫描槽；工作线程独占地图缓存和 SC 检索器。
服务与最终确认/发布共用会话锁；配准期间不持锁，旧触发代次不能发布结果。
关闭时通知工作线程并等待当前配准结束，没有 detached 线程。

## 输入输出

| 接口 | 类型 | 约定 |
|---|---|---|
| `/cloud_registered_body` | sensor_msgs/PointCloud2 | 输入 `base_link`，当前项目为 MID360 IMU body；best effort depth 10 |
| `/relocalized_pose` | geometry_msgs/PoseWithCovarianceStamped | 输出 `map` 下的 body 位姿，沿用扫描时间戳 |
| `/trigger_relocalization` | std_srvs/Trigger | 重置确认状态，重新开始尝试 |
| `/relocalization/current_hyp`、`submap`、`aligned` | sensor_msgs/PointCloud2 | 可选调试输出，map 系，改为对应扫描时间戳 |

不订阅 odometry，不发布 TF，不更新 EKF，不接管 Localization Manager。
当前仅支持 Scan Context；GPS 候选生成与朝向搜索尚未接入。
GPS 后续可复用精配准，不依赖 ICP 定位节点的可视化点云 topic。

## 数据与变换

`sc_db.bin` 描述子索引 = pose 行索引 = `Scans/%06d.pcd` 编号。
pose 为行主序 KITTI 3×4 的 `T_map_body`；Scans 为 body 系。
与原实现一样检查所需扫描文件存在和描述子/位姿数量一致。
`map.pcd` 不是当前 SC 重定位必需输入，地图格式不变。

`T_A_B` 将 B 系点变到 A 系：

```
T_map_body_hypothesis = T_map_candidate * RotZ(yaw_diff_sign * sc_yaw)
scan_hypothesis = T_map_body_hypothesis * scan_body
T_map_body = T_icp_correction * T_map_body_hypothesis
```

平移门限比较机器人位置的实际差值，不是 correction 矩阵的平移列。
保留 PCL ICP（不是连续定位中的 FastGICP）、100 次迭代及原始 fitness 算法。

## 运行行为与已知边界

启动后自动尝试，约 1 Hz，每次只取最新扫描；同一最近已处理时间戳不重复配准。
成功后空闲，直到服务触发。触发不清除最后已处理时间戳。
确认比较最终位置/yaw，不要求候选编号相同；**拒绝帧不清空此前确认计数**，
因此准确含义是“连续有效结果一致”，不是原注释所说的逐帧连续成功。
这些行为此次保持，时间倒退/旧扫描重发不提供全历史去重。

协方差仍固定：xyz 方差 0.5 m²，rpy 方差 0.05 rad²，未做质量自适应。
原始子图点数门限仍在降采样前判断。动态环境中的错误候选、确认超时、
GPS 搜索范围和地理对齐质量需另行验证，不因包拆分得到解决。
只增加非法体素尺寸/确认帧数/子图范围启动检查、错误输入 frame 限频提示、
调试云传感器时间戳、异常捕获及可唤醒线程退出；匹配门限和位姿算法不变。

正式入口见 `ml_bringup/launch/localization.launch.py`，同时启动 SC 和 Manager。
命令见工作区 `RUN_COMMANDS.txt`，独立调试约定见 `docs/DEPLOYMENT.md`。

`start_on_launch` 默认 true，便于独立测试；Manager 集成启动覆盖为 false，只通过 trigger 开始。集成与独立启动加载同一份 `config/relocalization.yaml`，地图目录和 pose_file 由 launch 覆盖。

`pose_file` 在 C++ 默认值、YAML 和独立 launch 中均为 `isam_poses.txt`，对应优化后的 `T_map_body`。`odom_poses.txt` 为前端未优化位姿，不应与优化地图混用；只有明确使用相应未优化地图时才显式覆盖。
