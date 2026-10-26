# ml_icp_localization

点云地图绝对位姿测量源。输入 body 点云和扫描时刻的 TF，输出 map 系位姿。

## 代码分工

| 文件 | 职责 |
|---|---|
| `src/ml_icp_localization_node.cpp` | 扫描有界等待、TF 查询、初值、观测发布和 start_tracking 服务 |
| `src/node_parameters.hpp` / `src/node_parameters.cpp` | 节点参数定义、声明、读取及已有参数校验；包内使用 |
| `include/ml_icp_localization/tracker.hpp` / `src/tracker.cpp` | 单次定位流程、质量门限、连续失败与 LOST 状态 |
| `include/ml_icp_localization/scan_matcher.hpp` / `src/scan_matcher.cpp` | PCL/FastGICP 配准、目标缓存、地图协方差预计算和配准指标 |
| `include/ml_icp_localization/submap_cache.hpp` | 整图常驻、XY 窗口裁剪、边界触发更新与原图索引 |
| `include/ml_icp_localization/map_loader.hpp` | 关键帧位姿读取、单帧 PCD 缓存和关键帧子图基线策略 |
| `include/ml_icp_localization/point_types.hpp` | 共用点和点云类型 |

`ScanMatcher` 的 FastGICP 辅助实现只在 `.cpp` 内可见；它不是另一套定位算法。
ROS 节点持有 `Tracker`，后者持有子图缓存和配准器。配准调用由 Tracker 的互斥锁串行保护。

## 坐标与状态

- 初值：查询 `lookupTransform(map, base_link, scan_stamp)`，由 TF 完成链式组合与时间插值。
- 不订阅 Odometry，不维护局部里程计历史；Global EKF、Local EKF 各自提供 TF。
- body 点云先按初值转换到 map 系；配准器求修正 `T_target_source`。
- 绝对测量：`T_map_base = T_target_source * T_map_base_init`。
- 只有通过质量检查的结果发布给 EKF。LOST 时是否继续匹配由 `pause_on_lost` 决定。
- 地图窗口中心描述缓存覆盖范围，不是机器人位姿状态；本包不维护或发布 `map -> odom`。

## 缓存约定

`SubmapCache` 返回的点云在被替换后仍保持内容不变；索引与当前窗口点顺序一一对应。
窗口不变时复用配准目标、目标协方差和质量评估 KD-tree；更新时从整图协方差中提取对应项。
每次配准使用新的 source 点云，避免复用上一帧 source 协方差。
关键帧策略保留原有每轮重建配准器的基线行为。

外部参数仍为 `submap_strategy: window/keyframes`，topic、服务、可执行文件名和地图格式保持兼容。
质量门限、协方差、配准线程和子图策略保持不变。

## 扫描时间与等待

- 正常配准只使用扫描时刻 TF，不用最新 TF 替代缺失的历史变换。
- 两个扫描槽：OnCloud 更新最新输入，TakeScanForTracking 固定待处理帧；等待期间不被新帧替换，不积压整段扫描队列。
- ConsumePendingScan 统一记录消费时间戳并清槽；配准或丢弃才消费，等待 TF 不消费。
- TrackTick 负责门控、准备扫描、预处理、Tracker 调用及发布；last_attempt_scan_stamp 只在实际配准前更新。
- start_tracking 清除待处理槽但保留已消费时间戳；首次放行后只允许时间晚于放行时刻的扫描。
- TF 不可用时保留待处理扫描，定时器非阻塞重试；`tf_wait_timeout` 默认1.0秒，
  按 steady clock 计算，超时丢弃，不增加 GICP 失败计数。
- 等待从选入待处理槽开始；不是从传感器采样时刻起算，也不是传感器年龄门限。
- 成功、拒绝或超时后，同一最近已消费时间戳不会再次尝试。保留原来的相等去重，
  不承诺全历史去重；回放倒带仍应重启定位链路。
- 扫描时间戳0会被丢弃，避免TF把它解释成查询最新值。
- Pose和Path均使用扫描时间戳；`tf_wait` 为等待耗时，取代旧日志中的 `sync_dt`。
- 状态新增 `waiting_for_tf` 和累计 `tf_dropped`；TF等待/超时不是一次失败配准。
- 同时发布标准 `diagnostic_msgs/msg/DiagnosticStatus` 到 `/ml_localization/status_diagnostic`；旧 `/ml_localization/status` 字符串接口保留兼容。
- Manager 只在已完成启动后忽略 `INIT + waiting_for_tf=1`；真正的连续配准失败仍由 ICP 的 fail_count/LOST 状态处理。
- 参数 `input_ekf_topic`、`sync_tolerance` 已移除。

Manager启动模式仍使用 `wait_for_initialization=true`，经 start_tracking 放行后只处理新扫描。
ICP 不再支持手工 `init_pose` 启动，配准初值统一查询扫描时刻的 `map <- base_link` TF。
已移除仅供旧启动路径使用的 `init_pose` 和 `odom_frame` 参数；TF 中间链仍可包含 odom。
全局初值由 Manager 初始化 Global EKF，ICP 不生成或缓存另一套启动位姿。
独立调试时可关闭初始化等待，但仍须由外部提供覆盖扫描时刻的完整 TF。


点云回调只保存最新 ROS 消息；选中扫描且对应 TF 可用后，才转换 PCL 并按原 0.4 m 分辨率降采样。状态附加 `input_age`（ROS 时间减传感器时间）、`input_stale` 和 `processing_stale`；`scan_timeout` 默认 2 秒，超时或明显时间回退不再显示 TRACKING，但不增加配准 fail。仿真暂停且 /clock 停止时不按墙钟判定断流。

启动/恢复服务统一为 `/ml_localization/start_tracking`（std_srvs/Trigger），替代旧 `/ml_localization/reset`。用于 Manager 确认 EKF 初值后放行，兼容旧暂停模式的恢复；不携带位姿。普通 LOST 自恢复不需要调用它，当前双 EKF 启动配置为 `pause_on_lost=false`。

## CPU FastGICP 依赖

源码内置于 `third_party/fast_gicp`，从当前使用的上游提交原样复制 CPU 必要部分。作为静态库与 ICP 一同构建，不需要独立 fast_gicp ROS 包或外部 libfast_gicp.so。版本、文件校验值和 BSD-3-Clause 许可证随包保存。构建只需选择 ml_icp_localization；算法与配准参数保持原样。

## 测量发布校验

ICP 直接向 Global EKF 发布位姿，Manager 不在这条测量链路上。
协方差底噪参数仍表示标准差；必须为正，平方后能表示为有限正方差。
评分系数必须有限且非负，最大可接受评分对应的方差不得溢出。
发布前另行检查时间戳、frame、位姿、四元数和最终方差；非法测量不发布并记录错误。
最终检查失败时不更新 ICP 轨迹，状态附带 `output_invalid=1` 并在输入就绪、未 LOST 时报告 DEGRADED；下一次合法测量发布后清除。Tracker 的算法接受状态及配准失败计数保持独立。

参数读取集中在 `src/node_parameters.cpp`，按输入/时效、地图、配准、质量/恢复、输出噪声分组。
默认值来自 NodeParams/Tracker::Config；ValidateNodeParams 统一校验参数约束，不自动修正非法值。
`pose_covariance` 与 `orientation_covariance` 虽沿用历史名称，值仍为标准差，发布消息时才计算方差。

参数消费者、条件生效范围与待补校验见 [R2 审查记录](../docs/REFACTOR_R2_ICP.md)。
正式 launch/YAML 和 C++ 独立默认值并不完全相同，本轮保留两种入口的原行为。

## 参数错误与旧配置迁移

参数校验在地图读取前执行；错误记录原因并返回退出码 1。
`icp_rate` 须为有限正数，且 `1000/icp_rate` 在 [1, INT_MAX] 毫秒范围；保留现有整数毫秒截断方式。
对应距离及其平方须有限且为正；内点比例为 [0,1]，平移/yaw 跳变门限有限且非负。
yaw 门限大于 180° 仍允许；不将它强行截断。后端须为 pcl_gicp/fast_gicp，线程数至少 1。
window 检查 size/margin，keyframes 检查非负 halfwidth；自动 LOST 开启时失败帧数至少 1。
四个已移除参数 input_ekf_topic、sync_tolerance、init_pose、odom_frame 若出现在覆盖配置中，将明确拒绝启动，需从旧 ICP YAML 删除。
这些约束不改变正式 localization.yaml 数值；参数测试位于 test/parameters_test.cpp。
