# 重定位模块

独立包 `ml_relocalization`；可执行文件 `relocalization_node`。暂时保留 ROS 节点名
`alaserRelocalizer`，兼容现有参数查询与监控。旧命令 `ros2 run sc_pgo alaserRelocalizer`
通过薄启动脚本转发，不包含第二份实现。新启动文件直接使用独立包。

## 代码结构

- `ml_scan_context`：从 SC-PGO 提取的公共 Scan Context 算法与二进制数据库读写。
  建图和重定位链接同一份库；没有引入 GTSAM 依赖，保留上游版权。
- `keyframe_map`：KITTI 位姿、索引检查、PCD 惰性缓存、候选关键帧邻域子图。
- `relocalizer`：SC 候选检索、yaw 初值、子图与精配准串联。
- `registration`：Autoware NDT 与独立质量门限，输出绝对 `T_map_body`。
- `confirmation`：有效配准结果的位置/yaw 一致性计数。
- `relocalization_node`：ROS 参数/输入输出、单工作线程、去重、触发代次与空闲状态。

线程所有权：回调更新最新扫描槽；工作线程独占地图缓存和 SC 检索器。
服务与最终确认/发布共用会话锁；配准期间不持锁，旧触发代次不能发布正式位姿或调试点云。
关闭时通知工作线程并等待当前配准结束，没有 detached 线程。

## 输入输出

| 接口 | 类型 | 约定 |
|---|---|---|
| `/cloud_registered_body` | sensor_msgs/PointCloud2 | 输入 `base_frame`（默认 base_link）；节点按安装 TF 转到 matching_frame 后再匹配；best effort depth 10 |
| `/relocalized_pose` | geometry_msgs/PoseWithCovarianceStamped | 输出 `map` 下的 base_frame 位姿，沿用扫描时间戳 |
| `/trigger_relocalization` | std_srvs/Trigger | 重置确认状态，重新开始尝试 |
| `/relocalization/current_hyp`、`submap`、`aligned` | sensor_msgs/PointCloud2 | 可选调试输出，map 系，改为对应扫描时间戳 |

不订阅 odometry，不发布 TF，不更新 EKF，不接管 Localization Manager。
当前仅支持 Scan Context；GPS 候选生成与朝向搜索尚未接入。
GPS 后续可复用精配准，不依赖持续定位节点的可视化点云 topic。

## 数据与变换

`sc_db.bin` 描述子索引 = pose 行索引 = `Scans/%06d.pcd` 编号。
pose 为行主序 KITTI 3×4 的 `T_map_body`；Scans 为 body 系。
与原实现一样检查所需扫描文件存在和描述子/位姿数量一致。
`map.pcd` 不是当前 SC 重定位必需输入，地图格式不变。

`T_A_B` 将 B 系点变到 A 系：

```
T_map_body_hypothesis = T_map_candidate * RotZ(yaw_diff_sign * sc_yaw)
scan_hypothesis = T_map_body_hypothesis * scan_body
T_map_body = NDT(scan_body, map_submap, T_map_body_hypothesis)
```

平移门限比较机器人位置的实际差值，不是 correction 矩阵的平移列。
采用固定 Autoware NDT core（与 ml_ndt_localization 共用同一导出库），2 m 概率体素、30 次最大迭代、2 线程。门控包括 NVTL≥2.3、完整扫描 1 m 地图覆盖率≥0.4、相对 SC 初值位置≤5 m 与航向修正≤30°。不沿用 ICP fitness 门限；icp_* 旧参数拒绝启动。输出方差与确认/交接逻辑不变。

## 运行行为与已知边界

启动后自动尝试，约 1 Hz，每次只取最新扫描；同一最近已处理时间戳不重复配准。
成功后空闲，直到服务触发。触发不清除最后已处理时间戳。
确认比较转换后的底盘位置/yaw，不要求候选编号相同；**拒绝帧不清空此前确认计数**，
因此准确含义是“连续有效结果一致”，不是原注释所说的逐帧连续成功。
这些行为此次保持，时间倒退/旧扫描重发不提供全历史去重。

协方差目前固定且参数化：`relocalization_position_variance` 为 xyz 方差（默认 0.5 m²），`relocalization_orientation_variance` 为 rpy 方差（默认 0.05 rad²），未做质量自适应。 两个方差参数必须为有限正值，非法配置在加载地图前报错退出；参数直接写入消息方差，不再次平方。
原始子图点数门限仍在降采样前判断。动态环境中的错误候选、确认超时、
GPS 搜索范围和地理对齐质量需另行验证，不因包拆分得到解决。
保留非法体素尺寸/确认帧数/子图范围启动检查、错误输入 frame 限频提示、
调试云传感器时间戳、异常捕获及可唤醒线程退出；匹配门限和位姿算法不变。

正式入口见 `ml_bringup/launch/localization.launch.py`，同时启动 SC 和 Manager。
命令见工作区 `RUN_COMMANDS.txt`，独立调试约定见 `docs/DEPLOYMENT.md`。

`start_on_launch` 默认 true，便于独立测试；Manager 集成启动覆盖为 false，只通过 trigger 开始。集成与独立启动加载同一份 `config/relocalization.yaml`，地图目录和 pose_file 由 launch 覆盖。

`pose_file` 在 C++ 默认值、YAML 和独立 launch 中均为 `isam_poses.txt`，对应优化后的 `T_map_body`。`odom_poses.txt` 为前端未优化位姿，不应与优化地图混用；只有明确使用相应未优化地图时才显式覆盖。

## 单次匹配与任务边界

节点内 `takeSnapshot` 将点云指针、传感器时间、安装变换与任务代次组成 AttemptSnapshot。
`attempt` 只执行该快照的匹配，`finishAttempt` 处理调试输出、多帧确认与正式结果发布。
单工作线程配合最新扫描槽限制积压；匹配期间新扫描可更新输入槽，不改当前快照。
同一任务后续确认使用新扫描；重复触发重置确认计数并增加代次，但保留已尝试扫描时间戳。
正式位姿发布前检查任务代次，检查/确认/发布与触发重置由同一个 session_mutex 串行化。

调试云、确认计数和正式位姿共用任务代次与停止检查；旧代次结果整体丢弃。
关闭通过停止标记唤醒工作线程并 join；正在执行的配准不会立即取消，退出可能等待该次匹配返回。
停止标记与结果提交共用会话/停止锁；停止标记设置后不再提交结果。正在提交的发布先完成，已进入 DDS 队列的消息不会被撤回。
ROS context 失效时也跳过结果提交；context 关闭与 DDS 发布并非原子操作，不承诺撤回信号到达前已经开始的提交。
调试云转换/发布在提交临界区，开启调试时触发可能短暂等待；配准仍在锁外执行。

重复触发、确认计数重置、成功后空闲、再次触发及配准中旧正式结果拦截已通过实际 bag0908 短时测试，证据见 docs/REFACTOR_R3_RELOCALIZATION.md。
Trigger 的 success 表示请求已接受，不代表定位完成；定位结果仍由 /relocalized_pose 独立输出。

## 输入合法性

重定位扫描时间须为正，ROS 纳秒字段须小于 10⁹；负秒、零时间或非法纳秒字段直接丢弃。
体素滤波前逐点检查 XYZ，不信任 is_dense 标记；剔除 NaN/Inf，保持有效点顺序和强度，空结果不替换最新扫描槽。
输入 frame 必须等于 base_frame；不新增扫描时间新鲜度策略，仍使用传感器时间。
地图 KITTI 位姿必须为有限刚体变换，旋转正交误差和行列式偏差容差均为 10⁻³；非法位姿报告行号并拒绝加载，不自动拉正地图。
这些校验不等于 PointCloud2 所有布局错误、地图 PCD 所有损坏形式都已覆盖。

## 底盘与匹配坐标系（2026-09-24）

- `base_frame`：输入点云与最终位姿描述的机器人参考点，默认 base_link。
- `matching_frame`：SC/NDT 的传感器参考系；独立启动默认 base_link 保持旧接口，正式启动覆盖为 livox_imu_link。
- 两者相同无需 TF；不同时在扫描时间查询 `T_base_matching`，仅依赖安装关系，不依赖 map→odom 或已初始化的 EKF。
- SC 体素滤波前转换 `p_matching = inverse(T_base_matching) * p_base`，保留历史传感器中心描述子约定。
- 核心输出 `T_map_matching`（代码沿用 T_map_body 名称），节点转换 `T_map_base = T_map_matching * inverse(T_base_matching)` 后做一致性确认并发布。
- 扫描和安装变换一起进入最新槽及匹配快照；执行期间不会改用另一帧的安装关系。
- 缺少扫描时刻的安装 TF／非法旋转或平移时丢弃该输入，等待后续新扫描，不用单位阵或最新动态 TF 兜底。
- 输出方差参数仍表示最终底盘位姿的固定方差，不是匹配器估计的传感器协方差；不宣称已完成质量自适应或外参不确定性标定。
- debug 点云仍在 map 系，反映传感器扫描配准结果；重定位不广播 TF，不新增系统状态。
- 独立使用新前端输出时，需设置 matching_frame:=livox_imu_link 并启动同一份安装描述。命令见根目录 RUN_COMMANDS.txt。

## 初始化任务的时间边界

触发时记录 ROS 时间、清空扫描缓存和确认计数；早于触发的扫描，即使在触发后才到达也被拒绝。选帧和结果提交均复核时效，计算期间过期则清空确认并继续等待新扫描，不发布成功结果。重复触发使旧在途结果失效；输出保留传感器采集时间。
正式 launch 将 Manager YAML 中 max_reloc_age 和 reloc_sync_tolerance 同时传给两个节点；独立运行的默认值分别为 1.5 秒和 0.2 秒。后者在重定位侧作为未来时间容差。扫描时间拒绝日志包含 phase/reason/stamp/request/now/age。
持续时钟偏差或前端积压不会因本修复消失，禁止通过改写扫描时间戳绕过。
