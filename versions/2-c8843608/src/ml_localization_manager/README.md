# Localization Manager

负责结果验证和交接，不包含 SC 或 NDT 配准算法、不发布 TF。

## 双 EKF 启动初始化

入口：`ml_bringup/localization.launch.py`。
配置：`ml_localization_manager/config/manager.yaml`。

```
SC /relocalized_pose → Manager → /global/set_pose
                                  ↓
                     Global EKF 输出确认（两条不同时间）
                                  ↓
                     /ml_localization/start_tracking 放行 NDT
                                  ↓
                     新扫描 TRACKING → NORMAL_TRACKING
```

- `startup_only=true`：第一次成功后只监控，LOST 不再次触发 SC；NDT 自恢复时回 NORMAL。
- NDT 状态中的 `INIT + waiting_for_tf=1` 表示扫描时刻 TF 暂未到达，不等同于配准 LOST；短时等待时 Manager 保持当前状态；超过处理时效窗口则由 processing_stale 标记不可用，等待后续新鲜 TRACKING。
- `injection_method=set_pose`：仅支持上述启动模式，不能再提供手工 init_pose。
- NDT `wait_for_initialization=true`：即使已有全局 TF 也等待 start_tracking，放行后只匹配
  时间晚于放行时刻的新扫描；未初始化时状态为 INIT，附加 waiting_for_initialization=1。
- SC 服务请求间隔120秒，不频繁重置有效结果确认计数（当前配置为两次）。正式入口设置 SC start_on_launch=false；Manager
  只接受自己已发出请求且扫描时间符合要求的结果。
- 接受前检查 frame、四元数、协方差、请求时间、结果年龄、局部位姿同步。
- 默认重定位结果最大年龄为6秒（2026-10-10，按静止初始化场景调整）；正式入口将同一配置传给重定位节点。仍使用真实扫描时间戳，并保留请求边界、未来时间和局部里程计同步检查。
- 先将 SC 位姿按平面约定保留 x/y/yaw，再用局部位姿增量推进到最新局部样本时间。
  T_map_base(t_i)=T_map_base(t_s)×inverse(T_odom_base(t_s))×T_odom_base(t_i)。
  使用现有最近样本与0.2秒容差；尚未增加时间插值或协方差传播。
- 保留 SC 协方差，只调用一次 set_pose，不再把同一结果发布到 reloc_pose 重复融合。
- 只有 service 应答和 Global EKF 实际输出一致后才调用 start_tracking；start_tracking应答后还要
  等到时间戳晚于应答时刻的有效 NDT TRACKING，才宣布初始化完成。
- set_pose 或 start_tracking 服务应答超时、交接确认超时：锁定 initialization_error=1，
  不自动重复初始化。先检查日志再重启测试。服务超时不代表服务端没有执行，
  所以不盲目重发；没有在此版本加入自动撤销或取消服务端操作。

`/ml_localization/manager_state` 包含状态、startup_complete、initialization_error、icp_heartbeat_stale。
初始化错误后对外发布 `INITIALIZATION_FAILED initialization_error=1`，迟到的 NDT 状态不能解除错误；检查原因后重新启动。
当前尚无运行中的人工重新初始化入口；再次测试需要重启这套启动流程。

## 支持范围与旧配置迁移

仅支持 set_pose 启动初始化，完成后只监控；独立启动默认值与正式 launch 一致。
`injection_method=set_pose` 与 `startup_only=true` 保留为兼容配置标记，其他值会拒绝启动。
移除 measurement 测量注入、手工 init_pose、运行中自动 SC 恢复，以及旧 ICP 位姿跳变参考。
不再订阅 `/ml_localization/icp_pose`，也不再发布 `/ml_localization/reloc_pose`。

旧参数 `init_pose`、`max_reloc_jump`、`reloc_grace_period`、`reloc_covariance_pos`、
`reloc_covariance_ori` 会明确拒绝启动，需从旧 YAML 删除。首次 SC 请求失败和超时重试仍保留。
旧 measurement 使用者需迁移到 Global set_pose 和启动交接接口，不能只删除参数后继续使用旧测量链。

命令见工作区 `RUN_COMMANDS.txt`。

初始化完成后停止更新 Local odom 历史，保留 NDT 状态监控。NDT 的 input_stale=1 或 processing_stale=1 在已完成启动后按不可用处理，不触发 SC；新鲜 TRACKING 可自行恢复。

接口更名：NDT 启动服务为 `/ml_localization/start_tracking`，不再提供旧 `/ml_localization/reset` 名称。确认 EKF 后调用，收到响应并观察新扫描 TRACKING 后完成交接。

## NDT 状态接口

Manager 仅订阅 `/ml_localization/status_diagnostic`（`diagnostic_msgs/msg/DiagnosticStatus`）。`state` 必须精确为 `INIT`、`LOST`、`DEGRADED` 或 `TRACKING`；缺失或未知状态不会刷新心跳，也不从 `message` 描述文本推断状态。`/ml_localization/status` 字符串话题仍供人工观察。

## 测量入口校验

Manager 校验重定位结果后执行初始化交接。NDT 测量直接发布给 Global EKF，由 NDT 发布端检查位姿、时间戳及输出方差；Manager 不拦截 Global EKF 的 NDT 输入。


## NDT 心跳存活监控

完成初始化后，超过 icp_status_timeout（默认 3 秒，必须有限且为正）未收到有效状态，Manager 进入 LOST，输出 icp_heartbeat_stale=1。监控使用 ROS 时间，正常仿真暂停不累计超时；1 Hz 定时检查会带来最多约一个检查周期的检测延迟，实际还受调度影响。

收到已识别状态后清除心跳超时标记，但 INIT/DEGRADED/LOST 不作为跟踪恢复；仍需有效 TRACKING。描述文本和旧字符串不能替代结构化心跳；无法识别的状态不能刷新心跳。启动阶段使用交接超时。

此功能只报告不可用，不自动重启节点、重复调用 set_pose 或触发 SC；不覆盖 Manager 自身退出。manager_state 仍按变化发布，不能把它当作周期性心跳。

## 内部职责

- `health_monitor.hpp`：NDT 状态快照、运行期健康判断和 ROS 时间心跳超时；不触发 SC 或调用服务。
- `manager.hpp`：初始化结果校验、局部运动推进、启动完成判定，以及对外状态变化通知。
- `ml_localization_manager.cpp`：消息解析、服务请求和 EKF 输出确认；健康标记统一从核心读取。

心跳恢复即使不改变 LOST 主状态，也会通过统一变化标记发布。初始化进度由核心的 InitializationStage 统一维护，服务请求 ID 和计时由适配层持有。

### 初始化阶段

等待重定位 → 准备注入 → 等待 set_pose 应答 → 等待 Global 输出确认 → 等待 start_tracking 应答 → 等待新扫描 TRACKING → 完成。
服务应答或交接超时进入失败，迟到应答不能解除失败；错误原因随 manager_state 的 reason 输出。
阶段转换有前置条件，不能绕过 set_pose 应答直接放行。Global 的两条不同时间样本检查仍在适配层执行。
start_tracking 明确返回 false 时保留原行为：回到 Global 确认阶段，允许后续有效样本再次尝试，仍受总交接超时约束。
SC 请求在途标记独立保留：触发应答与重定位结果是两条异步通道，不能用初始化阶段替代请求资源状态。

## ROS 交接边界回归

`test/check_handoff.py` 启动实际 Manager，并模拟传感器与服务，不播放 bag。
需先构建和 source 工作区，显式选择未使用的非零 ROS_DOMAIN_ID；脚本不自动探测该域是否空闲。
覆盖服务不可达、交接总超时、Global 不一致、缺少新扫描、明确拒绝重试和重复 Global 时间戳。
输出目录必须不存在；日志留在源码外，脚本退出时清理自己启动的 Manager。
为缩短验证时间，测试实例使用 3 秒交接窗口和 1 秒 SC 请求间隔，正式 YAML 不变。
测试按需手工执行，不自动加入 colcon test；命令见工作区 RUN_COMMANDS.txt。

Manager 只订阅 `/ml_localization/status_diagnostic` 作为配准心跳。
不再支持只有 String 状态输出的旧 ICP 节点；NDT 的字符串状态仍可供人工观察。

内部状态类型和日志采用 registration 命名；参数 `icp_status_timeout` 和输出字段 `icp_heartbeat_stale` 为兼容已有配置与消费者保持原名。
