# Localization Manager

负责结果验证和交接，不包含 SC 或 ICP 配准算法、不发布 TF。

## 双 EKF 启动初始化

入口：`ml_bringup/localization.launch.py`。
配置：`ml_localization_manager/config/manager.yaml`。

```
SC /relocalized_pose → Manager → /global/set_pose
                                  ↓
                     Global EKF 输出确认（两条不同时间）
                                  ↓
                     /ml_localization/start_tracking 放行 ICP
                                  ↓
                     新扫描 TRACKING → NORMAL_TRACKING
```

- `startup_only=true`：第一次成功后只监控，LOST 不再次触发 SC；ICP 自恢复时回 NORMAL。
- `injection_method=set_pose`：仅支持上述启动模式，不能再提供手工 init_pose。
- ICP `wait_for_initialization=true`：即使已有全局 TF 也等待 start_tracking，放行后只匹配
  时间晚于放行时刻的新扫描；未初始化时状态为 INIT，附加 waiting_for_initialization=1。
- SC 服务请求间隔120秒，不频繁重置三次确认。正式入口设置 SC start_on_launch=false；Manager
  只接受自己已发出请求且扫描时间符合要求的结果。
- 接受前检查 frame、四元数、协方差、请求时间、结果年龄、局部位姿同步和原跳变门。
- 先将 SC 位姿按平面约定保留 x/y/yaw，再用局部位姿增量推进到最新局部样本时间。
  T_map_base(t_i)=T_map_base(t_s)×inverse(T_odom_base(t_s))×T_odom_base(t_i)。
  使用现有最近样本与0.2秒容差；尚未增加时间插值或协方差传播。
- 保留 SC 协方差，只调用一次 set_pose，不再把同一结果发布到 reloc_pose 重复融合。
- 只有 service 应答和 Global EKF 实际输出一致后才调用 start_tracking；start_tracking应答后还要
  等到时间戳晚于应答时刻的有效 ICP TRACKING，才宣布初始化完成。
- set_pose 或 start_tracking 服务应答超时、交接确认超时：锁定 initialization_error=1，
  不自动重复初始化。先检查日志再重启测试。服务超时不代表服务端没有执行，
  所以不盲目重发；没有在此版本加入自动撤销或取消服务端操作。

`/ml_localization/manager_state` 包含状态、startup_complete、initialization_error。
初始化错误后对外发布 `INITIALIZATION_FAILED initialization_error=1`，迟到的 ICP 状态不能解除错误；检查原因后重新启动。
当前尚无运行中的人工重新初始化入口；再次测试需要重启这套启动流程。

## 旧测量方式

默认 `injection_method=measurement`、`startup_only=false` 保留原单 EKF/旧恢复模式：
验证后发布 `/ml_localization/reloc_pose`，由订阅它的 EKF 融合，再确认和 start_tracking。
不要在新启动配置中同时订阅该重定位测量和执行同一结果的 set_pose。

命令见工作区 `RUN_COMMANDS.txt`。

启动专用模式不订阅 ICP pose；set_pose 模式不创建旧 reloc_pose 测量发布器。初始化完成后停止更新 Local odom 历史，保留 ICP 状态监控。ICP 的 input_stale=1 在已完成启动后按不可用处理，不触发 SC；新鲜 TRACKING 可自行恢复。

接口更名：ICP 启动服务为 `/ml_localization/start_tracking`，不再提供旧 `/ml_localization/start_tracking` 名称。确认 EKF 后调用，收到响应并观察新扫描 TRACKING 后完成交接。
