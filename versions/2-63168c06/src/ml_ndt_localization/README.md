# ml_ndt_localization

基于固定版本 Autoware NDT core 的持续地图定位观测源。输入为扫描时刻的 `base_link` 点云和 TF 查询得到的 `T_map_base` 初值，输出为 `map` 系绝对位姿测量。没有里程计积分、EKF、TF 广播或全局重定位控制。

| 文件 | 职责 |
|---|---|
| `src/node.cpp` | ROS 接口、有界扫描等待、传感器时间 TF、观测/诊断与初始化放行 |
| `src/node_parameters.*` | 参数读取、非法配置与旧参数拒绝 |
| `src/tracker.cpp` | 窗口选择、单次定位、质量检查和连续失败计数 |
| `src/ndt_matcher.cpp` | 原生 NDT 调用、目标结构缓存、独立覆盖率统计 |
| `include/ml_ndt_localization/submap_cache.hpp` | 常驻地图与 XY 窗口缓存 |
| `third_party/autoware_ndt` | 原样上游算法代码、许可证与校验清单 |
| `test` | 参数、地图、六自由度配准、质量和离线回归 |

## 输入与输出

- 输入 `/cloud_registered_body`：PointCloud2，frame 必须为 `base_frame`，时间戳必须大于零。订阅为 reliable、深度10；发布者也须提供可靠QoS，best_effort发布者不兼容。
- 初值：同一个 scan stamp 的 `map ← base_link` TF，包含完整六自由度。不是最新 TF，不订阅 Odometry。
- 输出 `/ml_localization/ndt_pose`：PoseWithCovarianceStamped，绝对 `T_map_base`、map frame、原扫描时间。Global EKF 同步订阅该话题。
- 调试 `/ml_localization/ndt_path` 与 `/ml_localization/map`；地图 transient_local。
- 状态 `/ml_localization/status_diagnostic` 与 `/ml_localization/status`。Manager 使用通用健康字段；NDT 新增 NVTL、TP、迭代次数和目标构建耗时。
- 服务 `/ml_localization/start_tracking`：Trigger。Manager 确认 Global EKF 初始化后放行，不注入或保存位姿。
- TF：仅消费，不发布。Global EKF 发布 map→odom，Local EKF 发布 odom→base_link。

## 配准流程

扫描 → 显式剔除非有限坐标 → 0.4 m 体素 → 按 TF 预测选择地图窗口 → NDT(body scan, absolute map pose) → 质量检查 → 原时间戳测量。

整图 `map.pcd` 加载并降采样一次。当前YAML配置300 m方形窗口；离边界100 m时切换，保留完整高度。相同窗口复用 NDT 概率体素；切换时同步重建，日志分别记录 `crop_ms`、`target_build_ms`、`registration` 与 `track_total`。未实现后台预构建。覆盖率检查有独立最近邻树，仅用于统计，不是 NDT 对应点缓存。

NDT 使用绝对初值并返回绝对位姿，不再执行旧 GICP 的“预变换扫描、匹配单位矩阵、再乘初值”路径。

## 质量与状态

核心没有 PCL `hasConverged()` API；本适配层的 `converged` 表示有限结果和 `0 < iterations < max_iterations`，不代表已证明定位正确。接受还要求 NVTL ≥ 2.3、完整扫描 1 m 内点率 ≥ 0.4、平移修正 ≤ 2 m、yaw 修正 ≤ 15°。TP 仅记录，不使用 ICP fitness。

INIT / TRACKING / DEGRADED / LOST 的 Manager 协议保持。默认 LOST 继续尝试新扫描，通过即清除失败计数；不会重复处理最后一帧。TF 等待过期不计作配准失败。节点不触发 SC。

默认启用动态 XY 协方差：复用 Autoware MULTI_NDT_SCORE 的偏移采样和加权统计，在 map 系评分分布方差上叠加 0.09 m² 底噪；保留 XY 非对角项。z 方差仍为 0.09 m²，roll/pitch/yaw 方差仍为 0.0025 rad²。`covariance.dynamic_xy: false` 可恢复固定值。该估计尚未实车标定，不代表真实误差，也不替代质量门控；详见 `docs/NDT_DYNAMIC_COVARIANCE.md`。

## 2026-10-12 可靠订阅验证

1012同包自定位全量×1：基础ScanContext启动成功，持续NDT1192次接受、0拒绝，初始化后无输入过期、DEGRADED或LOST；最大更新间隔0.604秒，全部进程正常退出。可靠订阅已纳入正式源码，详见 [构建与验证记录](../test_results/ndt_reliable_production_20261012/SUMMARY.txt)。单机单包验证不保证所有负载下无背压，也不证明独立定位精度。

## 构建与配置

`colcon build --packages-select ml_ndt_localization`；仅需要当前 ROS/PCL/Eigen/OpenMP，毋需安装完整 Autoware。正式入口由 ml_bringup 提供。参数见 `config/localization.yaml`；旧 `icp_config` 改为 `ndt_config`，旧 GICP 参数会明确报错。

Autoware core 1.8.0，commit `16045061a9c10da468b60e190b8fab02110fa501`，算法文件未修改。更换上游时先核对 UPSTREAM.json，再进行相同输入矩阵回归。完整原生 ROS scan matcher 的地图服务、TPE、初始化和诊断依赖未导入。详见 `docs/ML_NDT_MIGRATION.md`。
