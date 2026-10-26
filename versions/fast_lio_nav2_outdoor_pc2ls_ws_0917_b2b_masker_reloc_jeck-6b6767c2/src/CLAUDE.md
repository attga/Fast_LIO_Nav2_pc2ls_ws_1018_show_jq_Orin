# 工作区约定

先阅读 README.md、docs/INTERFACES.md 和 docs/KNOWN_ISSUES.md，再修改代码。

- 正式入口为 ml_bringup/launch/localization.launch.py；各模块参数在各自包 config；双 EKF 与 IMU 集成配置在 ml_bringup/config/ekf。
- 修改前说明目的、文件和风险；每次解决一个具体问题，避免扩大重构。
- TF 每条边只有一个发布者：Global EKF 的 map→odom，Local EKF 的 odom→base_link。
- ICP 是绝对位姿测量源，不发布 TF、不复制 EKF 状态、不管理 SC 流程。
- T_A_B 表示 B 到 A 的变换；观测使用传感器时间，缺少历史 TF 不用最新 TF 替代。
- 已验证的 SC 和地图格式优先保持不动。FastGICP 上游代码及许可证保留，版本信息见第三方目录。
- 不把地图、bag、构建产物或运行日志混进交付源码；回放倒带需重启整套链路。
- 测试命令统一维护 RUN_COMMANDS.txt；未完成的实测明确记录，不声称已经验收。
