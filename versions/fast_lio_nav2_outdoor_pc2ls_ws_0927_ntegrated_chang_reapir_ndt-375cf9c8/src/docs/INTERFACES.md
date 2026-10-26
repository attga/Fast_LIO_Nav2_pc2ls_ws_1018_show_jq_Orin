# 定位接口

当前工程已用 0927 新定位替换旧 ICP 定位。
启动、构建和验证边界以 [README](../README.md) 及 [RUN_COMMANDS](../RUN_COMMANDS.txt) 为准。

持续 NDT 输入 /cloud_registered_body 和扫描时刻 TF，输出 /ml_localization/ndt_pose。
SC+NDT 重定位输出 /relocalized_pose，由 Manager 交接给 Global EKF。
Local EKF 发布 odom→base_link，Global EKF 发布 map→odom；安装固定边由 URDF 发布。
Nav2 单独启动，订阅接口及参数沿用原工程。
