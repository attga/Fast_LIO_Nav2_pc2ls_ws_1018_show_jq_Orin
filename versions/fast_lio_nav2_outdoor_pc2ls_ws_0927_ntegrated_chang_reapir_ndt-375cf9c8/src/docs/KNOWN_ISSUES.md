# 验证边界

当前工程已用 0927 新定位替换旧 ICP 定位。
启动、构建和验证边界以 [README](../README.md) 及 [RUN_COMMANDS](../RUN_COMMANDS.txt) 为准。

尚未完成本版实车验收。已修复向 NDT 传入已移除 pose_file 的启动错误。
FAST-LIO 关闭时的 DDS 销毁异常和 exit -11 尚未解决；本次不宣称解决该问题。
重定位节点在加载地图期间收到退出信号后，日志出现 context 已关闭时创建 guard condition 的错误；尚未完成目标 ROS 环境的退出时序验证。
运行中 LOST 不自动触发全局重定位；未知安装平移仍为零近似。

2026-09-29 复查：修正 Manager 初始日志将尚未初始化显示为 NORMAL_TRACKING 的误导。
启动时明确输出 startup_complete=0；后续状态变化将完整状态及 startup_complete 写入日志。
只有完成初始化并处于正常跟踪状态，才应继续导航。
离线核对了 NDT 28 项、SC 重定位 29 项、Manager 16 项实际启动/YAML 参数与代码声明，未发现其余参数名冲突。
4 组启动组合及 17 项外参测试通过；ROS Launch 动作使用替身，不等于实车节点运行验证。
