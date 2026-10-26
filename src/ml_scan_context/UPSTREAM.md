# 上游来源与本地边界

旧 `src/Scancontext.cpp` / `include/scancontext` 保持当前消费者兼容，不在本轮改变行为。

新 `polar_context` 目标参考 gisbi-kim/scancontext_tro，main，commit
b463bb7159a883ae8c36b5825547299c5bbded26 的 TRO2022
`scancontext/desc/ptcloud2polarcontextAug.m` 与 `scancontext/alg/sc_dist_with_argalign.m`。
是本地增强极坐标移植，不是完整SC++；不含Cartesian Context。
上游仓库声明CC BY-NC-SA 4.0，新移植部分及其使用须遵守上游许可，不因旧package.xml的BSD字段获得额外授权。商业部署前需解决许可适用性。

本地差异：输入由调用者预处理；忽略非有限点；空匹配返回无穷距离；参数校验；
显式T_keyframe_query；按关键帧去重；新的MLPCDB版本化文件，不读取旧SCDB。
不提供ROS、NDT、地图全局位姿、时间排除策略或系统状态机。
对比MATLAB体素网格一致性尚未验证；当前依据冻结Python/C++离线原型核对。
