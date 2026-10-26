# 待验收事项

- `maps/map_1012` 的三维定位地图与 `map_jeck/map.yaml` 的二维导航地图需要在现场用已知地标确认精确对齐，特别是历史航点坐标。
- `map_jeck` 未交付禁行区掩膜；`start_nav2.sh` 默认关闭 KeepoutFilter。不能把这一模式当作已有禁行区保护。
- `localization_config/mounts.yaml` 的平移为临时零值，尚未经过实测标定。雷达安装角度默认 +90°。
- `jq_reloc` 的 Manager 初始化、Polar 候选和 NDT 门限尚需在目标车辆、目标雷达网络及目标算力上完成整条路线验证。
- Livox ROS 驱动源码依赖单独提供的 Livox-SDK2 库和头文件；构建时指定目标机器对应架构的路径。
