# Shared Scan Context

Extracted from this workspace's `sc_pgo/include/scancontext` without changing
Scan Context calculations or the `sc_db.bin` format. Both `sc_pgo` and
`ml_relocalization` link `scan_context`; neither maintains a private copy.
Headers retain `scancontext/` include paths. The timer helper moved here;
Use `scancontext/tic_toc.h` directly. Original author and third-party
license notices are retained. This library has no GTSAM or pose-graph dependency.

The upstream `xy2theta` missing-return compiler warning remains outside this
structural extraction; invalid/degenerate input handling needs separate review.

## 来源核对（2026-09-21）

主体代码可追溯至 FAST_LIO_SLAM 提交 `1ab98da9c63e7f6f2306f1392ee4e045d6061b9f` 中的 `SC-PGO/include/scancontext/Scancontext.h` 和 `.cpp`。在线原文件与本地同一历史提交逐字节一致；当前版本还包含本工作区后续修改，不能宣称等同该上游版本。

- [历史头文件](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/1ab98da9c63e7f6f2306f1392ee4e045d6061b9f/SC-PGO/include/scancontext/Scancontext.h)
- [历史实现](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/1ab98da9c63e7f6f2306f1392ee4e045d6061b9f/SC-PGO/include/scancontext/Scancontext.cpp)

授权仍待确认：以上主体文件没有完整授权头，该提交的根目录 LICENSE 已移入 FAST_LIO，不能据此为 SC 指定许可证。package.xml 的 BSD 字段是现有元数据，不作为完成授权审计的证明。nanoflann 与 KDTree 文件中的独立授权头继续保留。
另一个 gisbi-kim/scancontext 仓库有不同的授权说明，但本轮没有建立其与这里具体 C++ 文件的对应关系，因此不直接套用。

## 新增强极坐标库（第一阶段，尚未切换消费者）

`polar_context` 是独立Eigen目标，公开头文件位于 `include/ml_scan_context/`：

- `types.hpp`：配置、候选和变换方向。
- `polar_context.hpp`：增强分支分箱与全列移位匹配。
- `database.hpp`：关键帧入库、候选查询、保存和加载。

输入为调用者已完成预处理的 `Points`（XYZ double）；库不自动下采样、裁剪或增加高度。
`DatabaseConfig.preprocessing_id`必须标识坐标系与预处理约定，保存加载严格核对。
默认七视点只应用于地图入库，查询使用原始视点。返回的 `T_keyframe_query`满足：
`T_map_query = T_map_keyframe * T_keyframe_query`。
每关键帧只返回距离最小的一个视点假设，同分保留入库顺序。空匹配不产生候选。
`QueryOptions.allowed_keyframes`由消费者控制区域/历史过滤；空列表表示不限。
查询使用穷举作为回归基准，尚未验证全2373关键帧数据库实时性。

新文件格式是带参数头的 `MLPCDB 1` 文本，使用17位double精度；不兼容旧`SCDB`，
不读取或覆盖现有`sc_db.bin`。保存采用同目录临时文件再重命名；加载完成才替换内存。
本阶段不提供旧SCDB转换器、索引加速或生产算法选择参数。普通SC行为仍由旧目标维护。

使用示例：

```cpp
ml_scan_context::DatabaseConfig config;
config.preprocessing_id = "imu_xy60_voxel0.5_floor_height2_v1";
ml_scan_context::Database database(config);
database.add(keyframe_id, prepared_map_points);
auto candidates = database.query(prepared_query_points);
```

参数标识不会执行预处理；调用者仍须保证实际点云遵循约定。
来源和授权范围见 `UPSTREAM.md`；增强分支不应因旧BSD元数据被误认为已取得商用授权。
组件测试：构建开启`BUILD_TESTING`后运行`polar_database_test`。
冻结实测回归：`OPENBLAS_NUM_THREADS=1 python3 tools/place_recognition/scpp_cpp/verify_library.py`。

安装后CMake链接：`find_package(ml_scan_context REQUIRED)`，
`target_link_libraries(your_target ml_scan_context::polar_context)`。
旧消费者继续使用现有ament依赖方式，不需要改名。

## 统一预处理入口

`ml_scan_context/preprocessing.hpp`提供显式`Preprocessor`，与`Database`分开调用。
默认顺序为：删除非有限XYZ → XY半径严格小于60m → Z加2m → 原点固定0.5m体素质心。
负坐标使用floor，输出按三维体素键字典序排列。预处理不做TF，不保留强度，不用于NDT输入替换。

```cpp
ml_scan_context::Preprocessor preprocessing;
ml_scan_context::DatabaseConfig config;
config.preprocessing_id = preprocessing.identifier("livox_imu");
ml_scan_context::Database database(config);
database.add(keyframe_id, preprocessing.apply(raw_map_points));
auto candidates = database.query(preprocessing.apply(raw_query_points));
```

建库和查询必须共用同一参数及坐标约定，且仅预处理一次，避免重复加高度。
自动生成标识包含参数及版本；与早期手写标识不同的文件会被拒绝加载，应重新构建，不能随意修改文件头绕过检查。
本入口复现本地冻结实验，并非声称MATLAB `pcdownsample` 网格实现一致。
原始PCD回归：`OPENBLAS_NUM_THREADS=1 python3 tools/place_recognition/scpp_cpp/verify_preprocessing.py`。

### 离线增强 Polar Context 建库

安装后的独立工具读取数字文件名的关键帧 PCD，保留实际 ID（允许不连续，消费者须核对位姿库），不会转换坐标系。当前工具采用经过对照验证的固定预处理配置，不覆盖已有文件，输出必须为 `.mlpc`：

```bash
ros2 run ml_scan_context build_polar_database \
  /path/to/map/Scans \
  /path/to/output/map.mlpc \
  livox_imu
```

`livox_imu` 是输入坐标约定标签，不是 TF 查询请求。PCD 的 XYZ 必须为标量 float32。预处理为有限点筛选、XY < 60 m、Z + 2 m、0.5 m 原点固定 floor 体素质心；描述子采用 20×60、80 m、每格至少 5 点、7 个虚拟视点。查询端必须使用完全相同的预处理及标签。PCD 读取依赖只位于工具边界，`polar_context` 核心仍只链接 Eigen。

该文件不兼容旧 `sc_db.bin`，当前生产建图/重定位尚未切换。运行多个进程时不要共享输出路径。

### 全搜索比较缓存

`Database::add/load` 构建归一化描述子及有效列缓存；每次查询只准备一次查询描述子。仍遍历全部关键帧、虚拟视点和旋转，阈值及排序不变。`.mlpc` 继续保存原始描述子，加载时重建缓存，文件格式无需转换。

缓存增加内存占用，不保证满足实时重定位时效。底层可使用 `PolarContext::prepare()` / `comparePrepared()`，普通调用者继续使用Database即可。源码/头文件更新后应重建依赖包，不能混用不同版本的二进制。

### 查询线程配置

新库支持 `QueryOptions::threads`（默认1，必须为正整数），在单次查询内并行比较关键帧，按原顺序归并。同分排序及候选范围不变。OpenMP仅作为库的实现依赖；不修改全局线程设置或绑定CPU。改线程数不改变描述子文件，因此无需重新建库。

```cpp
ml_scan_context::QueryOptions options;
options.threads = 2;
auto candidates = database.query(prepared_points, options);
```

库现需Eigen及OpenMP；早期文档中的“只依赖Eigen”为并行接入前状态。当前测试/想法总入口见 `docs/SCAN_CONTEXT_STATUS.md`。
