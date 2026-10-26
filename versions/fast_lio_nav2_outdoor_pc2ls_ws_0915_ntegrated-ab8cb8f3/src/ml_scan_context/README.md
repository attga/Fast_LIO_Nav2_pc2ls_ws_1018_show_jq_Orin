# Shared Scan Context

Extracted from this workspace's `sc_pgo/include/scancontext` without changing
Scan Context calculations or the `sc_db.bin` format. Both `sc_pgo` and
`ml_relocalization` link `scan_context`; neither maintains a private copy.
Headers retain `scancontext/` include paths. The timer helper moved here;
`sc_pgo/tic_toc.h` is a compatibility include. Original author and third-party
license notices are retained. This library has no GTSAM or pose-graph dependency.

The upstream `xy2theta` missing-return compiler warning remains outside this
structural extraction; invalid/degenerate input handling needs separate review.

## 来源核对（2026-09-21）

主体代码可追溯至 FAST_LIO_SLAM 提交 `1ab98da9c63e7f6f2306f1392ee4e045d6061b9f` 中的 `SC-PGO/include/scancontext/Scancontext.h` 和 `.cpp`。在线原文件与本地同一历史提交逐字节一致；当前版本还包含本工作区后续修改，不能宣称等同该上游版本。

- [历史头文件](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/1ab98da9c63e7f6f2306f1392ee4e045d6061b9f/SC-PGO/include/scancontext/Scancontext.h)
- [历史实现](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/1ab98da9c63e7f6f2306f1392ee4e045d6061b9f/SC-PGO/include/scancontext/Scancontext.cpp)

授权仍待确认：以上主体文件没有完整授权头，该提交的根目录 LICENSE 已移入 FAST_LIO，不能据此为 SC 指定许可证。package.xml 的 BSD 字段是现有元数据，不作为完成授权审计的证明。nanoflann 与 KDTree 文件中的独立授权头继续保留。
另一个 gisbi-kim/scancontext 仓库有不同的授权说明，但本轮没有建立其与这里具体 C++ 文件的对应关系，因此不直接套用。
