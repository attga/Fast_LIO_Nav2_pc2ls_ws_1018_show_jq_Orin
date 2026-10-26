# Shared Scan Context

Extracted from this workspace's `sc_pgo/include/scancontext` without changing
Scan Context calculations or the `sc_db.bin` format. Both `sc_pgo` and
`ml_relocalization` link `scan_context`; neither maintains a private copy.
Headers retain `scancontext/` include paths. The timer helper moved here;
`sc_pgo/tic_toc.h` is a compatibility include. Original author and third-party
license notices are retained. This library has no GTSAM or pose-graph dependency.

The upstream `xy2theta` missing-return compiler warning remains outside this
structural extraction; invalid/degenerate input handling needs separate review.
