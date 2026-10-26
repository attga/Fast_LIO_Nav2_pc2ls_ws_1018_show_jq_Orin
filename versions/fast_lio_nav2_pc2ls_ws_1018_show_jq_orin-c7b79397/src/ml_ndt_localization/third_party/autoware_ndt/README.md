# Bundled Autoware NDT core

This directory contains the CPU multigrid NDT registration core from
`autowarefoundation/autoware_core`, commit
`16045061a9c10da468b60e190b8fab02110fa501`, package
`localization/autoware_ndt_scan_matcher`.

The nine C++ source/header files are copied byte for byte from that commit.
`UPSTREAM.json` records each imported file's source path and SHA-256 digest.
Only this README, the provenance manifest, and the CMake integration are local.
The original attribution is in `README.upstream.md`; `LICENSE` contains the
ndt_omp BSD-2-Clause license, and `LICENSE.Apache-2.0` and `NOTICE` are copied
from the Autoware repository root. Retain the Apache and PCL BSD-3-Clause
copyright/license notices embedded in the source files.

The static target is `ml_autoware_ndt`, exported as `ml_ndt_localization::ml_autoware_ndt` for the relocalization package. It publicly provides
the `include` directory and links PCL, Eigen, and OpenMP. Its x86 SIMD compile
options follow upstream and propagate to callers to keep Eigen settings
consistent. It adds no ROS or Autoware node dependency. The Autoware ROS node,
map loader and launch/configuration files are
outside this subset.

Use `<autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>` and
`pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>`.
The unmodified sources explicitly instantiate `PointXYZ` and `PointXYZI`.
Configure `pclomp::NdtParams`, call `setInputTarget()`, then
`align(output, initial_pose, source)` and inspect `getResult()`.
The result contains pose, iteration count, transformation probability, nearest
voxel transformation likelihood, Hessian, and iteration histories. Acceptance
policy and conversion from the application's point type belong to the caller.

The frozen comparison uses KDTREE, resolution 2.0 m, transformation epsilon
0.01, step size 0.1, 30 maximum iterations, two threads, no regularization,
and no line search. Changing parameters does not change the imported source.

To verify the imported files from this directory:

```bash
python3 - <<'VERIFY'
from pathlib import Path
import hashlib, json
manifest = json.loads(Path('UPSTREAM.json').read_text())
for item in manifest['files']:
    assert hashlib.sha256(Path(item['path']).read_bytes()).hexdigest() == item['sha256'], item['path']
print('All imported files match UPSTREAM.json')
VERIFY
```

The covariance helper is also imported unchanged. The project adapter uses its pose offsets, softmax weights and weighted XY covariance with PointXYZI scoring; it adds an isotropic measurement noise floor instead of upstream diagonal clipping.
