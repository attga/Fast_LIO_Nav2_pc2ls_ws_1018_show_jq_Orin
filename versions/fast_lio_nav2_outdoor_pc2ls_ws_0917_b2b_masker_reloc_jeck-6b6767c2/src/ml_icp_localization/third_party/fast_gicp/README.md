# FastGICP CPU dependency

Source: https://github.com/koide3/fast_gicp
Commit: `0e7ec1441c99f7be453db2ea216d5de029387417`
License: BSD-3-Clause; see LICENSE.

Copied from the existing clean `src/temp_bag/fast_gicp` checkout. The algorithm files are byte-for-byte unchanged; UPSTREAM.json records their SHA-256 hashes.
Only the CPU FastGICP/LsqRegistration implementation and required headers are included. GPU/VGICP/ST backends, apps, Python bindings and datasets are not part of this dependency.

The local CMakeLists builds a package-private static target `ml_fast_gicp`; it does not invoke upstream ROS package discovery or install another fast_gicp package. The containing ICP executable includes the implementation; no external libfast_gicp.so is required. License and provenance are installed with ml_icp_localization.

For updates, copy this same subset from a reviewed upstream revision, update the manifest, rebuild and validate TF-only ICP input with a recorded route. Do not edit the vendored algorithm to tune localization; use the ICP configuration instead.
