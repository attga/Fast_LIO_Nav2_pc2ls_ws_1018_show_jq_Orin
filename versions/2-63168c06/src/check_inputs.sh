#!/usr/bin/env bash
# Source the integrated workspace and inspect localization/navigation inputs.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE/install/setup.bash"
for topic in /livox/lidar /livox/imu /imu/data_raw /lidar_odometry /cloud_registered_body /local_odometry/filter /global_odometry/filter /ml_localization/ndt_pose /scan; do
  echo "检查 $topic（最多5秒）"
  timeout 5 ros2 topic hz "$topic" || true
done
timeout 5 ros2 topic echo /ml_localization/manager_state --once || true
ros2 service list | grep -E 'trigger_relocalization|global/set_pose|start_tracking' || true
