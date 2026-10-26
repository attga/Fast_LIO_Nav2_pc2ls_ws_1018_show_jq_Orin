#!/usr/bin/env bash
# Run with: bash src/start_robot.sh [rviz:=true]
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE"

if [[ ! -r "$WORKSPACE/install/setup.bash" ]]; then
  echo "错误：缺少 $WORKSPACE/install/setup.bash，请先在车上完成 colcon build。" >&2
  exit 1
fi
# Colcon setup also loads the underlays recorded when this workspace was built.
# Do not enable nounset: ROS setup scripts may reference unset variables.
source "$WORKSPACE/install/setup.bash"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：找不到 ros2，请先 source 车上 ROS 版本的 /opt/ros/<版本>/setup.bash。" >&2
  exit 1
fi

echo "工作区：$WORKSPACE"
echo "启动底盘、MID360s、Yesense 和定位；Nav2 请在另一终端运行 bash src/start_nav2.sh。"
echo "请先关闭手动启动的同类节点；can0、雷达网络及 IMU 串口沿用原车配置。"
echo "按 Ctrl+C 统一退出本次启动的驱动和定位。"
echo "当前默认雷达安装朝向为俯视向左 90°；恢复原安装时追加 lidar_mount_yaw:=0.0。"
# ---------- 自动日志记录 ----------
LOG_DIR="$WORKSPACE/logs/localization"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/localization_$(date '+%Y-%m-%d_%H-%M-%S').log"
echo "定位日志将自动写入：$LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1

# Launch from this source tree, avoiding stale copies of the new wrapper launch.
# exec lets ROS launch handle SIGINT and terminate its child processes.
exec ros2 launch "$SCRIPT_DIR/ml_bringup/launch/robot.launch.py" \
  map_directory:="$SCRIPT_DIR/ml_bringup/map" rviz:=false "$@"

# exec ros2 launch "$SCRIPT_DIR/ml_bringup/launch/robot.launch.py" \
#   map_directory:="/home/nvidia/loc/map_1012" rviz:=true "$@"
