#!/usr/bin/env bash
# Run separately, after localization is ready: bash src/start_nav2.sh
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE"

if [[ ! -r "$WORKSPACE/install/setup.bash" ]]; then
  echo "错误：缺少 $WORKSPACE/install/setup.bash，请先在车上完成 colcon build。" >&2
  exit 1
fi
source "$WORKSPACE/install/setup.bash"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：找不到 ros2，请先 source 车上 ROS 版本的 /opt/ros/<版本>/setup.bash。" >&2
  exit 1
fi
for file in map.yaml map.pgm keepout_mask.yaml keepout_mask.pgm; do
  if [[ ! -r "$WORKSPACE/map_jeck/$file" ]]; then
    echo "错误：缺少或无法读取 $WORKSPACE/map_jeck/$file，请完整复制 map_jeck。" >&2
    exit 1
  fi
done

echo "工作区：$WORKSPACE"
echo "仅启动 Nav2（含点云转扫描和禁行区）；请先确认定位 startup_complete=1。"
echo "本脚本不会发送导航目标或启动航点巡航；按 Ctrl+C 退出 Nav2。"
exec ros2 launch "$SCRIPT_DIR/my_nav2_config/launch/nav2_bringup.launch.py" \
  map:="$WORKSPACE/map_jeck/map.yaml" \
  mask:="$WORKSPACE/map_jeck/keepout_mask.yaml" \
  use_sim_time:=false use_keepout:=true
