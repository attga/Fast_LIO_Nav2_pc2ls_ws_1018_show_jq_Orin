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
for file in map.yaml map.pgm; do
  if [[ ! -r "$WORKSPACE/map_jeck/$file" ]]; then
    echo "错误：缺少或无法读取 $WORKSPACE/map_jeck/$file，请完整复制 map_jeck。" >&2
    exit 1
  fi
done
KEEP_OUT="${USE_KEEPOUT:-auto}"
if [[ "$KEEP_OUT" == auto ]]; then
  if [[ -r "$WORKSPACE/map_jeck/keepout_mask.yaml" && -r "$WORKSPACE/map_jeck/keepout_mask.pgm" ]]; then
    KEEP_OUT=true
  else
    KEEP_OUT=false
  fi
fi
if [[ "$KEEP_OUT" != true && "$KEEP_OUT" != false ]]; then
  echo "错误：USE_KEEPOUT 只能是 auto、true 或 false。" >&2
  exit 1
fi
if [[ "$KEEP_OUT" == true ]]; then
  for file in keepout_mask.yaml keepout_mask.pgm; do
    if [[ ! -r "$WORKSPACE/map_jeck/$file" ]]; then
      echo "错误：启用禁行区需要 $WORKSPACE/map_jeck/$file。" >&2
      exit 1
    fi
  done
fi

echo "工作区：$WORKSPACE"
echo "仅启动 Nav2（点云转扫描；禁行区=$KEEP_OUT）；请先确认定位 startup_complete=1。"
echo "本脚本不会发送导航目标或启动航点巡航；按 Ctrl+C 退出 Nav2。"
# ---------- 自动日志记录 ----------
LOG_DIR="$WORKSPACE/logs/nav2"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/nav2_$(date '+%Y-%m-%d_%H-%M-%S').log"
PARAMS_FILE="$SCRIPT_DIR/my_nav2_config/config/nav2_params.yaml"
if [[ "$KEEP_OUT" == false ]]; then
  PARAMS_FILE="$LOG_DIR/nav2_params_without_keepout_$$.yaml"
  python3 - "$SCRIPT_DIR/my_nav2_config/config/nav2_params.yaml" "$PARAMS_FILE" <<'PYCONFIG'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    params = yaml.safe_load(stream)
for name in ('local_costmap', 'global_costmap'):
    # ROS 2 Humble treats an empty YAML sequence as an unset parameter.
    # Omit the key so the costmap uses its default (no filters).
    params[name][name]['ros__parameters'].pop('filters', None)
with open(sys.argv[2], 'w', encoding='utf-8') as stream:
    yaml.safe_dump(params, stream, allow_unicode=True, sort_keys=False)
PYCONFIG
fi
echo "Nav2 日志将自动写入：$LOG_FILE"
exec > >(tee -a "$LOG_FILE") 2>&1

exec ros2 launch "$SCRIPT_DIR/my_nav2_config/launch/nav2_bringup.launch.py" \
  params_file:="$PARAMS_FILE" \
  map:="$WORKSPACE/map_jeck/map.yaml" \
  mask:="$WORKSPACE/map_jeck/keepout_mask.yaml" \
  use_sim_time:=false use_keepout:="$KEEP_OUT" "$@"
