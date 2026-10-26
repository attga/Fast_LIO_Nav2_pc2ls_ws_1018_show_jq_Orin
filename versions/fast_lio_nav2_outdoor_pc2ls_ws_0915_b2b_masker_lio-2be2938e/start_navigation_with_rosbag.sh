#!/bin/bash

# ==============================================================================
# LIO-SAM + Nav2 进阶记录脚本 (顺序启动防锁死版)
# 自动启动: Livox Mid360 雷达驱动 + LIO-SAM 重定位 + Nav2 导航 + ROSBAG 录制
# ==============================================================================

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_DIR="bags/obstacle_debug_bag_${TIMESTAMP}"
LOG_DIR="logs"
mkdir -p "${LOG_DIR}"
mkdir -p "bags"

LOC_LOG="${LOG_DIR}/localization_${TIMESTAMP}.log"
NAV_LOG="${LOG_DIR}/nav2_${TIMESTAMP}.log"
BAG_LOG="${LOG_DIR}/rosbag_${TIMESTAMP}.log"

echo "======================================================================"
echo "  🚀 启动 LIO-SAM + Nav2 (分屏实时窗口 + 全量调试 ROSBAG 录制)"
echo "  📌 1. 定位日志: ${SCRIPT_DIR}/${LOC_LOG}"
echo "  📌 2. 导航日志: ${SCRIPT_DIR}/${NAV_LOG}"
echo "  📦 3. ROSBAG 包: ${SCRIPT_DIR}/${BAG_DIR}"
echo "======================================================================"

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "❌ 错误: 未找到 install/setup.bash，请先运行 colcon build 编译工作空间！"
    exit 1
fi

cleanup() {
    # 1. 忽略后续的终端信号，防止 kill 0 引起死循环
    trap '' SIGINT SIGTERM EXIT
    echo -e "\n🛑 正在停止所有导航、定位与 Rosbag 录制节点..."
    
    # 2. 将整个 shell 的标准错误输出重定向到空，屏蔽掉 Bash 自动打印的又臭又长的 "Terminated" 话题进程文本
    exec 2>/dev/null
    
    # 3. 安全杀死整个进程组（确保没有残留僵尸节点）
    kill 0
    wait
    echo "✅ 日志与 ROSBAG 数据包已安全保存！"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# 1. 优先启动定位节点 (包含 Livox Mid360 雷达驱动自动调起)
# 直接重定向到独立日志文件，彻底杜绝 bash 管道 process substitution (tee) 缓冲满导致 C++ write() 卡死
echo "  📡 [1/3] 正在启动 LIO-SAM 重定位与雷达驱动..."
ros2 launch lio_sam relocalization.launch.py use_rviz:=false > "${LOC_LOG}" 2>&1 &
LOC_PID=$!

echo "  ⏳ [1/3] 正在等待 LIO-SAM 发布 odom->base_link TF 坐标系 (最长等待 60 秒)..."
# 核心原则：等待 TF 树中真正出现 odom 坐标系，而不是等待话题数据！
# odom TF 只有在 LIO-SAM 完成第一帧 scan-match 后才会发布，是最可靠的就绪信号。
MAX_WAIT=60
WAIT_COUNT=0
until timeout 2 ros2 run tf2_ros tf2_echo odom base_link >/dev/null 2>&1 || [ $WAIT_COUNT -ge $MAX_WAIT ]; do
    WAIT_COUNT=$((WAIT_COUNT + 2))
    echo "  ⏳ 等待 odom TF 就绪... (${WAIT_COUNT}s/${MAX_WAIT}s)"
    sleep 0.5
done

if [ $WAIT_COUNT -ge $MAX_WAIT ]; then
    echo "⚠️ 警告: LIO-SAM odom TF 初始化超时 (${MAX_WAIT}s)，请检查激光雷达数据流！"
else
    echo "✅ odom->base_link TF 已就绪！LIO-SAM 首帧 scan-match 完成，可以安全启动 Nav2！"
fi

# 2. 启动 Nav2 导航节点
echo "  🚀 [2/3] 正在启动 Nav2 导航节点..."
ros2 launch my_nav2_config nav2_bringup.launch.py map:=src/my_nav2_config/maps/lio_sam_map/map_edited_cleaned.yaml mask:=src/my_nav2_config/maps/lio_sam_map/keepout_mask.yaml > "${NAV_LOG}" 2>&1 &
NAV_PID=$!

sleep 3

# 3. 定位与导航均就绪后，启动后台 ROSBAG 录制
echo "  🎥 [3/3] 正在启动 ROSBAG 调试数据包录制..."
ros2 bag record \
  /tf \
  /tf_static \
  /local_lidar_odom \
  /lio_sam/mapping/odometry \
  /Odometry \
  /cloud_registered \
  /cloud_registered_body \
  /scan \
  /livox/imu \
  /local_costmap/costmap \
  /local_costmap/costmap_updates \
  /local_costmap/published_footprint \
  /global_costmap/costmap \
  /global_costmap/costmap_updates \
  /global_costmap/published_footprint \
  /filter_mask \
  /plan \
  /local_plan \
  /transformed_global_plan \
  /cmd_vel \
  /cmd_vel_nav \
  /lidar_odom \
  -o "${BAG_DIR}" > "${BAG_LOG}" 2>&1 &
BAG_PID=$!

# 4. 自动弹出 2 个独立分屏终端实时查看日志
if command -v gnome-terminal >/dev/null 2>&1; then
    echo "  📺 正在自动拉起 2 个独立分屏终端显示日志..."
    gnome-terminal --title="📍 定位实时日志 (LIO-SAM + 雷达驱动)" -- bash -c "tail -f '${SCRIPT_DIR}/${LOC_LOG}'; exec bash" &
    gnome-terminal --title="🚀 导航实时日志 (Nav2 路径与代价地图)" -- bash -c "tail -f '${SCRIPT_DIR}/${NAV_LOG}'; exec bash" &
fi

# 5. 进程健康监控
(
    while true; do
        sleep 3
        if ! kill -0 $LOC_PID 2>/dev/null; then
            echo -e "\033[31;1m\n[⚠️ 严重警告] 定位节点组 (LIO-SAM) 异常崩溃退出！请检查: ${LOC_LOG}\033[0m"
            break
        fi
        if ! kill -0 $NAV_PID 2>/dev/null; then
            echo -e "\033[31;1m\n[⚠️ 严重警告] 导航节点组 (Nav2) 异常崩溃退出！请检查: ${NAV_LOG}\033[0m"
            break
        fi
    done
) &

echo "======================================================================"
echo "  💡 系统已全量启动。主终端正在实时输出定位与导航汇总日志。"
echo "  💡 按 Ctrl+C 可一键安全退出所有节点并保存 ROSBAG！"
echo "======================================================================"

tail -f "${LOC_LOG}" "${NAV_LOG}"
