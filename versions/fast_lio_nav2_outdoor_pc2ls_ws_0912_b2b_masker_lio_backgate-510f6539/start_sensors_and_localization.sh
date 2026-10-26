#!/usr/bin/env bash
# ==============================================================================
# 传感器与定位系统一键启动脚本
# 工作空间: Fast_LIO_Nav2_outdoor_pc2ls_ws_0912_20_b2b_masker_lio
# 
# 整合命令:
#   1. ros2 launch chassis_driver chassis_driver.launch.py
#   2. ros2 launch livox_ros_driver2 msg_MID360s_launch.py
#   3. ros2 launch lio_sam relocalization.launch.py
# ==============================================================================

# 进入当前工作空间根目录
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR" || exit 1

# 加载 ROS 2 及本地编译环境
source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[ERROR] 未找到 install/setup.bash，请先编译工作空间: colcon build"
    exit 1
fi

# 检查/激活 can0（避免底盘驱动因 CAN 接口未启动而报错崩溃）
if ip link show can0 2>/dev/null | grep -q "state DOWN"; then
    echo "[INFO] 检测到 can0 处于 DOWN 状态，尝试激活..."
    sudo ip link set can0 up type can bitrate 500000 2>/dev/null || true
fi

# 维护子进程 PID 列表
declare -a PIDS=()

# 退出清理函数 (捕获 Ctrl+C 时一键杀死这三个节点及其子进程)
cleanup() {
    echo -e "\n=========================================="
    echo "正在停止传感器与定位系统所有节点..."
    echo "=========================================="
    
    # 优雅发送中断信号
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    sleep 1.5

    # 兜底清理残留，杜绝端口和 CAN 占用
    pkill -f "chassis_driver" 2>/dev/null || true
    pkill -f "livox_ros_driver2" 2>/dev/null || true
    pkill -f "lio_sam" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    
    echo "所有节点已安全退出。"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "=================================================="
echo " [1/3] 启动底盘驱动 (chassis_driver)"
echo "=================================================="
ros2 launch chassis_driver chassis_driver.launch.py &
PID_CHASSIS=$!
PIDS+=($PID_CHASSIS)
sleep 1

echo "=================================================="
echo " [2/3] 启动 Livox Mid-360 激光雷达驱动"
echo "=================================================="
ros2 launch livox_ros_driver2 msg_MID360s_launch.py &
PID_LIDAR=$!
PIDS+=($PID_LIDAR)
sleep 1

echo "=================================================="
echo " [3/3] 启动 LIO-SAM 3D 重定位系统"
echo "=================================================="

# ------------------------------------------------------------------------------
# 智能日志文件名生成 (解决工控机无 RTC / 未联网导致系统时间为 1970 的问题)
# ------------------------------------------------------------------------------
if [ -n "$1" ]; then
    # 方式 1: 用户命令行手动传入后缀，如: ./start_sensors_and_localization.sh 09231005
    TIME_TAG="$1"
    LOG_FILE="$WS_DIR/localization.log${TIME_TAG}"
else
    SYS_YEAR=$(date +%Y 2>/dev/null || echo "1970")
    if [ "$SYS_YEAR" -ge 2024 ] 2>/dev/null; then
        # 方式 2: 系统时间有效（年份 >= 2024），采用 MMDDHHMM 格式 (如 localization.log09231005)
        TIME_TAG=$(date +%m%d%H%M)
        LOG_FILE="$WS_DIR/localization.log${TIME_TAG}"
        # 若同一分钟内重复运行，自动追加秒数避免覆盖
        if [ -f "$LOG_FILE" ]; then
            TIME_TAG=$(date +%m%d%H%M%S)
            LOG_FILE="$WS_DIR/localization.log${TIME_TAG}"
        fi
    else
        # 方式 3: 系统时间无效 (无 RTC/未联网，年份为 1970)，采用自动递增序列号 + 开机时间
        SEQ_FILE="$WS_DIR/.log_seq"
        if [ -f "$SEQ_FILE" ]; then
            SEQ=$(cat "$SEQ_FILE" 2>/dev/null || echo 0)
            SEQ=$((SEQ + 1))
        else
            MAX_NUM=$(ls -1 "$WS_DIR"/localization.log_seq* 2>/dev/null | grep -oE '[0-9]+' | tail -1 || echo 0)
            SEQ=$((MAX_NUM + 1))
        fi
        echo "$SEQ" > "$SEQ_FILE"
        
        # 开机运行秒数 (uptime)，即使同一次开机多次运行也绝不重复
        UPTIME_SEC=$(awk '{print int($1)}' /proc/uptime 2>/dev/null || echo "$$")
        SEQ_STR=$(printf "%03d" "$SEQ")
        LOG_FILE="$WS_DIR/localization.log_seq${SEQ_STR}_up${UPTIME_SEC}s"
    fi
fi

echo "[INFO] 定位日志将实时输出至: $LOG_FILE"

# 注: 因为步骤 2 中已单独启动雷达驱动，这里传入 use_lidar_driver:=false 避免重复拉起雷达驱动
# 使用 tee 同步实时输出到终端屏幕并持久化记录到文件
ros2 launch lio_sam relocalization.launch.py use_lidar_driver:=false 2>&1 | tee "$LOG_FILE" &
PID_LOC=$!
PIDS+=($PID_LOC)

echo ""
echo "=================================================="
echo " 所有传感器与定位节点已拉起！"
echo " 定位日志正在记录中: $LOG_FILE"
echo " 提示: 在本终端按 [Ctrl + C] 可一键安全退出所有节点。"
echo "=================================================="

# 等待定位进程
wait $PID_LOC
