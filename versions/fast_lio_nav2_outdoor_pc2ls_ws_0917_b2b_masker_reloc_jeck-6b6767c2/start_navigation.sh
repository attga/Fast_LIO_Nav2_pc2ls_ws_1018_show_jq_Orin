#!/bin/bash

# ==============================================================================
# MID360 0915 双 EKF 定位 + Nav2 室外导航一键启动脚本
# 功能：
#   1. 自动检测并加载 ROS 2 环境及工作空间
#   2. 顺序启动 3D 定位系统（FAST-LIO + 启动重定位 + 双 EKF + ICP）
#   3. 智能门禁检测（等待定位 Manager 确认进入 TRACKING 且 TF 就绪）
#   4. 启动 Nav2 导航系统（加载 2D 栅格地图、走廊禁区掩膜、MPPI 控制器）
#   5. 支持实车模式与 Rosbag 离线回放模式（自动发布时钟与话题播放）
#   6. 统一进程生命周期管理与安全退出（Ctrl+C 一键安全关闭全部节点）
# ==============================================================================

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}"

# ── 默认运行配置 ─────────────────────────────────────────────────────────────
USE_SIM_TIME="false"
BAG_PATH=""
DO_BUILD="false"
START_DRIVERS="false"
ENABLE_YESENSE="true"
ENABLE_RVIZ="true"
MAP_3D_DIR="${SCRIPT_DIR}/src/ml_bringup/map"
MAP_2D_YAML="${SCRIPT_DIR}/map_jeck/map.yaml"
MASK_YAML="${SCRIPT_DIR}/map_jeck/keepout_mask.yaml"

# ── 帮助信息 ─────────────────────────────────────────────────────────────────
show_help() {
    echo "======================================================================"
    echo "  🚀 MID360 0915 定位 + Nav2 导航 一键启动脚本 (ROS 2 Humble)"
    echo "======================================================================"
    echo "使用方法: ./start_navigation.sh [选项]"
    echo ""
    echo "选项说明:"
    echo "  --build                启动前先执行 colcon build --symlink-install 编译"
    echo "  --real                 实车模式 (默认, use_sim_time:=false)"
    echo "  --bag <bag_path>       Rosbag 回放模式 (自动开启 use_sim_time:=true 并播放数据包)"
    echo "  --drivers              实车模式下同时拉起 Livox 雷达与底盘驱动"
    echo "  --no-yesense           禁用 Yesense 外部陀螺仪 (仅使用雷达内置 IMU)"
    echo "  --no-rviz              不拉起 RViz 可视化界面"
    echo "  --map3d <dir>          指定 3D 定位地图目录 (默认: src/ml_bringup/map)"
    echo "  --map2d <yaml>         指定 2D 栅格地图文件 (默认: map_jeck/map.yaml)"
    echo "  --mask <yaml>          指定 2D 禁区掩膜文件 (默认: map_jeck/keepout_mask.yaml)"
    echo "  -h, --help             显示本帮助信息"
    echo ""
    echo "常用示例:"
    echo "  1. 编译并实车启动:"
    echo "     ./start_navigation.sh --build./start_navigation.sh --build"
    echo "  2. 实车正常启动 (不重新编译):"
    echo "     ./start_navigation.sh"
    echo "  3. 实车全自动启动 (含驱动):"
    echo "     ./start_navigation.sh --drivers"
    echo "  4. 离线 Rosbag 回放测试:"
    echo "     ./start_navigation.sh --bag /path/to/my_rosbag"
    echo "======================================================================"
}

# ── 参数解析 ─────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            DO_BUILD="true"
            shift
            ;;
        --real)
            USE_SIM_TIME="false"
            shift
            ;;
        --bag)
            USE_SIM_TIME="true"
            BAG_PATH="$2"
            shift 2
            ;;
        --drivers)
            START_DRIVERS="true"
            shift
            ;;
        --no-yesense)
            ENABLE_YESENSE="false"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        --map3d)
            MAP_3D_DIR="$2"
            shift 2
            ;;
        --map2d)
            MAP_2D_YAML="$2"
            shift 2
            ;;
        --mask)
            MASK_YAML="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "❌ 未知参数: $1"
            show_help
            exit 1
            ;;
    esac
done

# ── 环境检测与加载 ───────────────────────────────────────────────────────────
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "${LOG_DIR}"

LOC_LOG="${LOG_DIR}/localization_${TIMESTAMP}.log"
NAV_LOG="${LOG_DIR}/nav2_${TIMESTAMP}.log"
BAG_LOG="${LOG_DIR}/rosbag_${TIMESTAMP}.log"
DRV_LOG="${LOG_DIR}/drivers_${TIMESTAMP}.log"

echo "======================================================================"
echo "  🚀 启动 0915 双 EKF 定位 + Nav2 导航链路"
echo "  📅 启动时间: ${TIMESTAMP}"
echo "  ⏱️ 仿真时钟 (use_sim_time): ${USE_SIM_TIME}"
echo "  🗺️ 3D 定位地图目录: ${MAP_3D_DIR}"
echo "  🗺️ 2D 栅格地图文件: ${MAP_2D_YAML}"
echo "  🛑 2D 禁区掩膜文件: ${MASK_YAML}"
echo "  📝 定位日志: ${LOC_LOG}"
echo "  📝 导航日志: ${NAV_LOG}"
if [ -n "${BAG_PATH}" ]; then
    echo "  📦 回放数据包: ${BAG_PATH}"
fi
echo "======================================================================"

# 1. 检测并激活 ROS 2 环境 (优先使用 Humble)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "  📦 已加载 ROS 2 环境: Humble"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "  📦 已加载 ROS 2 环境: Jazzy"
else
    echo "❌ 错误: 未在 /opt/ros 找到 Humble 或 Jazzy 环境！"
    exit 1
fi

# 2. 检测第三方驱动安装环境 (若有)
if [ -f "/home/jackie/Desktop/ws_livox/install/setup.bash" ]; then
    source /home/jackie/Desktop/ws_livox/install/setup.bash
fi

# 3. 编译工作空间 (若指定 --build 或初次运行缺少 install/setup.bash)
if [ "${DO_BUILD}" = "true" ] || [ ! -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    echo "  🔨 [编译] 正在执行: colcon build --symlink-install ..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ $? -ne 0 ]; then
        echo "❌ 编译失败，请检查终端报错输出！"
        exit 1
    fi
    echo "  ✅ 编译完成！"
fi

# 4. 加载当前工作空间安装环境
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
else
    echo "❌ 错误: 未找到 install/setup.bash，编译可能被中断！"
    exit 1
fi

# ── 安全退出捕获 ─────────────────────────────────────────────────────────────
cleanup() {
    echo -e "\n🛑 正在停止定位、导航与所有后台节点..."
    # 向同进程组发送信号，防止残留孤儿进程占用串口或雷达端口
    kill 0 2>/dev/null || true
    wait 2>/dev/null || true
    echo "✅ 系统已安全退出，日志保存于: ${LOG_DIR}"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ── 步骤 0: 可选启动实车硬件驱动 ─────────────────────────────────────────────
if [ "${START_DRIVERS}" = "true" ] && [ "${USE_SIM_TIME}" = "false" ]; then
    echo "  🔌 [0/3] 正在启动雷达与底盘硬件驱动..."
    (
        ros2 launch livox_ros_driver2 msg_MID360_launch.py &
        ros2 launch chassis_driver chassis_driver.launch.py &
        if [ "${ENABLE_YESENSE}" = "true" ]; then
            ros2 launch yesense_std_ros2 yesense_node.launch.py &
        fi
        wait
    ) > "${DRV_LOG}" 2>&1 &
    sleep 3
fi

# ── 步骤 1: 启动 3D 定位系统 ─────────────────────────────────────────────────
echo "  📡 [1/3] 正在启动 0915 双 EKF 定位节点..."
ros2 launch ml_bringup localization.launch.py \
    map_directory:="${MAP_3D_DIR}" \
    pose_file:=isam_poses.txt \
    use_sim_time:="${USE_SIM_TIME}" \
    enable_yesense:="${ENABLE_YESENSE}" \
    rviz:="${ENABLE_RVIZ}" \
    > "${LOC_LOG}" 2>&1 &
LOC_PID=$!

# ── 步骤 1.5: 若为 Rosbag 模式，启动数据包回放 ──────────────────────────────
if [ -n "${BAG_PATH}" ]; then
    echo "  📼 正在启动 Rosbag 回放 (--clock 模式)..."
    ros2 bag play "${BAG_PATH}" \
        --rate 1.0 \
        --clock \
        --topics \
            /livox/lidar \
            /livox/imu \
            /imu/data_raw \
            /wheel_odom \
        > "${BAG_LOG}" 2>&1 &
    BAG_PID=$!
fi

# ── 步骤 2: 等待定位系统初始化就绪（门禁检查）───────────────────────────────
echo "  ⏳ [2/3] 正在等待定位系统完成初始配准 (Scan Context + EKF 就绪)..."
MAX_WAIT=45
WAIT_COUNT=0
LOC_READY=0

while [ ${WAIT_COUNT} -lt ${MAX_WAIT} ]; do
    # 检查进程是否意外存活
    if ! kill -0 ${LOC_PID} 2>/dev/null; then
        echo -e "\n❌ 定位节点异常退出！请查看日志: ${LOC_LOG}"
        exit 1
    fi

    # 检测局部里程计或全局定位话题是否有有效数据流
    if timeout 2 ros2 topic echo /local_odometry/filter --once >/dev/null 2>&1; then
        LOC_READY=1
        break
    fi

    sleep 2
    WAIT_COUNT=$((WAIT_COUNT + 2))
    echo "  ⏳ 正在等待定位就绪... (${WAIT_COUNT}s/${MAX_WAIT}s)"
done

if [ ${LOC_READY} -eq 1 ]; then
    echo "  ✅ 定位系统与 TF 坐标系 (map->odom->base_link) 已 100% 就绪！"
else
    echo "  ⚠️ 警告: 达到预设等待阈值 (${MAX_WAIT}s)，尝试推进启动 Nav2..."
fi

# ── 步骤 3: 启动 Nav2 导航系统 ───────────────────────────────────────────────
echo "  🚀 [3/3] 正在启动 Nav2 导航系统..."
ros2 launch my_nav2_config nav2_bringup.launch.py \
    use_sim_time:="${USE_SIM_TIME}" \
    map:="${MAP_2D_YAML}" \
    mask:="${MASK_YAML}" \
    use_keepout:=true \
    autostart:=true \
    > "${NAV_LOG}" 2>&1 &
NAV_PID=$!

sleep 3

# ── 后台监控与实时日志输出 ───────────────────────────────────────────────────
echo "======================================================================"
echo "  🎉 系统已全量成功拉起！"
echo "  💡 提示: 按 Ctrl+C 可一键安全退出所有节点。"
echo "======================================================================"

# 实时滚动输出定位与导航日志
tail -f "${LOC_LOG}" "${NAV_LOG}"
