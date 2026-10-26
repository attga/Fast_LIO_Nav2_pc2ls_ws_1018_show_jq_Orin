# FAST_LIO_SLAM (Mid-360) — Orin NX 编译与建图完整实战手册

**适用硬件**：NVIDIA Jetson Orin NX 16GB  
**激光雷达**：Livox Mid-360  
**操作系统**：JetPack 5.x (Ubuntu 20.04 + ROS Noetic)  
**建图算法**：FAST-LIO2 (前端里程计) + SC-PGO (ScanContext 回环 + GTSAM 位姿图优化)

---

## 第一步：拷贝源码至 Orin NX

在您的 Windows 电脑上，通过 SCP 或 U 盘将整个工程目录传输至 Orin NX：

```bash
# 在 Windows 端用 SCP 推送（替换 <orin_ip> 为实际 IP）
scp -r "D:\Users\Desktop\carto_nav2_ws_06_comm_yamlmap\src\FAST_LIO_SLAM" \
    nvidia@<orin_ip>:~/catkin_ws/src/
```

或直接将 U 盘插入 Orin NX 后手动拷贝到 `~/catkin_ws/src/`。

---

## 第二步：Orin NX 系统预检与拉满算力

**每次开机运行建图前，必须先执行以下两条命令，锁死 CPU/GPU 最高频率：**

```bash
# 切换至最高功耗模式 (MAXN模式，8核全开)
sudo nvpmodel -m 0

# 锁死 CPU/GPU 最高频率，防止热降频导致点云处理卡顿
sudo jetson_clocks
```

验证当前功耗模式（应显示 "NV Power Mode: MAXN"）：
```bash
sudo nvpmodel -q
```

---

## 第三步：安装系统级依赖库

```bash
# 1. 安装 GTSAM (SC-PGO 后端因子图优化核心库)
sudo apt-get update
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev

# 2. 安装 PCL (点云库)
sudo apt-get install -y libpcl-dev

# 3. 安装 Eigen3 (矩阵运算，FAST-LIO2 核心依赖)
sudo apt-get install -y libeigen3-dev

# 4. 安装 OpenMP (并行加速，ikd-Tree 依赖)
sudo apt-get install -y libomp-dev
```

---

## 第四步：安装 Livox Mid-360 ROS 驱动

```bash
cd ~/catkin_ws/src

# 拉取 livox_ros_driver2 (支持 Mid-360 CustomMsg 格式)
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 安装 Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j4
sudo make install
cd ~/catkin_ws
```

---

## 第五步：修改 Mid-360 网络配置（连接雷达必须）

Mid-360 通过以太网 UDP 广播点云数据，需要在 Orin NX 上固定 IP：

```bash
# 将连接雷达的网卡 IP 固定为 192.168.1.50 (Mid-360 默认广播至此网段)
sudo ip addr add 192.168.1.50/24 dev eth0   # eth0 替换为实际网卡名

# 设置 MTU 为 1500，防止大数据包拆包导致点云丢帧
sudo ip link set eth0 mtu 1500

# 验证连通性 (Mid-360 默认 IP 为 192.168.1.1xx)
ping 192.168.1.100
```

在 `livox_ros_driver2` 配置文件中，填入您 Mid-360 的序列号：
```bash
# 编辑驱动配置文件
nano ~/catkin_ws/src/livox_ros_driver2/config/MID360_config.json
# 将 "lidar_configs" -> "ip" 字段改为您的 Mid-360 实际 IP
```

---

## 第六步：编译整个工作空间

```bash
cd ~/catkin_ws

# 初始化 catkin 工作空间（如尚未初始化）
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

# 完整编译（-j4 限制并行数，防止 Orin NX 内存溢出）
catkin_make -j4

# Source 环境变量
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

> ⚠️ **编译常见报错与解决方案**：
> - 报 `gtsam not found`：重新执行第三步 GTSAM 安装
> - 报 `livox_ros_driver2 not found`：确保 `livox_ros_driver2` 已在同一 `catkin_ws/src` 下
> - 报 `Eigen version`：执行 `sudo apt install libeigen3-dev` 后重新 catkin_make

---

## 第七步：开始建图（三终端并行启动）

建图需要同时运行 **3 个终端** 执行以下节点：

### 终端 1：启动 Mid-360 激光雷达驱动
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 终端 2：启动 FAST-LIO2 前端里程计与建图
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

### 终端 3：启动 SC-PGO 回环检测与位姿图优化
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch aloam_velodyne fastlio_mid360.launch
```

---

## 第八步：保存最终优化后的 PCD 地图

建图完成后，向 SC-PGO 节点发送保存指令：

```bash
# 在新终端中发布保存信号
rostopic pub /save_map std_msgs/Empty "{}" --once
```

地图将自动保存至 `fastlio_mid360.launch` 中配置的路径：
```
~/slam_maps/mid360/
    ├── OriginCloud.pcd         # 未优化原始点云
    ├── GlobalMap.pcd           # SC-PGO 回环优化后全局地图 ★ 这是您要的最终地图
    └── trajectory.txt          # 位姿轨迹文件
```

---

## 建图操作规约（防止地图漂移与重影）

| 规约 | 说明 |
| :--- | :--- |
| **遥控速度 ≤ 0.8 m/s** | 速度过快时 ikd-Tree 点云匹配跟不上，前端漂移加剧 |
| **8字形/网格化扫图** | 每前进 300~500 米主动绕回旧路触发回环，禁止只绕一圈不回头 |
| **切掉 3.5m 以上点云** | 在 mid360.yaml 中可加点云高度裁切，防止摇摆树冠污染地图 |
| **首次建图环境要遮挡丰富** | 玻璃幕墙和纯空旷草坪区域极易造成几何退化，如遇此类区域减速至 0.3m/s |
| **建图时不运行 Nav2** | 建图期间关闭 Nav2 及所有深度相机话题，减少 Orin NX 内存压力 |

---

## 快速验证清单（拷贝后第一次启动检查）

- [ ] `rostopic echo /livox/lidar` 能收到点云数据
- [ ] `rostopic echo /livox/imu` 能收到 200Hz IMU 数据  
- [ ] RViz 中能看到 `/cloud_registered` 实时点云地图在生长
- [ ] RViz 中 `/trajectory_path` 轨迹线流畅无跳变
- [ ] 终端 3 的 SC-PGO 日志中，驾车绕回原地后出现 `Loop detected!` 字样即回环成功
