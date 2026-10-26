> 工作区维护说明：重定位实现已迁至 [`ml_relocalization`](../ml_relocalization/README.md)。
> 共用 Scan Context 位于 `ml_scan_context`；本包保留建图节点与旧重定位命令转发入口。

## 点云与位姿输入

同一前端的点云与里程计必须使用相同传感器时间戳（整数纳秒严格匹配）。
不插值，不把未来位姿配给旧扫描。不同话题到达先后可不同，单话题必须严格递增。
重复和倒序消息丢弃；回放跳回或重新从头播放时应重启建图节点。
`input_queue_size` 默认 100，为每路缓存上限；溢出丢弃最旧消息。
当两路队首时间不同，丢弃较早者，避免静默错配。单路停止时不会无限增长。
`[input_sync]` 输出累计匹配、未配对、溢出、重复/倒序与退出时待配对数量；
单路不再到来时，少量未配对消息可能保留到退出，不使用墙钟超时推断传感器数据过期。

## 新关键帧初值

首帧使用前端位姿；之后先用原始前端位姿计算相邻关键帧相对运动 ΔT，
再以“上一关键帧后端位姿 × ΔT”初始化新节点。
上一帧已经优化时使用优化结果，尚未优化时使用其当前递推初值。
原始前端位姿独立保存，里程计因子的测量仍为原始 ΔT；不混入后端修正。
关键帧插入按图锁、关键帧锁顺序执行，与优化更新互斥。

## 回环位姿快照

每次验证回环时，在关键帧锁内一次复制 source/target 所需位姿、两端位姿与扫描时间。
点云入库后不修改，快照保留只读共享引用，避免复制整份点云。
释放锁后构建子图、执行 GICP，并用同一快照计算历史帧到修正后当前帧的相对测量。
配准期间发生的新一轮图优化不会改变本次测量使用的几何。
候选搜索也使用一致的位姿/时间副本；日志中的 `pair=历史帧->当前帧` 标识实际配准对象。

## 当前 ROS2 建图保存方式

建图完成后，先停止输入或等 rosbag 播放结束，再在建图 launch 终端按一次 Ctrl+C。
节点停止工作线程、完成已加入因子的优化，统一保存 `isam_poses.txt`、
`odom_poses.txt`、`times.txt`、`Scans/*.pcd`、`sc_db.bin` 和 `map.pcd`。
等待 `[PGO] final map saved` 和进程退出；不再根据停车 30 秒自动保存。
强制结束、断电不会触发可靠保存。两个建图 launch 的退出等待时间为 120 秒，
超大地图如需更长保存时间，应相应增加等待时间。

`odom_rot_noise` 为旋转方差（rad²），`odom_trans_noise` 为平移方差（m²）。
默认分别为 `1e-6` 和 `1e-4`，保持修正前默认配置的实际权重。
旧自定义配置中 `odom_trans_noise` 曾错误控制旋转，请迁移到对应参数。
在线回环仍使用距离候选与 GICP；SC 数据库在最终保存时生成。

# SC-A-LOAM（上游历史说明）

以下内容保留上游来源与致谢，ROS1 命令不适用于当前工作区。当前构建及入口以根目录 README 和 docs/DEPLOYMENT.md 为准。

## What is SC-A-LOAM? 
- A real-time LiDAR SLAM package that integrates A-LOAM and ScanContext. 
    - **A-LOAM** for odometry (i.e., consecutive motion estimation)
    - **ScanContext** for coarse global localization that can deal with big drifts (i.e., place recognition as kidnapped robot problem without initial pose)
    - and iSAM2 of GTSAM is used for pose-graph optimization. 
- This package aims to show ScanContext's handy applicability. 
    - The only things a user should do is just to include `Scancontext.h`, call `makeAndSaveScancontextAndKeys` and `detectLoopClosureID`. 

## Features 
1.  A strong place recognition and loop closing 
    - We integrated ScanContext as a loop detector into A-LOAM, and ISAM2-based pose-graph optimization is followed. (see https://youtu.be/okML_zNadhY?t=313 to enjoy the drift-closing moment)
2. A modular implementation 
    - The only difference from A-LOAM is the addition of the `laserPosegraphOptimization.cpp` file. In the new file, we subscribe the point cloud topic and odometry topic (as a result of A-LOAM, published from `laserMapping.cpp`). That is, our implementation is generic to any front-end odometry methods. Thus, our pose-graph optimization module (i.e., `laserPosegraphOptimization.cpp`) can easily be integrated with any odometry algorithms such as non-LOAM family or even other sensors (e.g., visual odometry).  
    - <p align="center"><img src="picture/anypipe.png" width=800></p>
3. (optional) Altitude stabilization using consumer-level GPS  
    - To make a result more trustworthy, we supports GPS (consumer-level price, such as U-Blox EVK-7P)-based altitude stabilization. The LOAM family of methods are known to be susceptible to z-errors in outdoors. We used the robust loss for only the altitude term. For the details, see the variable `robustGPSNoise` in the `laserPosegraphOptimization.cpp` file. 

## Prerequisites (dependencies)
- We mainly depend on ROS, Ceres (for A-LOAM), and GTSAM (for pose-graph optimization). 
    - For the details to install the prerequisites, please follow the A-LOAM and LIO-SAM repositiory. 
- The below examples are done under ROS melodic (ubuntu 18) and GTSAM version 4.x. 

## How to use? 
- First, install the abovementioned dependencies, and follow below lines. 
```
    mkdir -p ~/catkin_scaloam_ws/src
    cd ~/catkin_scaloam_ws/src
    git clone https://github.com/gisbi-kim/SC-A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_scaloam_ws/devel/setup.bash
    roslaunch aloam_velodyne aloam_mulran.launch # for MulRan dataset setting 
```

## Example Results 

### Riverside 01, MulRan dataset 
- The MulRan dataset provides lidar scans (Ouster OS1-64, horizontally mounted, 10Hz) and consumer level gps (U-Blox EVK-7P, 4Hz) data.
    - About how to use (publishing data) data: see here https://github.com/irapkaist/file_player_mulran
- example videos on Riverside 01 sequence. 
    1. with consumer level GPS-based altitude stabilization: https://youtu.be/FwAVX5TVm04
    2. without the z stabilization: https://youtu.be/okML_zNadhY 
- example result:

<p align="center"><img src="picture/riverside01.png" width=800></p>

### KITTI 05 
- For KITTI (HDL-64 sensor), run using the command 
    ```
    roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch # for KITTI dataset setting
    ```
- To publish KITTI scans, you can use mini-kitti publisher, a simple python script: https://github.com/gisbi-kim/mini-kitti-publisher
- example video (no GPS used here): https://youtu.be/hk3Xx8SKkv4
- example result: 

<p align="center"><img src="picture/kitti05.png" width=800></p>

### Indoor
- ScanContext also works at indoor environments (use smaller sc_max_radius value).
- example video: https://youtu.be/Uv6_BRmxJho
- example result: 
<p align="center"><img src="picture/indoor.png" width=800></p>

### For Livox LiDAR 
- Scan Context also works for Livox LiDAR data
    - In this example, Scan Context is integrated with FAST-LIO (https://github.com/hku-mars/FAST_LIO).
    - Note: No additional integration effort is required. A user just run seperately FAST-LIO node and SC-A-LOAM's posegraphoptimization.cpp node!
- example video (tutoial and results): https://youtu.be/Fw9S6D6HozA
- example result: 
    <p align="center"><img src="picture/scfastlio.png" width=600></p>

### For Navtech Radar 
- Scan Context also works for Navtech Radar data!
- For the details, please see 
    - https://github.com/gisbi-kim/navtech-radar-slam
        - used the pose-graph optimization node of this repository (SC-A-LOAM)
    - [example video](https://www.youtube.com/watch?v=avtIQ8fesgU&t=128s)

## Utilities

### Data saver and Map construction 
- Similar to the [SC-LIO-SAM's saver utility](https://github.com/gisbi-kim/SC-LIO-SAM#applications), we support pose and scan saver per keyframes. Using these saved data, the map (within ROI) can be constructed offline. See the `utils/python/makeMergedMap.py` and [this tutorial](https://youtu.be/jmR3DH_A4Co). 
- Below is the example results of MulRan dataset KAIST 03's merged map, visualized using CloudCompare ([download the map data here](https://www.dropbox.com/sh/96jrpx3x6hh316j/AACb07kGbocnQWMIpksmU6MQa?dl=0)).  

    <p align="center"><img src="picture/kaist-03-merged.png" width=800></p>

- A user also can remove dynamic points using these saved keyframe poses and scans. See [this tutorial](https://www.youtube.com/watch?v=UiYYrPMcIRU) and our [Removert project](https://github.com/irapkaist/removert).

## Acknowledgements
- Thanks to LOAM, A-LOAM, and LIO-SAM code authors. The major codes in this repository are borrowed from their efforts.

## Maintainer 
- please contact me through `paulgkim@kaist.ac.kr` 

## TODO
- Delayed RS loop closings 
- SLAM with multi-session localization 
- More examples on other datasets (KITTI, complex urban dataset, etc.)
