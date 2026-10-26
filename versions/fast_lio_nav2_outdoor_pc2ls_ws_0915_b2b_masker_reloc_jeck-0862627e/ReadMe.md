1847  ros2 bag record -o obstacle_debug_bag091701 /tf /tf_static /Odometry /cloud_registered_body /scan /local_costmap/costmap /local_costmap/published_footprint /plan /local_plan /cmd_vel_nav /cmd_vel /global_costmap/costmap /global_costmap/published_footprint /trajectory /livox/imu
 1848  ros2 run teleop_twist_keyboard teleop_twist_keyboard 
 1849  cd src/my_nav2_config/scripts/
 1850  python3 waypoint_through_poses_buinding2building.py 
 1851  source install/setup.bash 
 1852  ros2 launch my_nav2_config nav2_bringup.launch.py 2>&1 | tee nav2_0917_1356.log
 1855  source install/setup.bash 
 1856  ros2 launch ml_bringup localization.launch.py   map_directory:=/home/nvidia/Fast_LIO_Nav2_outdoor_pc2ls_ws_0915_b2b_masker_reloc_jeck/src/ml_bringup/map pose_file:=isam_poses.txt   use_sim_time:=false   enable_yesense:=true   rviz:=false   2>&1 | tee "20260917_1356.log"
 1857  source install/setup.bash 
 1858  ros2 launch livox_ros_driver2 msg_MID360s_launch.py 
 1859  source install/setup.bash 
 1860  ros2 run teleop_twist_keyboard teleop_twist_keyboard 
 1861  source install/setup.bash 
 1862  ros2 launch yesense_std_ros2 yesense_node.launch.py
 1863  source install/setup.bash 
 1864  ros2 launch chassis_driver chassis_driver.launch.py 