cmake_minimum_required(VERSION 3.5)
project(sc_pgo)

# colcon 默认不设构建类型 → -O0；显式默认 Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# ---- ROS2 ament dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ml_scan_context REQUIRED)

# ---- Third-party ----
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED) # Scancontext.h 依赖
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# =============================================
# alaserPGO:位姿图优化后端(距离回环 + GICP 验证 + 地面 z 约束 + GPS XY factor)
# =============================================
add_executable(alaserPGO
  src/laserPosegraphOptimization.cpp
)
target_compile_options(alaserPGO PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(alaserPGO
  rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions tf2 tf2_ros ml_scan_context
)
target_link_libraries(alaserPGO
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

install(TARGETS
  alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/alaserRelocalizer DESTINATION lib/${PROJECT_NAME})

ament_package()
