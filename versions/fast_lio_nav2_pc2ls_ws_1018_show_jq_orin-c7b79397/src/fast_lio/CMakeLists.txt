cmake_minimum_required(VERSION 3.5)
project(fast_lio)

# colcon 默认不设构建类型 → -O0 调试构建 → rate 2 回放实时性不足（曾致 z 轴爆炸）
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Default to C++14 for FAST-LIO (original standard)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenMP detection for multi-core processing
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  if(N GREATER 6)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=6)  
    message("core for MP: 6")
  elseif(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP: 4")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)

# ---- ROS2 ament dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) # laserMapping.cpp 发布 Marker(原 package.xml 漏声明,Foxy 靠传递依赖侥幸通过)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# ---- ROS-agnostic third-party dependencies ----
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Python3 REQUIRED COMPONENTS Development)

# ---- Custom message generation (replaces add_message_files + generate_messages) ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES geometry_msgs
)

# ---- Executable ----
add_executable(fastlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
)

# Include directories
target_include_directories(fastlio_mapping PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_include_directories(fastlio_mapping SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

# Compile definitions
target_compile_definitions(fastlio_mapping PRIVATE
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)

# ROS2 dependencies via ament_target_dependencies
ament_target_dependencies(fastlio_mapping
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_ros_driver2
)

# Non-ament libraries
target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  ${Python3_LIBRARIES}
)

# Link custom message typesupport (Foxy uses rosidl_target_interfaces)
rosidl_target_interfaces(fastlio_mapping
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio_mapping OpenMP::OpenMP_CXX)
endif()

# ---- Install rules ----
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(body_output_test test/body_output_test.cpp)
  target_include_directories(body_output_test PRIVATE include)
  target_link_libraries(body_output_test Eigen3::Eigen)
  ament_add_gtest(imu_processing_test test/imu_processing_test.cpp)
  target_include_directories(imu_processing_test PRIVATE include src ${PCL_INCLUDE_DIRS})
  target_compile_definitions(imu_processing_test PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
  ament_target_dependencies(imu_processing_test rclcpp sensor_msgs nav_msgs geometry_msgs pcl_conversions)
  target_link_libraries(imu_processing_test Eigen3::Eigen ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
  rosidl_target_interfaces(imu_processing_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()
