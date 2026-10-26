cmake_minimum_required(VERSION 3.5)
project(ml_icp_localization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
add_subdirectory(third_party/fast_gicp)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(localization_core STATIC
  src/tracker.cpp
  src/scan_matcher.cpp
)
target_link_libraries(localization_core ml_fast_gicp ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(ml_icp_localization_node src/ml_icp_localization_node.cpp)
ament_target_dependencies(ml_icp_localization_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs pcl_conversions
  tf2_ros tf2
)
target_link_libraries(ml_icp_localization_node
  localization_core
  ${PCL_LIBRARIES}
)

install(TARGETS ml_icp_localization_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  third_party/fast_gicp/LICENSE
  third_party/fast_gicp/UPSTREAM.json
  third_party/fast_gicp/README.md
  DESTINATION share/${PROJECT_NAME}/third_party/fast_gicp)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
