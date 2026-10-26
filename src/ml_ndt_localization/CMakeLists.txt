cmake_minimum_required(VERSION 3.5)
project(ml_ndt_localization)

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
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
add_subdirectory(third_party/autoware_ndt)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(localization_core STATIC
  src/tracker.cpp
  src/ndt_matcher.cpp
)
target_link_libraries(localization_core ml_autoware_ndt ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(ml_ndt_localization_node
  src/node.cpp
  src/node_parameters.cpp
)
ament_target_dependencies(ml_ndt_localization_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs diagnostic_msgs std_srvs pcl_conversions
  tf2_ros tf2
)
target_link_libraries(ml_ndt_localization_node
  localization_core
  ${PCL_LIBRARIES}
)

install(TARGETS ml_ndt_localization_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  third_party/autoware_ndt/LICENSE
  third_party/autoware_ndt/LICENSE.Apache-2.0
  third_party/autoware_ndt/NOTICE
  third_party/autoware_ndt/UPSTREAM.json
  third_party/autoware_ndt/README.md
  third_party/autoware_ndt/README.upstream.md
  DESTINATION share/${PROJECT_NAME}/third_party/autoware_ndt)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(parameters_test test/parameters_test.cpp src/node_parameters.cpp)
  target_include_directories(parameters_test PRIVATE src)
  ament_target_dependencies(parameters_test rclcpp)
  target_link_libraries(parameters_test ${PCL_LIBRARIES})
  ament_add_gtest(registration_test test/registration_test.cpp)
  target_link_libraries(registration_test localization_core)
  add_executable(replay_registration test/replay_registration.cpp)
  target_link_libraries(replay_registration localization_core)
endif()

# 仅导出固定版本算法核心，重定位不依赖持续定位节点或其状态。
install(TARGETS ml_autoware_ndt EXPORT autoware_ndt_targets ARCHIVE DESTINATION lib)
install(DIRECTORY third_party/autoware_ndt/include/ DESTINATION include)
ament_export_targets(autoware_ndt_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL Eigen3 OpenMP)
ament_package()
