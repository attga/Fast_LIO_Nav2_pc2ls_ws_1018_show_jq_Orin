cmake_minimum_required(VERSION 3.5)
project(ml_localization_manager)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ml_localization_manager
  src/ml_localization_manager.cpp
)
ament_target_dependencies(ml_localization_manager
  rclcpp nav_msgs geometry_msgs std_msgs std_srvs robot_localization
)
target_link_libraries(ml_localization_manager
  Eigen3::Eigen
)

install(TARGETS ml_localization_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
