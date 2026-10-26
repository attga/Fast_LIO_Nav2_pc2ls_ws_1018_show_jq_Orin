cmake_minimum_required(VERSION 3.8)
project(my_nav2_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)

add_library(cruise_speed_critic SHARED src/cruise_speed_critic.cpp)
target_compile_features(cruise_speed_critic PRIVATE cxx_std_20)
target_include_directories(cruise_speed_critic PRIVATE include ${xsimd_INCLUDE_DIRS})
# Match the tensor/SIMD settings of the Humble/Jazzy MPPI library ABI.
target_compile_definitions(cruise_speed_critic PRIVATE XTENSOR_ENABLE_XSIMD XTENSOR_USE_XSIMD)
target_link_libraries(cruise_speed_critic xtensor xtensor::use_xsimd)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  include(CheckCXXCompilerFlag)
  foreach(flag IN ITEMS -mno-avx512f -msse4.2 -mavx2 -mfma)
    string(MAKE_C_IDENTIFIER "cruise_${flag}" flag_var)
    check_cxx_compiler_flag("${flag}" ${flag_var})
    if(${flag_var})
      target_compile_options(cruise_speed_critic PRIVATE "${flag}")
    endif()
  endforeach()
endif()
ament_target_dependencies(cruise_speed_critic
  nav2_mppi_controller nav2_costmap_2d pluginlib rclcpp sensor_msgs)
pluginlib_export_plugin_description_file(nav2_mppi_controller cruise_critic.xml)
install(TARGETS cruise_speed_critic
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本
install(PROGRAMS
#  scripts/waypoint_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
