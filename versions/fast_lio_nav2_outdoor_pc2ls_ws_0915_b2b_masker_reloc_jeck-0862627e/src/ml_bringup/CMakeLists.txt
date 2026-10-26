cmake_minimum_required(VERSION 3.5)
project(ml_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS scripts/imu_measurements.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
