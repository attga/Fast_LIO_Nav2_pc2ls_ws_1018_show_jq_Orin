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
  # Explicit permissions force ament's symlink installer to use a regular
  # program install, so Windows-transferred sources need no executable bit.
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

ament_package()
