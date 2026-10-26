cmake_minimum_required(VERSION 3.16)
project(ml_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS scripts/imu_measurements.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_test test/test_description.py)
  ament_add_pytest_test(localization_inputs_test test/test_localization_inputs.py)
endif()

ament_package()
