cmake_minimum_required(VERSION 3.8)
project(airy_excavator_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../rviz/airy_points.rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_operator_launch
    test/test_operator_launch.py
  )
  ament_add_pytest_test(
    test_operator_profiles
    test/test_operator_profiles.py
  )
endif()

ament_package()
