cmake_minimum_required(VERSION 3.8)
project(waji_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calibrated_urdf_contract
    test/test_calibrated_urdf_contract.py
  )
endif()

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/bucket_tip_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
