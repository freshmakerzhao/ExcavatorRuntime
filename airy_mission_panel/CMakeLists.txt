cmake_minimum_required(VERSION 3.16)
project(airy_mission_panel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(airy_excavator_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME}_state
  src/panel_state.cpp
)
set_target_properties(${PROJECT_NAME}_state PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME} SHARED
  include/airy_mission_panel/excavation_panel.hpp
  src/excavation_panel.cpp
  src/excavation_panel_ros.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  action_msgs
  airy_excavator_interfaces
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_action
  rviz_common
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_state
  Qt5::Widgets
)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_state
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins_description.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_panel_state test/test_panel_state.cpp)
  target_link_libraries(test_panel_state ${PROJECT_NAME}_state)

  ament_add_gtest(test_plugin_discovery test/test_plugin_discovery.cpp)
  ament_target_dependencies(test_plugin_discovery pluginlib rviz_common)
  target_link_libraries(test_plugin_discovery Qt5::Widgets)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
