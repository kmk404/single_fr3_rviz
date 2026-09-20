cmake_minimum_required(VERSION 3.16)
project(omega7_servo_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/controller.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface pluginlib rclcpp realtime_tools sensor_msgs std_msgs)
target_link_libraries(${PROJECT_NAME} ruckig::ruckig)
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ruckig_profile_test test/ruckig_profile_test.cpp)
  target_link_libraries(ruckig_profile_test ruckig::ruckig)
  ament_add_gtest(fault_diagnostics_test test/fault_diagnostics_test.cpp)
  target_include_directories(fault_diagnostics_test PRIVATE include)
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface pluginlib rclcpp realtime_tools ruckig sensor_msgs std_msgs)
ament_package()
