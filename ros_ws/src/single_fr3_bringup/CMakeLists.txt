cmake_minimum_required(VERSION 3.16)
project(single_fr3_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/controllers.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/controllers.fake.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/robot.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
