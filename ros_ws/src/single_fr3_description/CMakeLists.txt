cmake_minimum_required(VERSION 3.16)
project(single_fr3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()

