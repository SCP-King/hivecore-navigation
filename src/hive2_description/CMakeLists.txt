cmake_minimum_required(VERSION 3.10)
project(hive2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
