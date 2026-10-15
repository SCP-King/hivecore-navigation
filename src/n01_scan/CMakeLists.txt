cmake_minimum_required(VERSION 3.8)
project(n01_scan)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(scan_validator src/scan_validator.cpp)
ament_target_dependencies(scan_validator rclcpp sensor_msgs diagnostic_msgs)

install(TARGETS scan_validator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
