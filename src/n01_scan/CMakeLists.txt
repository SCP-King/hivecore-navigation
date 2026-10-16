cmake_minimum_required(VERSION 3.8)
project(n01_scan)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(scan_validator src/scan_validator.cpp)
ament_target_dependencies(scan_validator rclcpp sensor_msgs diagnostic_msgs)

add_executable(scan_grid_map src/scan_grid_map.cpp)
ament_target_dependencies(scan_grid_map rclcpp sensor_msgs)

if(BUILD_TESTING)
  add_executable(test_voxel_map src/test_voxel_map.cpp)
  target_include_directories(test_voxel_map PRIVATE src)
  add_test(NAME test_voxel_map COMMAND test_voxel_map)
endif()

install(TARGETS scan_validator scan_grid_map DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
