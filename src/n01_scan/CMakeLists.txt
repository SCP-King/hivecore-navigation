cmake_minimum_required(VERSION 3.8)
project(n01_scan)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(scan_validator src/scan_validator.cpp)
ament_target_dependencies(scan_validator rclcpp sensor_msgs diagnostic_msgs)

add_executable(scan_grid_map src/scan_grid_map.cpp)
ament_target_dependencies(scan_grid_map rclcpp sensor_msgs tf2 tf2_ros geometry_msgs message_filters)

if(BUILD_TESTING)
  add_executable(test_voxel_map src/test_voxel_map.cpp)
  target_include_directories(test_voxel_map PRIVATE src)
  add_test(NAME test_voxel_map COMMAND test_voxel_map)

  add_executable(test_world_map src/test_world_map.cpp)
  target_include_directories(test_world_map PRIVATE src)
  add_test(NAME test_world_map COMMAND test_world_map)

  add_executable(test_ground_filter src/test_ground_filter.cpp)
  target_include_directories(test_ground_filter PRIVATE src)
  add_test(NAME test_ground_filter COMMAND test_ground_filter)

  add_executable(test_map_policy src/test_map_policy.cpp)
  target_include_directories(test_map_policy PRIVATE src)
  ament_target_dependencies(test_map_policy rclcpp)
  add_test(NAME test_map_policy COMMAND test_map_policy)
endif()

install(TARGETS scan_validator scan_grid_map DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()