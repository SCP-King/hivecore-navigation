cmake_minimum_required(VERSION 3.8)
project(n01_traj_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(traj_tracker_node src/traj_tracker_node.cpp)

target_include_directories(traj_tracker_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

ament_target_dependencies(traj_tracker_node
  rclcpp geometry_msgs nav_msgs std_msgs scan_planner_msgs bspline_opt)

target_link_libraries(traj_tracker_node Eigen3::Eigen bspline_opt::bspline_opt)

install(TARGETS traj_tracker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
