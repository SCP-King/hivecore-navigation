cmake_minimum_required(VERSION 3.8)
project(scan_planner)

# Optimize production math/grid loops while preserving explicit Debug builds.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen)

# R2 navigation only: call the installed SDK, never build another chassis driver.
add_executable(n01_navigation_controller src/navigation_controller.cpp)
target_include_directories(n01_navigation_controller PRIVATE include /opt/hivecore/hive2-motion/1.0.0/install/include)
ament_target_dependencies(n01_navigation_controller rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs scan_planner_msgs)
target_link_libraries(n01_navigation_controller /opt/hivecore/hive2-motion/1.0.0/install/lib/libmotion_controller.so)
set_target_properties(n01_navigation_controller PROPERTIES INSTALL_RPATH /opt/hivecore/hive2-motion/1.0.0/install/lib)
install(TARGETS n01_navigation_controller DESTINATION lib/${PROJECT_NAME})
install(TARGETS scan_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  add_executable(test_recovery_ladder test/test_recovery_ladder.cpp)
  target_include_directories(test_recovery_ladder PRIVATE include)
  target_compile_options(test_recovery_ladder PRIVATE -UNDEBUG)
  add_test(NAME navigation_recovery_ladder COMMAND test_recovery_ladder)

  add_executable(test_tracking_core test/test_tracking_core.cpp)
  target_include_directories(test_tracking_core PRIVATE include)
  target_compile_options(test_tracking_core PRIVATE -UNDEBUG)
  add_test(NAME navigation_tracking_core COMMAND test_tracking_core)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
endif()

ament_package()
