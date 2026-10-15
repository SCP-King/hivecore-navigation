cmake_minimum_required(VERSION 3.8)
project(n01_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs)

install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
