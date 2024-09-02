cmake_minimum_required(VERSION 3.5)
project(map_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

add_executable(merger_node src/laser_scan_merger.cpp)

ament_target_dependencies(merger_node 
  rclcpp 
  sensor_msgs 
  laser_geometry 
  pcl_ros 
  PCL
)

install(TARGETS
  merger_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
