cmake_minimum_required(VERSION 2.8.3)
project(pcl_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#Ros
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tool_class
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pcl_tools
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs pcl_conversions pcl_ros tool_class
#  DEPENDS system_lib	
)

include_directories(
	include	
	${catkin_INCLUDE_DIRS}
	$(find tool_class)		
)

link_directories(	  		 
	/opt/ros/jade/lib	
	lib/linux64	
)

add_library(pcl_tools src/pcl_tools.cpp)
