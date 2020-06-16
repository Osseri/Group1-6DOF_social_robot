cmake_minimum_required(VERSION 2.8.3)
project(plane_extractor)

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
  pcl_tools
  pcm_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plane_extractor
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs pcl_conversions pcl_ros pcl_tools pcm_msgs
#  DEPENDS system_lib	
)

include_directories(
	include	
	$(find pcl_tools)	
	${catkin_INCLUDE_DIRS}	
)

link_directories(	  		 
	/opt/ros/jade/lib	
	lib/linux64	
)

add_executable(
	plane_extractor	src/plane_extractor.cpp		
						src/nui_tracker.cpp						
)

target_link_libraries(
	plane_extractor	libnuitrack.so
						libmiddleware.so					
						${catkin_LIBRARIES}																												
)

add_dependencies(plane_extractor pcm_msgs_generate_messages_cpp)

install( FILES lib/linux64/libnuitrack.so
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}    
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
