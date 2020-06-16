cmake_minimum_required(VERSION 2.8.3)
project(pcm_msgs)	#Point Clouds Map Msgs

find_package(catkin REQUIRED std_msgs COMPONENTS message_generation)
# add_compile_options(-std=c++11)

add_message_files(
	DIRECTORY msg
	FILES
		Point.msg
		Vector.msg
		Plane.msg    
		PlaneMultiArray.msg		
)

add_service_files(
  DIRECTORY srv
  FILES    
    RegisterInMap.srv    
    PcmLocalization.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)
