cmake_minimum_required(VERSION 2.8.3)
project(social_robot_arm_controller)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    gazebo_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    social_robot_arm_msgs
    robotis_manipulator
    social_robot_arm_libs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    cmake_modules
)
find_package(Boost REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  gazebo_msgs
  geometry_msgs
  moveit_msgs
  trajectory_msgs
  social_robot_arm_msgs
  robotis_manipulator
  social_robot_arm_libs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  cmake_modules
  DEPENDS Boost
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(social_robot_arm_controller src/social_robot_arm_controller.cpp)
add_dependencies(social_robot_arm_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(social_robot_arm_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS social_robot_arm_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
