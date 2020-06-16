/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Hye-Jong KIM */

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef SOCIAL_ROBOT_ARM_CONTROL_GUI_QNODE_HPP_
#define SOCIAL_ROBOT_ARM_CONTROL_GUI_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <ros/network.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <string>
#include <sstream>
#include <map>
#include <QThread>
#include <QStringListModel>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

#include "social_robot_arm_msgs/ManipulatorState.h"
#include "social_robot_arm_msgs/SetJointPosition.h"
#include "social_robot_arm_msgs/SetKinematicsPose.h"
#include "social_robot_arm_msgs/SetDrawingTrajectory.h"
#include "social_robot_arm_msgs/SetActuatorState.h"
#include "social_robot_arm_msgs/SetMode.h"
#include "social_robot_arm_msgs/MotionSave.h"
#include "social_robot_arm_msgs/MotionControl.h"

// for test
//#include <cstdlib>

#endif

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace social_robot_arm_control_gui
{

Eigen::Vector3d convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion);
Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Vector3d convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation);
Eigen::Matrix3d convertRPYToRotationMatrix(double roll, double pitch, double yaw);
Eigen::Matrix3d convertRollAngleToRotationMatrix(double angle);
Eigen::Matrix3d convertPitchAngleToRotationMatrix(double angle);
Eigen::Matrix3d convertYawAngleToRotationMatrix(double angle);

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode: public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init(std::string left_hand_name_for_pose, std::string right_hand_name_for_pose);
  void run();

  /*********************
   ** Logging
   **********************/
  enum LogLevel
  {
    Debug, Info, Warn, Error, Fatal
  };

  QStringListModel* loggingModel(){ return &logging_model_; }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "GUI");

  /*********************
   ** Data Get Function
   **********************/
  std::map<std::string,double> getLeftArmJointAngle();
  std::map<std::string,double> getRightArmJointAngle();
  std::vector<double> getLeftHandKinematicsPose();
  std::vector<double> getRightHandKinematicsPose();
  bool usingLeftArm();
  bool usingRightArm();
  bool getLeftArmMovingState();
  bool getRightArmMovingState();
  bool getLeftArmActuatorState();
  bool getRightArmActuatorState();

  // motion
  std::string getModeState();
  uint32_t getMotionStorageSize();
  std::map<std::string,double> getMotionStorage(uint32_t index);
  bool saveMotion(uint32_t index, double time);
  bool saveMotion(social_robot_arm_msgs::MotionSave motion_storage);
  bool removeMotion(uint32_t index);
  bool clearMotion();

  /*********************
   ** Subscriber callback
   **********************/
  void updateModeState(const std_msgs::String::ConstPtr &msg);
  void updateLeftArmStates(const social_robot_arm_msgs::ManipulatorState::ConstPtr &msg);
  void updateRightArmStates(const social_robot_arm_msgs::ManipulatorState::ConstPtr &msg);
  void updateLeftArmJointStates(const sensor_msgs::JointState::ConstPtr &msg);
  void updateRightArmJointStates(const sensor_msgs::JointState::ConstPtr &msg);
  void updateLeftHandsKinematicPose(const social_robot_arm_msgs::KinematicsPose::ConstPtr &msg);
  void updateRightHandsKinematicPose(const social_robot_arm_msgs::KinematicsPose::ConstPtr &msg);

  /*********************
   ** Publisher call
   **********************/
  void sendEStop();
  void sendSetMode(bool mode);
  void sendMotionSave(uint32_t repeat);
  void sendMotionControl(uint8_t request);

  /*********************
   ** ServiceClient call
   **********************/
  bool sendLeftArmGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool sendRightArmGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);

  bool sendLeftArmKinematicPose(std::vector<double> kinematics_pose, double path_time);
  bool sendRightArmKinematicPose(std::vector<double> kinematics_pose, double path_time);

  bool sendLeftArmMoveitGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool sendRightArmMoveitGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);

  bool sendLeftArmMoveitKinematicPose(std::vector<double> kinematics_pose, double path_time);
  bool sendRightArmMoveitKinematicPose(std::vector<double> kinematics_pose, double path_time);

  bool sendLeftHandPosition(std::vector<std::string> joint_name, std::vector<double> joint_angle);
  bool sendRightHandPosition(std::vector<std::string> joint_name, std::vector<double> joint_angle);

  bool sendLeftArmActuatorState(bool actuator_state);
  bool sendRightArmActuatorState(bool actuator_state);

  /*********************
   ** Signal
   **********************/
Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

private:
  int     init_argc_;
  char**  init_argv_;
  QStringListModel    logging_model_;

  // Publisher
  ros::Publisher e_stop_pub_;
  ros::Publisher set_mode_pub_;
  ros::Publisher motion_save_pub_;
  ros::Publisher motion_control_pub_;

  // Subscriber
  ros::Subscriber motion_state_sub_;
  ros::Subscriber left_arm_states_sub_;
  ros::Subscriber right_arm_states_sub_;
  ros::Subscriber left_arm_joint_states_sub_;
  ros::Subscriber right_arm_joint_states_sub_;
  ros::Subscriber left_hands_kinematics_pose_sub_;
  ros::Subscriber right_hands_kinematics_pose_sub_;

  // Client
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_task_space_path_client_;
  ros::ServiceClient set_moveit_joint_position_client_;
  ros::ServiceClient set_moveit_kinematics_pose_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient set_actuator_state_client_;

  // Data Set
  std::string mode_state_;
  social_robot_arm_msgs::MotionSave motion_storage_;
  std::map<std::string, double> left_arm_joint_angle_;
  std::map<std::string, double> right_arm_joint_angle_;

  std::map<std::string, double> left_hand_position_;
  std::map<std::string, double> right_hand_position_;

  std::string left_hand_name_for_pose_;
  social_robot_arm_msgs::KinematicsPose left_hand_kinematics_pose_;
  std::string right_hand_name_for_pose_;
  social_robot_arm_msgs::KinematicsPose right_hand_kinematics_pose_;

  bool left_arm_moving_state_;
  bool right_arm_moving_state_;
  bool left_arm_actuator_state_;
  bool right_arm_actuator_state_;  
};

}  // namespace social_robot_arm_control_gui

#endif /* SOCIAL_ROBOT_ARM_CONTROL_GUI_QNODE_HPP_ */
