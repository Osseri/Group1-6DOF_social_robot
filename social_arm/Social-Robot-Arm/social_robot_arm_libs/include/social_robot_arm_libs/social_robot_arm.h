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

#ifndef SOCIAL_ROBOT_ARM_H_
#define SOCIAL_ROBOT_ARM_H_

#include "dynamixel.h"
#include "custom_trajectory.h"
#include "kinematics.h"
#include "dynamics.h"

#define CUSTOM_TRAJECTORY_SIZE 4
#define CUSTOM_TRAJECTORY_LINE    "custom_trajectory_line"
#define CUSTOM_TRAJECTORY_CIRCLE  "custom_trajectory_circle"
#define CUSTOM_TRAJECTORY_RHOMBUS "custom_trajectory_rhombus"
#define CUSTOM_TRAJECTORY_HEART   "custom_trajectory_heart"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DOF  3
#define TOOL1_DYNAMIXEL  "tool1_dxl"
#define TOOL2_DYNAMIXEL  "tool2_dxl"
#define TOOL3_DYNAMIXEL  "tool3_dxl"

#define X_AXIS robotis_manipulator::math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS robotis_manipulator::math::vector3(0.0, 1.0, 0.0)
#define M_Y_AXIS robotis_manipulator::math::vector3(0.0, -1.0, 0.0)
#define Z_AXIS robotis_manipulator::math::vector3(0.0, 0.0, 1.0)

class SocialRobotArm : public robotis_manipulator::RobotisManipulator
{
protected:
  bool using_platform_;
  bool gravity_compensation_mode_;
  bool mode_changing_;
  bool comm_;
  robotis_manipulator::Kinematics *kinematics_;
  robotis_manipulator::Dynamics *dynamics_;
  robotis_manipulator::JointActuator *actuator_;
  robotis_manipulator::ToolActuator *tool_[TOOL_DOF];
  robotis_manipulator::CustomTaskTrajectory *custom_trajectory_[CUSTOM_TRAJECTORY_SIZE];

public:
  SocialRobotArm();
  virtual ~SocialRobotArm();

  // Gain turn
  void gainTurn(uint8_t id, STRING p, STRING i, STRING d);
  // body position
  void setWaistPitch(KinematicPose waist_pitch_pose);
  KinematicPose getWaistPitch();
  // check moving when use moveit
  bool TrajectoryEnd();
  // for direct teaching
  std::string getMode();
  bool modeChanging(bool state);
  bool changeMode(std::string mode);
  JointWaypoint gravityCompensation();
  // Process
  void process(double present_time);
};


class SocialRobotLeftArm : public SocialRobotArm
{
private:

public:
  SocialRobotLeftArm(){}
  virtual ~SocialRobotLeftArm(){}

  // init parameters
  void init(bool using_actual_robot_state, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000", float control_loop_time = 0.010);
};

class SocialRobotRightArm : public SocialRobotArm
{
public:
  SocialRobotRightArm(){}
  virtual ~SocialRobotRightArm(){}

  // init parameters
  void init(bool using_actual_robot_state, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000", float control_loop_time = 0.010);
};

#endif // SOCIAL_ROBOT_ARM_H_
