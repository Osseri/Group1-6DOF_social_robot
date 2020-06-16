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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../include/social_robot_arm_libs/social_robot_arm.h"
/*****************************************************************************
 *****************************************************************************
 ** Social Robot Arm
 *****************************************************************************
*****************************************************************************/

SocialRobotArm::SocialRobotArm()
{
  gravity_compensation_mode_ = false;
  mode_changing_ = false;
  comm_ = false;
}
SocialRobotArm::~SocialRobotArm()
{
  delete kinematics_;
  delete actuator_;
  for(uint8_t index = 0; index < TOOL_DOF; index++)
    delete tool_[index];
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void SocialRobotArm::gainTurn(uint8_t id, STRING p, STRING i, STRING d)
{
  std::vector<uint8_t> jointDxlId;
  jointDxlId.push_back(id);

  STRING joint_dxl_opt_arg[2];
  joint_dxl_opt_arg[0] = "Position_P_Gain";
  joint_dxl_opt_arg[1] = p;
  void *p_joint_dxl_opt_arg = &joint_dxl_opt_arg;
  setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

  joint_dxl_opt_arg[0] = "Position_D_Gain";
  joint_dxl_opt_arg[1] = d;
  setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

  joint_dxl_opt_arg[0] = "Position_I_Gain";
  joint_dxl_opt_arg[1] = i;
  setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);
}

void SocialRobotArm::setWaistPitch(KinematicPose waist_pitch_pose)
{
  return this->getManipulator()->setWorldKinematicPose(waist_pitch_pose);
}

KinematicPose SocialRobotArm::getWaistPitch()
{
  return this->getManipulator()->getWorldKinematicPose();
}

bool SocialRobotArm::TrajectoryEnd()
{
  static bool moving = false;

  if(moving)
  {
    if(!getMovingState())
    {
      moving = false;
      return true;
    }
  }
  else
  {
    if(getMovingState())
      moving = true;
  }
  return false;
}

std::string SocialRobotArm::getMode()
{
  if(gravity_compensation_mode_)
    return "gravity_compensation_mode";
  else
    return "normal_mode";
}

bool SocialRobotArm::modeChanging(bool state)
{
  mode_changing_ = state;
  return mode_changing_;
}

bool SocialRobotArm::changeMode(std::string mode)
{
  if(!getMovingState())      //moving
  {
    if(mode == "gravity_compensation_mode")
    {
      if(gravity_compensation_mode_)
        return true;
      else
      {
//        mode_changing_ = true;
        while(comm_);
        // Change to Gravity_compensation_mode
        STRING joint_dxl_mode_arg = "gravity_compensation_mode";
        void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
        setJointActuatorMode(JOINT_DYNAMIXEL, getJointActuatorId(JOINT_DYNAMIXEL), p_joint_dxl_mode_arg);
        // tool disable
        disableAllToolActuator();
//        mode_changing_ = false;
        gravity_compensation_mode_ = true;
        return true;
      }
    }
    else if(mode == "normal_mode")
    {
      if(!gravity_compensation_mode_)
        return true;
      else
      {
//        mode_changing_ = true;
        while(comm_);
        // Change to Normal_mode
        STRING joint_dxl_mode_arg = "normal_mode";
        void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
        setJointActuatorMode(JOINT_DYNAMIXEL, getJointActuatorId(JOINT_DYNAMIXEL), p_joint_dxl_mode_arg);
        // tool enable
        enableAllToolActuator();
//        mode_changing_ = false;
        gravity_compensation_mode_ = false;
        return true;
      }
    }
    else
    {
      log::warn("Fail to change the mode. Wrong mode name.");
      return false;
    }
  }
  else
  {
    log::warn("Fail to change the mode. Robot is moving.");
    return false;
  }
}

JointWaypoint SocialRobotArm::gravityCompensation()
{
  // Get present_joint_value
  std::vector<JointValue> present_joint_value = getManipulator()->getAllActiveJointValue();

  // Dynamics solve
  std::map<Name, double> joint_torque_map;
  if(solveGravityTerm(&joint_torque_map))
  {
    auto names = getManipulator()->getAllActiveJointComponentName();
    std::vector<double> joint_torque;
    for(uint8_t i = 0; i < names.size(); i++)
    {
      if(joint_torque_map.find(names.at(i))!=joint_torque_map.end())
        joint_torque.push_back(joint_torque_map.at(names.at(i)));
      else
        joint_torque.push_back(0.0);
//      log::warn("name: "+names.at(i)+", torque: ",joint_torque_map.at(names.at(i)));
    }
    robotis_manipulator::setEffortToValue(&present_joint_value, joint_torque);
  }

  return present_joint_value;
}

void SocialRobotArm::process(double present_time)
{
  if(mode_changing_)
    return;
  comm_ = true;
  // get Present position
  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();
  solveForwardKinematics();

  JointWaypoint goal_joint_value;
  JointWaypoint goal_tool_value;
  if(!gravity_compensation_mode_)
  {
    goal_joint_value = getJointGoalValueFromTrajectory(present_time, DYNAMICS_GRAVITY_ONLY);
    goal_tool_value = getToolGoalValue();
  }
  else
  {
    if(using_platform_)
      goal_joint_value = gravityCompensation();
  }

  if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
  if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);
  comm_ = false;
}

/*****************************************************************************
 *****************************************************************************
 ** Social Robot Left Arm
 *****************************************************************************
*****************************************************************************/

void SocialRobotLeftArm::init(bool using_actual_robot_state, STRING usb_port, STRING baud_rate, float control_loop_time)
{
  Eigen::Vector3d body_position_vector = math::vector3(0.0, 0.0, 0.0);
  Eigen::Matrix3d body_orientation_matrix = math::convertRPYToRotationMatrix(0.0, 0.0, 0.0);

  /*****************************************************************************
   ** Initialize Manipulator Parameter
  *****************************************************************************/
  addWorld("Waist_Pitch",       // world name
          "LShoulder_Pitch",             // child name
          body_position_vector,
          body_orientation_matrix
          );

  addJoint("LShoulder_Pitch",  // my name
          "Waist_Pitch",   // parent name
          "LShoulder_Roll",  // child name
          math::vector3(0.0, 0.1065, 0.2815),               // relative positionSolverUsingNewtonEuler
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          Y_AXIS,    // axis of rotation
          1,        // actuator id
          M_PI,      // max joint limit (3.14 rad)
          -M_PI,    // min joint limit (-3.14 rad)
          1.0,     //coefficient
          0.25802501, //mass
          math::matrix3(0.00078167957, -0.00000062373043, 0.0000000021638484,
                        -0.00000062373043, 0.00010532761, 0.00000000024776853,
                        0.0000000021638484, 0.00000000024776853, 0.00077621568),    //inertia tensor
          math::vector3(0.00046871137, 0.047922220, -0.00000059742066), // COM
          6.0);

  addJoint("LShoulder_Roll",  // my name
          "LShoulder_Pitch",   // parent name
          "LElbow_Pitch",  // child name
          math::vector3(0.0, 0.06375, 0.0),               // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          X_AXIS,    // axis of rotation
          2,        // actuator id
          M_PI,      // max joint limit (80 deg)
          -M_PI,    // min joint limit (-80 deg)
          1.0,     //coefficient
          0.24899507, //mass
          math::matrix3(0.0015813034, -0.00000032124803, -0.0000000072719573,
                        -0.00000032124803, 0.00016009044, 0.00000026716959,
                        -0.0000000072719573, 0.00000026716959, 0.0016224156),    //inertia tensor
          math::vector3(0.000026047442, 0.069489221, -0.000017513593), // COM
          5.0);

  addJoint("LElbow_Pitch",  // my name
          "LShoulder_Roll",   // parent name
          "LElbow_Yaw",  // child name
          math::vector3(0.0, 0.09525, 0.0),               // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          Y_AXIS,    // axis of rotation
          3,        // actuator id
          M_PI,      // max joint limit (150 deg)
          -M_PI,    // min joint limit (-150 deg)
          -1.0,     //coefficient
          0.22953581, //mass
          math::matrix3(0.0012018549, 0.0, 0.0,
                        0.0, 0.000069116644, -0.0000089338855,
                        0.0, -0.0000089338855, 0.0011951211),    //inertia tensor
          math::vector3(0.0, 0.067026195, 0.00052754282), // COM
          -3.0);

  addJoint("LElbow_Yaw",  // my name
          "LElbow_Pitch",   // parent name
          "LWrist_Pitch",  // child name
          math::vector3(0.0, 0.086, 0.0),               // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          Z_AXIS,    // axis of rotation
          4,        // actuator id
          M_PI,      // max joint limit (45 deg)
          -M_PI,    // min joint limit (-80 deg)
          1.0,     //coefficient
          0.22625975, //mass
          math::matrix3(0.0013412658, -0.00000081401171, 0.0000000072713423,
                        -0.00000081401171, 0.00014226184, -0.00000027066590,
                        0.0000000072713423, -0.00000027066590, 0.0013122537),    //inertia tensor
          math::vector3(-0.00022532640, 0.066117708, 0.000027620342), // COM
          6.0);

  addJoint("LWrist_Pitch",  // my name
          "LElbow_Yaw",   // parent name
          "LWrist_Roll",  // child name
          math::vector3(0.0, 0.08745, 0.0),               // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          Y_AXIS,    // axis of rotation
          5,        // actuator id
          M_PI,      // max joint limit (60 deg)
          -M_PI,    // min joint limit (-60 deg)
          -1.0,     //coefficient
          0.13034150, //mass
          math::matrix3(0.00053537399, -0.0000034499040, 0.0,
                        -0.0000034499040, 0.000028340595, 0.0,
                        0.0, 0.0, 0.000082859645),    //inertia tensor
          math::vector3(0.00040281206, 0.058804030, 0.0), // COM
          -3.0);

  addJoint("LWrist_Roll",  // my name
          "LWrist_Pitch",   // parent name
          "LFinger_1",  // child name
          math::vector3(0.0, 0.07655, 0.0),               // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          X_AXIS,    // axis of rotation
          6,        // actuator id
          M_PI,      // max joint limit (60 deg)
          -M_PI,    // min joint limit (-60 deg)
          1.0,     //coefficient
          0.35505172, //mass
          math::matrix3(0.0023682289, -0.0000017796221, -0.000033729539,
                        -0.0000017796221, 0.00030871837, 0.000072153079,
                        -0.000033729539, 0.000072153079, 0.0022820369),    //inertia tensor
          math::vector3(-0.000047949459, 0.07041888, -0.0018305952), // COM
          3.0);
  addComponentChild("LWrist_Roll", "LFinger_2");
  addComponentChild("LWrist_Roll", "LFinger_3");

  addTool("LFinger_1",   // my name
          "LWrist_Roll", // parent name
          math::vector3(0.0, 0.109, 0.0), // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          7,     // actuator idvirtual
          20*DEG2RAD,      // max joint limit (30 deg)
          -15*DEG2RAD    // min joint limit (0 deg)
          );

  addTool("LFinger_2",   // my name
          "LWrist_Roll", // parent name
          math::vector3(0.0, 0.109, 0.0), // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          8,     // actuator idvirtual
          20*DEG2RAD,      // max joint limit (30 deg)
          -15*DEG2RAD    // min joint limit (0 deg)
          );

  addTool("LFinger_3",   // my name
          "LWrist_Roll", // parent name
          math::vector3(0.0, 0.109, 0.0), // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          9,     // actuator idvirtual
          20*DEG2RAD,      // max joint limit (30 deg)
          -20*DEG2RAD    // min joint limit (0 deg)
          );

  /*****************************************************************************
  ** Initialize Kinematics
  *****************************************************************************/
  kinematics_ = new kinematics::SolverUsingCRAndSRJacobian();
//  kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
  addKinematics(kinematics_);

  /*****************************************************************************
  ** Initialize Dynamics
  *****************************************************************************/
  dynamics_ = new dynamics::SolverUsingNewtonEuler();
  addDynamics(dynamics_);
    using_platform_ = using_actual_robot_state;
    if(using_platform_)
    {
      /*****************************************************************************
      ** Initialize joint Actuator
      *****************************************************************************/
      actuator_ = new dynamixel::ArmsDynamixel(control_loop_time);

      // communication setting argument
      STRING dxl_comm_arg[2] = {usb_port, baud_rate};
      void *p_dxl_comm_arg = &dxl_comm_arg;

      // set joint actuator id
      std::vector<uint8_t> jointDxlId;
      jointDxlId.push_back(1);
      jointDxlId.push_back(2);
      jointDxlId.push_back(3);
      jointDxlId.push_back(4);
      jointDxlId.push_back(5);
      jointDxlId.push_back(6);
      addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

      // set joint actuator control mode
      STRING joint_dxl_mode_arg = "position_mode";
      void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
      setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

      // set joint actuator parameter
//      gainTurn(1, "3000", "100", "200");
//      gainTurn(2, "3000", "100", "200");
//      gainTurn(3, "3000", "100", "200");
//      gainTurn(4, "1500", "100", "200");
//      gainTurn(5, "1000", "100", "200");
//      gainTurn(6, "800", "0", "0");

      /*****************************************************************************
      ** Initialize Tool Actuator
      *****************************************************************************/
      tool_[0] = new dynamixel::HandsDynamixel();
      tool_[1] = new dynamixel::HandsDynamixel();
      tool_[2] = new dynamixel::HandsDynamixel();

      uint8_t gripperDxlId =7;
      addToolActuator(TOOL1_DYNAMIXEL, tool_[0], gripperDxlId, p_dxl_comm_arg);

      gripperDxlId =8;
      addToolActuator(TOOL2_DYNAMIXEL, tool_[1], gripperDxlId, p_dxl_comm_arg);

      gripperDxlId =9;
      addToolActuator(TOOL3_DYNAMIXEL, tool_[2], gripperDxlId, p_dxl_comm_arg);

      // Set gripper actuator control mode
      STRING gripper_dxl_mode_arg = "position_mode";
      void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
      setToolActuatorMode(TOOL1_DYNAMIXEL, p_gripper_dxl_mode_arg);
      setToolActuatorMode(TOOL2_DYNAMIXEL, p_gripper_dxl_mode_arg);

      gripper_dxl_mode_arg = "current_based_position_mode";
      setToolActuatorMode(TOOL3_DYNAMIXEL, p_gripper_dxl_mode_arg);

      // Set gripper actuator parameter
      STRING gripper_dxl_opt_arg[2];
      void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
      gripper_dxl_opt_arg[0] = "Profile_Acceleration";
      gripper_dxl_opt_arg[1] = "20";
      setToolActuatorMode(TOOL1_DYNAMIXEL, p_gripper_dxl_opt_arg);
      setToolActuatorMode(TOOL2_DYNAMIXEL, p_gripper_dxl_opt_arg);
      setToolActuatorMode(TOOL3_DYNAMIXEL, p_gripper_dxl_opt_arg);

      gripper_dxl_opt_arg[0] = "Profile_Velocity";
      gripper_dxl_opt_arg[1] = "200";
      setToolActuatorMode(TOOL1_DYNAMIXEL, p_gripper_dxl_opt_arg);
      setToolActuatorMode(TOOL2_DYNAMIXEL, p_gripper_dxl_opt_arg);
      setToolActuatorMode(TOOL3_DYNAMIXEL, p_gripper_dxl_opt_arg);

      // all actuator enable
      enableAllActuator();
      receiveAllJointActuatorValue();
      receiveAllToolActuatorValue();
    }

    /*****************************************************************************
    ** Initialize Custom Trajectory
    *****************************************************************************/
    custom_trajectory_[0] = new custom_trajectory::Line();
    custom_trajectory_[1] = new custom_trajectory::Circle();
    custom_trajectory_[2] = new custom_trajectory::Rhombus();
    custom_trajectory_[3] = new custom_trajectory::Heart();

    addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
    addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
    addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
    addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
    log::println("Left Arm Initialize Success.","GREEN");
}

/*****************************************************************************
 *****************************************************************************
 ** Social Robot Right Arm
 *****************************************************************************
*****************************************************************************/

void SocialRobotRightArm::init(bool using_actual_robot_state, STRING usb_port, STRING baud_rate, float control_loop_time)
{

Eigen::Vector3d body_position_vector = math::vector3(0.0, 0.0, 0.0);
Eigen::Matrix3d body_orientation_matrix = math::convertRPYToRotationMatrix(0.0, 0.0, 0.0);

/*****************************************************************************
 ** Initialize Manipulator Parameter
*****************************************************************************/
addWorld("Waist_Pitch",          // Base name
        "RShoulder_Pitch",                // child name
        body_position_vector,
        body_orientation_matrix
        );

addJoint("RShoulder_Pitch",  // my name
        "Waist_Pitch",   // parent name
        "RShoulder_Roll",  // child name
        math::vector3(0.0, -0.1065, 0.2815),               // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        Y_AXIS,    // axis of rotation
        11,        // actuator id
        M_PI,      // max joint limit (3.14 rad)
        -M_PI,    // min joint limit (-3.14 rad)
        -1.0,     //coefficient
         0.25802501, //mass
         math::matrix3(0.00078167957, 0.00000062373043, 0.0000000021638484,
                       0.00000062373043, 0.00010532761, -0.00000000024776853,
                       0.0000000021638484, -0.00000000024776853, 0.00077621568),    //inertia tensor
         math::vector3(0.00046871137, -0.047922220, -0.00000059742066), // COM
        -5.0);

addJoint("RShoulder_Roll",  // my name
        "RShoulder_Pitch",   // parent name
        "RElbow_Pitch",  // child name
        math::vector3(0.0, -0.06375, 0.0),               // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        X_AXIS,    // axis of rotation
        12,        // actuator id
        M_PI,      // max joint limit (80 deg)
        -M_PI,    // min joint limit (-80 deg)
        1.0,     //coefficient
         0.24899507, //mass
         math::matrix3(0.0015813034, 0.00000032124803, -0.0000000072719573,
                       0.00000032124803, 0.00016009044, -0.00000026716959,
                       -0.0000000072719573, -0.00000026716959, 0.0016224156),    //inertia tensor
         math::vector3(0.000026047442, -0.069489221, -0.000017513593), // COM
        5.0);

addJoint("RElbow_Pitch",  // my name
        "RShoulder_Roll",   // parent name
        "RElbow_Yaw",  // child name
        math::vector3(0.0, -0.09525, 0.0),               // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        Y_AXIS,    // axis of rotation
        13,        // actuator id
        M_PI,      // max joint limit (150 deg)
        -M_PI,    // min joint limit (-150 deg)
        1.0,     //coefficient
         0.22953581, //mass
         math::matrix3(0.0012018549, 0.0, 0.0,
                       0.0, 0.000069116644, 0.0000089338855,
                       0.0, 0.0000089338855, 0.0011951211),    //inertia tensor
         math::vector3(0.0, -0.067026195, 0.00052754282), // COM
        3.0);

addJoint("RElbow_Yaw",  // my name
        "RElbow_Pitch",   // parent name
        "RWrist_Pitch",  // child name
        math::vector3(0.0, -0.086, 0.0),               // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        Z_AXIS,    // axis of rotation
        14,        // actuator id
        M_PI,      // max joint limit (45 deg)
        -M_PI,    // min joint limit (-80 deg)
        1.0,     //coefficient
         0.22625975, //mass
         math::matrix3(0.0013412658, 0.00000081401171, 0.0000000072713423,
                       0.00000081401171, 0.00014226184, 0.00000027066590,
                       0.0000000072713423, 0.00000027066590, 0.0013122537),    //inertia tensor
         math::vector3(-0.00022532640, -0.066117708, 0.000027620342), // COM
        6.0);

addJoint("RWrist_Pitch",  // my name
        "RElbow_Yaw",   // parent name
        "RWrist_Roll",  // child name
        math::vector3(0.0, -0.08745, 0.0),               // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        Y_AXIS,    // axis of rotation
        15,        // actuator id
        M_PI,      // max joint limit (60 deg)  changeMode("gravity_compensation_mode");
        -M_PI,    // min joint limit (-60 deg)
        1.0,     //coefficient
         0.13034150, //mass
         math::matrix3(0.00053537399, 0.0000034499040, 0.0,
                       0.0000034499040, 0.000028340595, 0.0,
                       0.0, 0.0, 0.000082859645),    //inertia tensor
         math::vector3(0.00040281206, -0.058804030, 0.0), // COM
        3.0);

addJoint("RWrist_Roll",  // my name
        "RWrist_Pitch",   // parent name
        "RFinger_1",  // child name
        math::vector3(0.0, -0.07655, 0.0),               // relative position두 구의 외접￩
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        X_AXIS,    // axis of rotation
        16,        // actuator id
        M_PI,      // max joint limit (60 deg)
        -M_PI,    // min joint limit (-60 deg)
        1.0,     //coefficient
        0.35505172, //mass
        math::matrix3(0.0023682289, 0.0000017796221, -0.000033729539,
                      0.0000017796221, 0.00030871837, -0.000072153079,
                      -0.000033729539, -0.000072153079, 0.0022820369),    //inertia tensor
        math::vector3(-0.000047949459, -0.07041888, -0.0018305952), // COM
        3.0);

addComponentChild("RWrist_Roll", "RFinger_2");
addComponentChild("RWrist_Roll", "RFinger_3");

addTool("RFinger_1",   // my name
        "RWrist_Roll", // parent name
        math::vector3(0.0, 0.109, 0.0), // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        17,     // actuator idvirtual
        20*DEG2RAD,      // max joint limit (30 deg)
        -15*DEG2RAD    // min joint limit (0 deg)
        );

addTool("RFinger_2",   // my name
        "RWrist_Roll", // parent name
        math::vector3(0.0, 0.109, 0.0), // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        18,     // actuator idvirtual
        20*DEG2RAD,      // max joint limit (30 deg)
        -15*DEG2RAD    // min joint limit (0 deg)
        );

addTool("RFinger_3",   // my name
        "RWrist_Roll", // parent name
        math::vector3(0.0, 0.109, 0.0), // relative position
        math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
        19,     // actuator idvirtual
        20*DEG2RAD,      // max joint limit (30 deg)
        -15*DEG2RAD    // min joint limit (0 deg)
        );

/*****************************************************************************
** Initialize Kinematics
*****************************************************************************/
kinematics_ = new kinematics::SolverUsingCRAndSRJacobian();
//kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
addKinematics(kinematics_);

/*****************************************************************************
** Initialize Dynamics
*****************************************************************************/
dynamics_ = new dynamics::SolverUsingNewtonEuler();
addDynamics(dynamics_);

  using_platform_ = using_actual_robot_state;
  if(using_platform_)
  {
    /*****************************************************************************
    ** Initialize joint Actuator
    *****************************************************************************/
    actuator_ = new dynamixel::ArmsDynamixel(control_loop_time);

    // communication setting argument
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // set joint actuator id
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(11);
    jointDxlId.push_back(12);
    jointDxlId.push_back(13);
    jointDxlId.push_back(14);
    jointDxlId.push_back(15);
    jointDxlId.push_back(16);
    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

    // set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

    // set joint actuator parameter
//    STRING joint_dxl_opt_arg[2] = {"Position_P_Gain", "1200"};
//    void *p_joint_dxl_opt_arg = &joint_dxl_opt_arg;
//    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

//    joint_dxl_opt_arg[0] = "Position_D_Gain";
//    joint_dxl_opt_arg[1] = "200";
//    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_opt_arg);

//    gainTurn(11, "6000", "600", "200");
//    gainTurn(12, "6000", "600", "200");
//    gainTurn(13, "3000", "300", "200");
//    gainTurn(14, "5000", "500", "200");
//    gainTurn(15, "1000", "150", "200");

    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    tool_[0] = new dynamixel::HandsDynamixel();
    tool_[1] = new dynamixel::HandsDynamixel();
    tool_[2] = new dynamixel::HandsDynamixel();

    uint8_t gripperDxlId =17;
    addToolActuator(TOOL1_DYNAMIXEL, tool_[0], gripperDxlId, p_dxl_comm_arg);

    gripperDxlId =18;
    addToolActuator(TOOL2_DYNAMIXEL, tool_[1], gripperDxlId, p_dxl_comm_arg);

    gripperDxlId =19;
    addToolActuator(TOOL3_DYNAMIXEL, tool_[2], gripperDxlId, p_dxl_comm_arg);

    // Set gripper actuator control mode
    STRING gripper_dxl_mode_arg = "position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    setToolActuatorMode(TOOL1_DYNAMIXEL, p_gripper_dxl_mode_arg);
    setToolActuatorMode(TOOL2_DYNAMIXEL, p_gripper_dxl_mode_arg);

    gripper_dxl_mode_arg = "current_based_position_mode";
    setToolActuatorMode(TOOL3_DYNAMIXEL, p_gripper_dxl_mode_arg);

    // Set gripper actuator parameter
    STRING gripper_dxl_opt_arg[2];
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    gripper_dxl_opt_arg[0] = "Profile_Acceleration";
    gripper_dxl_opt_arg[1] = "20";
    setToolActuatorMode(TOOL1_DYNAMIXEL, p_gripper_dxl_opt_arg);
    setToolActuatorMode(TOOL2_DYNAMIXEL, p_gripper_dxl_opt_arg);
    setToolActuatorMode(TOOL3_DYNAMIXEL, p_gripper_dxl_opt_arg);

    gripper_dxl_opt_arg[0] = "Profile_Velocity";
    gripper_dxl_opt_arg[1] = "200";
    setToolActuatorMode(TOOL1_DYNAMIXEL, p_gripper_dxl_opt_arg);
    setToolActuatorMode(TOOL2_DYNAMIXEL, p_gripper_dxl_opt_arg);
    setToolActuatorMode(TOOL3_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // all actuator enable
    enableAllActuator();
    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
  }

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new custom_trajectory::Line();
  custom_trajectory_[1] = new custom_trajectory::Circle();
  custom_trajectory_[2] = new custom_trajectory::Rhombus();
  custom_trajectory_[3] = new custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
  log::println("Right Arm Initialize Success.","GREEN");
}
