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

#include "../include/social_robot_arm_libs/dynamics.h"

using namespace dynamics;

/*****************************************************************************
** Dynamics Solver Using Newton-Euler method
*****************************************************************************/

///////////////////////private Function/////////////////////////

bool SolverUsingNewtonEuler::setExterrnalEffort(std::map<Name, VectorXd> external_effort)
{
  external_effort_ = external_effort;
  return true;
}

std::map<Name, double> SolverUsingNewtonEuler::getJointTorqueSingleComponent(Manipulator manipulator, Name my_name, Eigen::Vector3d* my_force, Eigen::Vector3d* my_torque)
{
  std::map<Name, double> joint_torque;

  // Parameters
  Pose my_pose_value = manipulator.getComponentPoseFromWorld(my_name);
  double my_mass = manipulator.getComponentMass(my_name);
  Eigen::Vector3d my_center_of_mass = my_pose_value.kinematic.position + my_pose_value.kinematic.orientation * manipulator.getComponentCenterOfMass(my_name);
  Eigen::Matrix3d my_inertia_tensor = my_pose_value.kinematic.orientation * manipulator.getComponentInertiaTensor(my_name) * my_pose_value.kinematic.orientation.transpose();
  Eigen::Matrix3d my_inertia_tensor_2 = my_inertia_tensor + my_mass*math::skewSymmetricMatrix(my_center_of_mass)*math::skewSymmetricMatrix(my_center_of_mass).transpose();

  // Calc Momentums
  Eigen::Vector3d momentum = my_mass * (my_pose_value.dynamic.linear.velocity + math::skewSymmetricMatrix(my_pose_value.dynamic.angular.velocity)*my_center_of_mass);
  Eigen::Vector3d angular_momentum = my_mass * (math::skewSymmetricMatrix(my_center_of_mass) * my_pose_value.dynamic.linear.velocity)
      + (my_inertia_tensor_2 * my_pose_value.dynamic.angular.velocity);

  // Calc Force and Torque
  *my_force = my_mass * (my_pose_value.dynamic.linear.acceleration + math::skewSymmetricMatrix(my_pose_value.dynamic.angular.acceleration)*my_center_of_mass)
      + math::skewSymmetricMatrix(my_pose_value.dynamic.angular.velocity) * momentum;
  *my_torque = my_mass * math::skewSymmetricMatrix(my_center_of_mass) * my_pose_value.dynamic.linear.acceleration
      + my_inertia_tensor_2 * my_pose_value.dynamic.angular.acceleration
      + math::skewSymmetricMatrix(my_pose_value.dynamic.linear.velocity) * momentum
      + math::skewSymmetricMatrix(my_pose_value.dynamic.angular.velocity) * angular_momentum;

  // add Gravity Force
  double gravity_acceleration = 9.8;
  Eigen::Vector3d gravity_force = math::vector3(0, 0, -gravity_acceleration * my_mass);
  Eigen::Vector3d gravity_torque = math::skewSymmetricMatrix(my_center_of_mass)*gravity_force;
  *my_force -= gravity_force;
  *my_torque -= gravity_torque;

  // Consider External Force
  if(external_effort_.find(my_name) !=  external_effort_.end())
  {
    Eigen::Vector3d external_force;
    Eigen::Vector3d external_torque;
    for(uint8_t i = 0; i < 3; i++)
    {
      external_force[i] = external_effort_.at(my_name)[i];
      external_torque[i] = external_effort_.at(my_name)[i+3];
    }
    *my_force -= external_force;
    *my_torque -= external_torque;
  }

  // add Child Force and Torque
  for(uint8_t i = 0; i < manipulator.getComponentChildName(my_name).size(); i++)
  {
    Name child_name = manipulator.getComponentChildName(my_name).at(i);
    Eigen::Vector3d child_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d child_torque = Eigen::Vector3d::Zero();
    auto child_joint_torque = getJointTorqueSingleComponent(manipulator, child_name, &child_force, &child_torque);

    joint_torque.insert(child_joint_torque.begin(), child_joint_torque.end());
    *my_force += child_force;
    *my_torque += child_torque;
  }

  Eigen::Vector3d pa;
  Eigen::Vector3d a;
  // Clac joint Torque (If not Tool Component)
  if(manipulator.checkComponentType(my_name, ACTIVE_JOINT_COMPONENT))
  {
    Name parent_name = manipulator.getComponentParentName(my_name);
    Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero();
    if(parent_name != manipulator.getWorldName())
      joint_axis = manipulator.getComponentOrientationFromWorld(parent_name) * manipulator.getAxis(my_name);
    else
      joint_axis = manipulator.getWorldOrientation() * manipulator.getAxis(my_name);
    Eigen::Vector3d position_by_axis = math::skewSymmetricMatrix(my_pose_value.kinematic.position) * joint_axis;
    double my_joint_torque = position_by_axis.dot(*my_force) + joint_axis.dot(*my_torque);
    joint_torque.insert(std::make_pair(my_name, my_joint_torque));
    pa = position_by_axis;
    a = joint_axis;
  }

  return joint_torque;
}

////////////////////////////////////////////////////////////////


//////////////////////virtual Function//////////////////////////
bool SolverUsingNewtonEuler::setOption(STRING param_name, const void *arg)
{
  return false;
}

bool SolverUsingNewtonEuler::setEnvironments(STRING param_name, const void *arg)
{
  if (param_name == "external_effort")
  {
    std::map<Name, Eigen::VectorXd> *external_effort = (std::map<Name, Eigen::VectorXd> *) arg;
    return setExterrnalEffort(*external_effort);
  }

  return false;
}

bool SolverUsingNewtonEuler::solveForwardDynamics(Manipulator *manipulator, std::map<Name, double> joint_torque)
{
  return false;
}

bool SolverUsingNewtonEuler::solveInverseDynamics(Manipulator manipulator, std::map<Name, double>* joint_torque)
{
  joint_torque->clear();
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();

  *joint_torque = getJointTorqueSingleComponent(manipulator, manipulator.getWorldChildName(), &force, &torque);
  return true;
}
////////////////////////////////////////////////////////////////
