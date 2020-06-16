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

#ifndef DYNAMICS_H
#define DYNAMICS_H

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
#endif

using namespace robotis_manipulator;
using namespace Eigen;


namespace dynamics
{

/*****************************************************************************
** Dynamics Solver Using Newton-Euler method
*****************************************************************************/
class SolverUsingNewtonEuler : public robotis_manipulator::Dynamics
{
private:
  std::map<Name, Eigen::VectorXd> external_effort_;

  bool setExterrnalEffort(std::map<Name, Eigen::VectorXd> external_effort);
  std::map<Name, double> getJointTorqueSingleComponent(Manipulator manipulator, Name my_name, Eigen::Vector3d* my_force, Eigen::Vector3d* my_torque);


public:
  SolverUsingNewtonEuler(){}
  virtual ~SolverUsingNewtonEuler(){}

  virtual bool setOption(STRING param_name, const void *arg);
  virtual bool setEnvironments(STRING param_name, const void *arg);
  virtual bool solveForwardDynamics(Manipulator *manipulator, std::map<Name, double> joint_torque);          //torque to joint value
  virtual bool solveInverseDynamics(Manipulator manipulator, std::map<Name, double> *joint_torque);          //joint values to torque
};

}



#endif // DYNAMICS_H
