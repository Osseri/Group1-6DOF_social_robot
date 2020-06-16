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

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
  #include <DynamixelWorkbench.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
  #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#endif

using namespace robotis_manipulator;

namespace dynamixel
{
// Index
#define WRITE 0
#define READ 1

// Dynamixel X-Serise address (Protocol 2.0)
#define ADDR_TORQUE_ENABLE 64
#define ADDR_OPERATING_MODE 11

#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY 112

#define ADDR_PRESENT_CURRENT 126
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132
#define ADDR_REALTIME_TICK 120

#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_POSITION 116

#define ADDR_WRITE_FIRST_INDIRECT_ADDR 168
#define ADDR_WRITE_FIRST_INDIRECT_DATA 224

#define ADDR_READ_FIRST_INDIRECT_ADDR 578
#define ADDR_READ_FIRST_INDIRECT_DATA 634

// Dynamixel X-Serise length (Protocol 2.0)
#define LENGTH_TORQUE_ENABLE 1
#define LENGTH_OPERATING_MODE 1

#define LENGTH_PROFILE_ACCELERATION 4
#define LENGTH_PROFILE_VELOCITY 4

#define LENGTH_PRESENT_CURRENT 2
#define LENGTH_PRESENT_VELOCITY 4
#define LENGTH_PRESENT_POSITION 4
#define LENGTH_REALTIME_TICK 2

#define LENGTH_GOAL_CURRENT 2
#define LENGTH_GOAL_POSITION 4

#define LENGTH_INDIRECT_ADDR 2
#define LENGTH_INDIRECT_DATA 1

/*****************************************************************************
** Joint Structure
*****************************************************************************/
typedef struct
{
  std::vector<uint8_t> id;
  uint8_t num;
} Joint;

typedef struct
{
  std::vector<uint8_t> item_data_size;
  uint16_t all_data_size = 0;
  std::vector<uint16_t> addr_indirect_address;
  std::vector<uint16_t> addr_indirect_data;
} IndirectHandler;

/*****************************************************************************
** Low Pass Filter for calculating acceleration
*****************************************************************************/
class LowPassFilter
{
private:
  double tau_;
  double previous_filtering_data_;

public:
  LowPassFilter();
  ~LowPassFilter(){}

  void setTau(double tau = 0.0005);
  double getFilteringData(double raw_data, double sampling_time);
};

/*****************************************************************************
** Acceleration calculator
*****************************************************************************/
class AccelerationCalculator
{
private:
  double previous_velocity_;
  LowPassFilter filter_;

public:
  AccelerationCalculator(){}
  ~AccelerationCalculator(){}

  void initialize(double start_velocity, double tau = 0.005);
  double getPresentAcceleration(double sampling_time, double present_velocity);
};

/*****************************************************************************
** Arms Dynamixel
*****************************************************************************/
//using time base profile
//using current based position control
//set return delay time to 0
//write goal current and position
//read present current, velocity, position and realtime tick
class ArmsDynamixel : public robotis_manipulator::JointActuator
{
private:
  //dynamixel
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;

  //Indirect Address
  IndirectHandler read_indirect_handler_;
  uint8_t write_indirect_size_;

  //for calculating goal position overwrited to profile control
  std::map<uint8_t ,bool> time_profile_enable_state_;
  float control_loop_time_; // unit: ms
  std::map<uint8_t, robotis_manipulator::ActuatorValue> previous_goal_value_;

  //for calculating present acceleration from velocity received from dynamixel
  std::map<uint8_t ,double> previous_time_;
  std::map<uint8_t, AccelerationCalculator> acceleration_calculator_;
  std::map<uint8_t,LowPassFilter> velocity_filter_;

  // init Functions
  bool initDynamixelWorkbench(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool pingToDynamixel();
  bool initReturnDelayTime();
  bool initSDKHandler();
  bool initAccelerationCalculator();
  bool addIndirectAddress(uint16_t item_address, uint8_t item_length, uint8_t write_or_read);

  //set mode Function
  bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
  bool setGavityCompenMode();
  bool setNormalMode();
  bool setItem(std::vector<uint8_t> actuator_id, STRING item_name, uint32_t value);

  //write value Function
  float calcGoalPositionForTimeBaseProfile(ActuatorValue value);
  float calcGoalCurrentFromGoalEffort(uint8_t id, ActuatorValue value);
  bool writeGoalValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);

  //reed value Function
  double calcPresentAcceleration(uint8_t id, double *velocity, int32_t realtime_tick);
  double calcPresentEffortFromPresentCurrent(ActuatorValue value);
  std::vector<robotis_manipulator::ActuatorValue> receivePresentValue(std::vector<uint8_t> actuator_id);

public:
  ArmsDynamixel(float control_loop_time = 0.010);
  virtual ~ArmsDynamixel();

  // Virtual Functions
  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();
  virtual void enable();
  virtual void disable();
  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
  virtual std::vector<robotis_manipulator::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);
};


/*****************************************************************************
** Hands Dynamixel
*****************************************************************************/
class HandsDynamixel : public robotis_manipulator::ToolActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;

  // private functions
  bool initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(STRING dynamixel_mode = "position_mode");
  bool writeProfileValue(STRING profile_mode, uint32_t value);
  bool setSDKHandler();
  bool writeGoalPosition(double radian);
  double receiveDynamixelValue();

 public:
  HandsDynamixel() {}
  virtual ~HandsDynamixel() {}
  // virtual Functions
  virtual void init(uint8_t actuator_id, const void *arg);
  virtual void setMode(const void *arg);
  virtual uint8_t getId();
  virtual void enable();
  virtual void disable();
  virtual bool sendToolActuatorValue(robotis_manipulator::ActuatorValue value);
  virtual robotis_manipulator::ActuatorValue receiveToolActuatorValue();
};

}
#endif // DYNAMIXEL_H_
