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

#include "../include/social_robot_arm_libs/dynamixel.h"

using namespace dynamixel;

/*****************************************************************************
** Additional Function
*****************************************************************************/
LowPassFilter::LowPassFilter()
{
  previous_filtering_data_ = 0.0;
}

void LowPassFilter::setTau(double tau)
{
  tau_ = tau;
}

double LowPassFilter::getFilteringData(double raw_data, double sampling_time)
{
  previous_filtering_data_ = ((tau_*previous_filtering_data_) + (sampling_time * raw_data)) / (tau_ + sampling_time);
  return previous_filtering_data_;
}

void AccelerationCalculator::initialize(double start_velocity, double tau)
{
  previous_velocity_ = start_velocity;
  filter_.setTau(tau);
}

double AccelerationCalculator::getPresentAcceleration(double sampling_time, double present_velocity)
{
  double raw_acceleration = (present_velocity - previous_velocity_) / sampling_time;
  previous_velocity_ = present_velocity;

  return filter_.getFilteringData(raw_acceleration, sampling_time);
}

/*****************************************************************************
** Arms Dynamixel Class
*****************************************************************************/
ArmsDynamixel::ArmsDynamixel(float control_loop_time)
{
  control_loop_time_ = control_loop_time;
//  previous_time_ = -control_loop_time;
  dynamixel_workbench_ = new DynamixelWorkbench;

  //Indirect Setting
  write_indirect_size_ = 0;
  read_indirect_handler_.all_data_size = 0;
  read_indirect_handler_.addr_indirect_address.push_back(ADDR_READ_FIRST_INDIRECT_ADDR);
  read_indirect_handler_.addr_indirect_data.push_back(ADDR_READ_FIRST_INDIRECT_DATA);

}

ArmsDynamixel::~ArmsDynamixel()
{
  delete dynamixel_workbench_;
}

// Init Functions
bool ArmsDynamixel::initDynamixelWorkbench(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  const char* log = NULL;
  dynamixel_.id = actuator_id;
  dynamixel_.num = actuator_id.size();

  //Init
  if(!dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log))
  {
    log::error("ID: ");
    for(uint8_t i=0; i<dynamixel_.num; i++)log::print(" ", dynamixel_.id.at(i), 0, "RAD");
    log::println("");
    log::error(log);
    return false;
  }

  //set time profile enable states
  for(uint8_t i = 0; i < actuator_id.size(); i++)
    time_profile_enable_state_[actuator_id.at(i)] = false;

  return true;
}

bool ArmsDynamixel::pingToDynamixel()
{
  const char* log = NULL;
  uint16_t get_model_number;
  //ping
  for (uint8_t index = 0; index < dynamixel_.num; index++)
  {
    uint8_t id = dynamixel_.id.at(index);
    if (!dynamixel_workbench_->ping(id, &get_model_number, &log))
    {
      log::error("ID: ",id);
      log::error(log);
    }
    else
    {
      char str[100];
      sprintf(str, "Joint Dynamixel ID : %d, Model Name : %s", id, dynamixel_workbench_->getModelName(id));
      log::println(str);
    }
    previous_time_.insert(std::make_pair(id ,0.0));
  }
  return true;
}

bool ArmsDynamixel::initReturnDelayTime()
{
  bool result = true;
  const char* log = NULL;
  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();
  for (uint8_t index = 0; index < dynamixel_.num; index++)
  {
    uint8_t id = dynamixel_.id.at(index);
    if (!dynamixel_workbench_->writeRegister(id, return_delay_time_char, 0, &log))
    {
      log::error("ID: ", id);
      log::error(log);
      log::error("[Fail to set Return Delay Time] Please check your Dynamixel firmware version");
      result = false;
    }
  }
  return result;
}

bool ArmsDynamixel::initSDKHandler()
{
  const char* log = NULL;
  bool result = false;

  //set Indirect Write Address
  addIndirectAddress(ADDR_GOAL_POSITION, LENGTH_GOAL_POSITION, WRITE);
  addIndirectAddress(ADDR_GOAL_CURRENT, LENGTH_GOAL_CURRENT, WRITE);
  //Set Indirect Read Address
  addIndirectAddress(ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION, READ);
  addIndirectAddress(ADDR_PRESENT_VELOCITY, LENGTH_PRESENT_VELOCITY, READ);
  addIndirectAddress(ADDR_PRESENT_CURRENT, LENGTH_PRESENT_CURRENT, READ);
  addIndirectAddress(ADDR_REALTIME_TICK, LENGTH_REALTIME_TICK, READ);

  //wirte Handler
  result = dynamixel_workbench_->addSyncWriteHandler(ADDR_WRITE_FIRST_INDIRECT_DATA, 4*write_indirect_size_, &log);
  if (result == false)
  {
    log::error(log);
  }
  //reed Handler
  result = dynamixel_workbench_->addSyncReadHandler(read_indirect_handler_.addr_indirect_data.front(), read_indirect_handler_.all_data_size, &log);
  if (result == false)
  {
    log::error(log);
  }
  //wirte torque enable Handler
  result = dynamixel_workbench_->addSyncWriteHandler(ADDR_TORQUE_ENABLE, LENGTH_TORQUE_ENABLE, &log);
  if (result == false)
  {
    log::error(log);
  }
  //wirte operating mode Handler
  result = dynamixel_workbench_->addSyncWriteHandler(ADDR_OPERATING_MODE, LENGTH_OPERATING_MODE, &log);
  if (result == false)
  {
    log::error(log);
  }

  return true & result;
}

bool ArmsDynamixel::initAccelerationCalculator()
{
  for(int index = 0; index < dynamixel_.num; index++)
  {
    //acceleration calculator
    AccelerationCalculator acceleration_calculator;
    acceleration_calculator.initialize(0.0);
    //low pass filter
    LowPassFilter filter;
    filter.setTau();
    //number parameter set
    acceleration_calculator_.insert(std::make_pair(dynamixel_.id.at(index), acceleration_calculator));
    velocity_filter_.insert(std::make_pair(dynamixel_.id.at(index), filter));
  }
  return true;
}

bool ArmsDynamixel::addIndirectAddress(uint16_t item_address, uint8_t item_length, uint8_t write_or_read)
{
  const char* log = NULL;
  uint8_t p_item_address[2];

  switch (write_or_read) {
  ///////////////Write Indirect Address//////////
  case WRITE:
    for(uint8_t num = 0; num < 4; num++)
    {
      if(num >= 4-item_length)
      {
        p_item_address[0] = DXL_LOBYTE(item_address);
        p_item_address[1] = DXL_HIBYTE(item_address);
        for(uint8_t i = 0; i < dynamixel_.num; i++)
        {
          uint8_t id = dynamixel_.id.at(i);
          dynamixel_workbench_->writeRegister(id, ADDR_WRITE_FIRST_INDIRECT_ADDR + (4*write_indirect_size_) +(LENGTH_INDIRECT_ADDR*num),
                                                LENGTH_INDIRECT_ADDR, (uint8_t*) p_item_address, &log);
        }
        item_address += 1;
      }
    }
    write_indirect_size_ += 1;
    return true;
  //////////////////////////////////////////////
  ///////////////Read Indirect Address//////////
  case READ:
    for(uint8_t num = 0; num < item_length; num++)
    {
      p_item_address[0] = DXL_LOBYTE(item_address);
      p_item_address[1] = DXL_HIBYTE(item_address);
      for(uint8_t i = 0; i < dynamixel_.num; i++)
      {
        uint8_t id = dynamixel_.id.at(i);
        dynamixel_workbench_->writeRegister(id, read_indirect_handler_.addr_indirect_address.back() + (LENGTH_INDIRECT_ADDR*num),
                                              LENGTH_INDIRECT_ADDR, (uint8_t*) p_item_address, &log);
      }
      item_address += 1;
    }
    read_indirect_handler_.item_data_size.push_back(item_length);
    read_indirect_handler_.all_data_size += item_length;
    read_indirect_handler_.addr_indirect_address.push_back(read_indirect_handler_.addr_indirect_address.back() + LENGTH_INDIRECT_ADDR*item_length);
    read_indirect_handler_.addr_indirect_data.push_back(read_indirect_handler_.addr_indirect_data.back() + LENGTH_INDIRECT_DATA*item_length);
    return true;
  ////////////////////////////////////////////////
  default:
    return false;
  }
}

// Set Mode Function
bool ArmsDynamixel::setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  for(uint8_t num = 0; num < actuator_id.size(); num++)
  {
    //Profile mode
    if(!dynamixel_workbench_->setTimeBasedProfile(actuator_id.at(num), &log))
    {
      log::error("ID: ",actuator_id.at(num));
      log::error(log);
      log::error("Please check your Dynamixel firmware version (v42~)");
    }
    else
      time_profile_enable_state_[actuator_id.at(num)] = true;

    //set Profile data
    uint32_t velocity = 0;
    uint32_t acceleration = 0;
    uint32_t current = 0;
    if(time_profile_enable_state_.at(actuator_id.at(num)))
    {
      velocity = uint32_t(control_loop_time_*1000) * 3;
      acceleration = uint32_t(control_loop_time_*1000);
      current = 0;
    }
    else
    {
      velocity = 0;
      acceleration = 0;
      current = 0;
    }
    auto number = dynamixel_workbench_->getModelNumber(actuator_id.at(num));

    //Operating Mode
    if(dynamixel_mode == "position_mode")
    {
      if(!dynamixel_workbench_->setPositionControlMode(actuator_id.at(num), &log))
      {
        log::error("ID: ",actuator_id.at(num));
        log::error(log);
      }
      if(!dynamixel_workbench_->writeRegister(actuator_id.at(num), "Profile_Velocity", velocity, &log))
      {
        log::error("ID: ",actuator_id.at(num));
        log::error(log);
      }
      if(!dynamixel_workbench_->writeRegister(actuator_id.at(num), "Profile_Acceleration", acceleration, &log))
      {
        log::error("ID: ",actuator_id.at(num));
        log::error(log);
      }
    }
    else if(dynamixel_mode == "current_based_position_mode")
    {
      if(!dynamixel_workbench_->setCurrentBasedPositionControlMode(actuator_id.at(num), &log))
      {
        log::error("ID: ",actuator_id.at(num));
        log::error(log);
      }
      if(!dynamixel_workbench_->writeRegister(actuator_id.at(num), "Profile_Velocity", velocity, &log))
      {
        log::error("ID: ",actuator_id.at(num));
        log::error(log);
      }
      if(!dynamixel_workbench_->writeRegister(actuator_id.at(num), "Profile_Acceleration", acceleration, &log))
      {
        log::error("ID: ",actuator_id.at(num));
        log::error(log);
      }
    }
    else if(dynamixel_mode == "current_mode")
    {
      if(!dynamixel_workbench_->setCurrentControlMode(actuator_id.at(num), &log))
      {
        log::error("ID: ",actuator_id.at(num));
        log::error(log);
      }

    }
    else
    {
      log::error("Please Set Operating Mode to Position or Current Based Position Mode");
    }
  }
  return true;
}

bool ArmsDynamixel::setGavityCompenMode()
{
  bool result = false;
  const char* log = NULL;

  log::info("disable start");
  disable();
  setOperatingMode(dynamixel_.id,"current_mode");
  enable();
  log::info("enable end");
}

bool ArmsDynamixel::setNormalMode()
{
  bool result = false;
  const char* log = NULL;

  log::info("disable start");
  disable();
  setOperatingMode(dynamixel_.id,"position_mode");
  enable();
  log::info("enable end");
}

bool ArmsDynamixel::setItem(std::vector<uint8_t> actuator_id, STRING item_name, uint32_t value)
{
  const char* log = NULL;
  bool result = true;

  const char * char_item_name = item_name.c_str();
  for(uint8_t num = 0; num < actuator_id.size(); num++)
  {
    if(!dynamixel_workbench_->writeRegister(actuator_id.at(num), char_item_name, value, &log))
    {
      log::error("ID: ",actuator_id.at(num));
      log::error(log);
      result = false;
    }
  }
  return result;
}

//write value Function
float ArmsDynamixel::calcGoalPositionForTimeBaseProfile(ActuatorValue value)
{
  return value.position + 3*(value.velocity * (control_loop_time_))/2;
}

float ArmsDynamixel::calcGoalCurrentFromGoalEffort(uint8_t id, ActuatorValue value)
{
  // A to mA
  float current = value.effort * 1000.0;
  return current;
}

bool ArmsDynamixel::writeGoalValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[actuator_id.size()];
  int32_t goal_value[4*write_indirect_size_*actuator_id.size()];
  for(uint8_t index = 0; index < actuator_id.size(); index++)
  {
    //Set position
    float result_position;
    if(time_profile_enable_state_.at(actuator_id.at(index)))
      result_position = calcGoalPositionForTimeBaseProfile(value_vector.at(index));   //add tarajectory eq.
    else
      result_position = value_vector.at(index).position;

    //Set current
    float result_current;
    result_current = calcGoalCurrentFromGoalEffort(index, value_vector.at(index));

    //Convert ID
    id_array[index] = actuator_id.at(index);

    //Convert Data
    goal_value[2*index] = dynamixel_workbench_->convertRadian2Value(actuator_id.at(index), result_position);    //position
    goal_value[2*index+1] = dynamixel_workbench_->convertCurrent2Value(actuator_id.at(index), result_current);  //current

//    printf("[ID:%d]p= %lf,\t p2= %d,\t i= %lf,\t i2= %d\n",actuator_id.at(index),result_position*RAD2DEG,goal_value[2*index],result_current,goal_value[2*index+1]);

    // Limit Check
    if(goal_value[2*index+1] < -2047)
      goal_value[2*index+1] = -2047;
    else if(goal_value[2*index+1] > 2047)
      goal_value[2*index+1] = 2047;

    //save precious
    previous_goal_value_[actuator_id.at(index)] = value_vector.at(index);
  }

  result = dynamixel_workbench_->syncWrite(0, id_array, (uint8_t) actuator_id.size(), goal_value, write_indirect_size_, &log);
  if (result == false)
  {
    log::error("ID: ");
    for(uint8_t i=0; i<actuator_id.size(); i++)log::print(" ", actuator_id.at(i), 0);
    log::error(log);
  }
  return true;
}

double ArmsDynamixel::calcPresentAcceleration(uint8_t id, double* velocity, int32_t realtime_tick)
{
  //Time
  double sampling_time;
  double present_time = (double) realtime_tick;
  present_time = present_time / 1000;
  if(previous_time_.at(id) > present_time)
    sampling_time = present_time - previous_time_.at(id) + 32.767;
  else
    sampling_time = present_time - previous_time_.at(id);
  previous_time_.at(id) = previous_time_.at(id) + sampling_time;

  //velocity filtering
  *velocity = velocity_filter_.at(id).getFilteringData(*velocity, sampling_time);

  //Calc acceleration
  return acceleration_calculator_.at(id).getPresentAcceleration(sampling_time, *velocity);
}

double ArmsDynamixel::calcPresentEffortFromPresentCurrent(ActuatorValue value)
{
  // mA to A
  return value.effort / 1000;
}

std::vector<robotis_manipulator::ActuatorValue> ArmsDynamixel::receivePresentValue(std::vector<uint8_t> actuator_id)
{
  bool result = false;
  const char* log = NULL;

  std::vector<robotis_manipulator::ActuatorValue> all_actuator;

  uint8_t id_array[actuator_id.size()];
  for (uint8_t index = 0; index < actuator_id.size(); index++)
    id_array[index] = actuator_id.at(index);

  //sync read
  result = dynamixel_workbench_->syncRead(0, id_array, actuator_id.size(), &log);
  if (result == false)
  {
    log::error("ID: ");
    for(uint8_t i=0; i<actuator_id.size(); i++)log::print(" ", actuator_id.at(i), 0);
    log::error(log);
  }
  //get position
  int32_t get_position[actuator_id.size()];
  result = dynamixel_workbench_->getSyncReadData(0, id_array, actuator_id.size(),
                                                 read_indirect_handler_.addr_indirect_data.at(0),
                                                 read_indirect_handler_.item_data_size.at(0),
                                                 get_position,
                                                 &log);
  if (result == false)
  {
    log::error("ID: ");
    for(uint8_t i=0; i<actuator_id.size(); i++)log::print(" ", actuator_id.at(i), 0);
    log::error(log);
  }

  //get Velocity
  int32_t get_velocity[actuator_id.size()];
  result = dynamixel_workbench_->getSyncReadData(0, id_array, actuator_id.size(),
                                                 read_indirect_handler_.addr_indirect_data.at(1),
                                                 read_indirect_handler_.item_data_size.at(1),
                                                 get_velocity,
                                                 &log);
  if (result == false)
  {
    log::error("ID: ");
    for(uint8_t i=0; i<actuator_id.size(); i++)log::print(" ", actuator_id.at(i), 0);
    log::error(log);
  }

  //get current
  int32_t get_current[actuator_id.size()];
  result = dynamixel_workbench_->getSyncReadData(0, id_array, actuator_id.size(),
                                                 read_indirect_handler_.addr_indirect_data.at(2),
                                                 read_indirect_handler_.item_data_size.at(2),
                                                 get_current,
                                                 &log);
  if (result == false)
  {
    log::error("ID: ");
    for(uint8_t i=0; i<actuator_id.size(); i++)log::print(" ", actuator_id.at(i), 0);
    log::error(log);
  }

  //get Real Time Tick
  int32_t get_realtime_tick[actuator_id.size()];
  result = dynamixel_workbench_->getSyncReadData(0, id_array, actuator_id.size(),
                                                 read_indirect_handler_.addr_indirect_data.at(3),
                                                 read_indirect_handler_.item_data_size.at(3),
                                                 get_realtime_tick,
                                                 &log);
  if (result == false)
  {
    log::error("ID: ");
    for(uint8_t i=0; i<actuator_id.size(); i++)log::print(" ", actuator_id.at(i), 0);
    log::error(log);
  }

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    robotis_manipulator::ActuatorValue actuator;
    actuator.position = dynamixel_workbench_->convertValue2Radian(actuator_id.at(index), get_position[index]);
    actuator.velocity = dynamixel_workbench_->convertValue2Velocity(actuator_id.at(index), get_velocity[index]);
    actuator.acceleration = calcPresentAcceleration(actuator_id.at(index), &actuator.velocity, get_realtime_tick[index]);
    actuator.effort = dynamixel_workbench_->convertValue2Current(get_current[index]);

    //Calc Torque
    actuator.effort = calcPresentEffortFromPresentCurrent(actuator);

    all_actuator.push_back(actuator);
  }

//  log::print("reciv effor","GREEN");
//  for(uint8_t i = 0; i < all_actuator.size(); i++)
//  {
//    log::print("\t",all_actuator.at(i).effort,3,"GREEN");
//  }
//  log::println("");

  return all_actuator;
}


// Virtual Functions
void ArmsDynamixel::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;
  STRING dxl_device_name = get_arg_[0];
  STRING dxl_baud_rate = get_arg_[1];

  // Dynamixel Workbech Init
  initDynamixelWorkbench(actuator_id, dxl_device_name, dxl_baud_rate);
  //ping
  pingToDynamixel();
  //Set return delay time
  initReturnDelayTime();
  //set Indirect Address
  initSDKHandler();
  //acceleration calculator parameter init
  initAccelerationCalculator();
  //disable
  disable();

  return;
}

void ArmsDynamixel::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  if(get_arg_[0] == "gravity_compensation_mode")
  {
    setGavityCompenMode();
  }
  else if(get_arg_[0] == "normal_mode")
  {
    setNormalMode();
  }
  else if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode" || get_arg_[0] == "current_mode")
  {
    setOperatingMode(actuator_id, get_arg_[0]);
  }
  else
  {
    setItem(actuator_id, get_arg_[0], std::atoi(get_arg_[1].c_str()));
  }
  return;
}

std::vector<uint8_t> ArmsDynamixel::getId()
{
  return dynamixel_.id;
}

void ArmsDynamixel::enable()
{
  const char* log = NULL;
  bool result = false;

  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error("ID: ",dynamixel_.id.at(index));
      log::error(log);
    }
  }
  enabled_state_ = true;
}

void ArmsDynamixel::disable()
{
  const char* log = NULL;
  bool result = false;

  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(index), &log);

    if (result == false)
    {
      log::error("ID: ",dynamixel_.id.at(index));
      log::error(log);
    }
  }
  enabled_state_ = false;
}

bool ArmsDynamixel::sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;

  result = ArmsDynamixel::writeGoalValue(actuator_id, value_vector);
  if (result == false)
    return false;

  return true;
}

std::vector<robotis_manipulator::ActuatorValue> ArmsDynamixel::receiveJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  return ArmsDynamixel::receivePresentValue(actuator_id);
}



/*****************************************************************************
** Hnads Dynamixel Class
*****************************************************************************/

// Virtual Functions Hands Dynamixel
void HandsDynamixel::init(uint8_t actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = HandsDynamixel::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void HandsDynamixel::setMode(const void *arg)
{
  bool result = false;
// const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = HandsDynamixel::setOperatingMode(get_arg_[0]);
    if (result == false)
      return;

    result = HandsDynamixel::setSDKHandler();
    if (result == false)
      return;
  }
  else
  {
    result = HandsDynamixel::writeProfileValue(get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }
}

uint8_t HandsDynamixel::getId()
{
  return dynamixel_.id.at(0);
}

void HandsDynamixel::enable()
{
  const char* log = NULL;
  bool result = false;

  result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }
  enabled_state_ = true;
}

void HandsDynamixel::disable()
{
  const char* log = NULL;
  bool result = false;

  result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }
  enabled_state_ = false;
}

bool HandsDynamixel::sendToolActuatorValue(robotis_manipulator::ActuatorValue value)
{
  return HandsDynamixel::writeGoalPosition(value.position);
}

robotis_manipulator::ActuatorValue HandsDynamixel::receiveToolActuatorValue()
{
  robotis_manipulator::ActuatorValue result;
  result.position = HandsDynamixel::receiveDynamixelValue();
  result.velocity = 0.0;
  result.acceleration = 0.0;
  result.effort = 0.0;
  return result;
}

// Private Functions Hands Dynamixel
bool HandsDynamixel::initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  const char* log = NULL;
  bool result = false;

  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();

  dynamixel_.id.push_back(actuator_id);
  dynamixel_.num = 1;

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }

  uint16_t get_model_number;
  result = dynamixel_workbench_->ping(dynamixel_.id.at(0), &get_model_number, &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
    log::error("Please check your Dynamixel ID");
  }
  else
  {
    char str[100];
    sprintf(str, "Gripper Dynamixel ID : %d, Model Name :", dynamixel_.id.at(0));
    strcat(str, dynamixel_workbench_->getModelName(dynamixel_.id.at(0)));
    log::println(str);

//    result = dynamixel_workbench_->setVelocityBasedProfile(dynamixel_.id.at(0), &log);
//    if(result == false)
//    {
//      log::error(log);
//      log::error("Please check your Dynamixel firmware version (v38~)");
//    }

    result = dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), return_delay_time_char, 0, &log);
    if (result == false)
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
      log::error("Please check your Dynamixel firmware version");
    }
  }
  //disable
  disable();
  return true;
}

bool HandsDynamixel::setOperatingMode(STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t acceleration = 0;
  const uint32_t current = 200;

  if (dynamixel_mode == "position_mode")
  {
    result = dynamixel_workbench_->setPositionControlMode(dynamixel_.id.at(0), &log);
    if (result == false)
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
    if(!dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), "Profile_Velocity", velocity, &log))
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
    if(!dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), "Profile_Acceleration", acceleration, &log))
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    result = dynamixel_workbench_->setCurrentBasedPositionControlMode(dynamixel_.id.at(0), &log);
    if (result == false)
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
    if(!dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), "Goal_Current", current, &log))
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
    if(!dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), "Profile_Velocity", velocity, &log))
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
    if(!dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), "Profile_Acceleration", acceleration, &log))
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
  }
  else
  {
    result = dynamixel_workbench_->setPositionControlMode(dynamixel_.id.at(0), &log);
    if (result == false)
    {
      log::error("ID: ",dynamixel_.id.at(0));
      log::error(log);
    }
  }

  return true;
}

bool HandsDynamixel::writeProfileValue(STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  result = dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), char_profile_mode, value, &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }

  return true;
}

bool HandsDynamixel::setSDKHandler()
{
  bool result = false;
  const char* log = NULL;

  result = dynamixel_workbench_->addSyncWriteHandler(dynamixel_.id.at(0), "Goal_Position", &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(dynamixel_.id.at(0),
                                                    "Present_Position",
                                                    &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }

  return true;
}

bool HandsDynamixel::writeGoalPosition(double radian)
{
  bool result = false;
  const char* log = NULL;

  int32_t goal_position = 0;

  goal_position = dynamixel_workbench_->convertRadian2Value(dynamixel_.id.at(0), radian);

  result = dynamixel_workbench_->syncWrite(0, &goal_position, &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }

  return true;
}

double HandsDynamixel::receiveDynamixelValue()
{
  bool result = false;
  const char* log = NULL;

  int32_t get_value = 0;
  uint8_t id_array[1] = {dynamixel_.id.at(0)};
  result = dynamixel_workbench_->syncRead(0,
                                          id_array,
                                          (uint8_t)1,
                                          &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }
  result = dynamixel_workbench_->getSyncReadData(0,
                                            id_array,
                                            (uint8_t)1,
                                            &get_value,
                                            &log);
  if (result == false)
  {
    log::error("ID: ",dynamixel_.id.at(0));
    log::error(log);
  }
  return dynamixel_workbench_->convertValue2Radian(dynamixel_.id.at(0), get_value);
}

