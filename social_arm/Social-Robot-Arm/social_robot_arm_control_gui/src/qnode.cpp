
/*****************************************************************************
** Includes
*****************************************************************************/

#include "social_robot_arm_control_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace social_robot_arm_control_gui {

Eigen::Vector3d convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion)
{
  Eigen::Vector3d rpy = convertRotationMatrixToRPYVector(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw)
{
 Eigen::Quaterniond quaternion;
 quaternion = convertRPYToRotationMatrix(roll,pitch,yaw);

 return quaternion;
}

Eigen::Vector3d convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation)
{
  Eigen::Vector3d rpy;// = Eigen::MatrixXd::Zero(3,1);
  rpy.coeffRef(0,0) = atan2(rotation.coeff(2,1), rotation.coeff(2,2));
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1), 2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2 (rotation.coeff(1,0), rotation.coeff(0,0));

  return rpy;
}

Eigen::Matrix3d convertRPYToRotationMatrix(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d rotation = convertYawAngleToRotationMatrix(yaw)*convertPitchAngleToRotationMatrix(pitch)*convertRollAngleToRotationMatrix(roll);

  return rotation;
}

Eigen::Matrix3d convertRollAngleToRotationMatrix(double angle)
{
  Eigen::Matrix3d rotation(3,3);
  rotation <<
      1.0, 0.0, 0.0,
      0.0, cos(angle), -sin(angle),
      0.0, sin(angle), cos(angle);

  return rotation;
}

Eigen::Matrix3d convertPitchAngleToRotationMatrix(double angle)
{
  Eigen::Matrix3d rotation(3,3);
  rotation <<
      cos(angle), 0.0, sin(angle),
      0.0, 1.0, 0.0,
      -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::Matrix3d convertYawAngleToRotationMatrix(double angle)
{
  Eigen::Matrix3d rotation(3,3);
  rotation <<
      cos(angle), -sin(angle), 0.0,
      sin(angle), cos(angle), 0.0,
      0.0, 0.0, 1.0;

  return rotation;
}

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) :
  init_argc_(argc),
  init_argv_(argv),
  left_arm_moving_state_(false),
  right_arm_moving_state_(false),
  left_arm_actuator_state_(false),
  right_arm_actuator_state_(false),
  mode_state_("Mode")
{}

QNode::~QNode() {
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init(std::string left_hand_name_for_pose, std::string right_hand_name_for_pose) {
  //ROS Start
  ros::init(init_argc_,init_argv_,"social_robot_arm_control_gui");
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  left_hand_name_for_pose_ = left_hand_name_for_pose;
  right_hand_name_for_pose_ = right_hand_name_for_pose;

  // Publisher
  e_stop_pub_ = n.advertise<std_msgs::Empty>("e_stop", 10);
  set_mode_pub_ = n.advertise<social_robot_arm_msgs::SetMode>("set_mode", 10);
  motion_save_pub_ = n.advertise<social_robot_arm_msgs::MotionSave>("motion_save", 10);
  motion_control_pub_ = n.advertise<social_robot_arm_msgs::MotionControl>("motion_control", 10);

  // Subscriber
  left_arm_states_sub_ = n.subscribe("left_arm/states", 10, &QNode::updateLeftArmStates, this);
  right_arm_states_sub_ = n.subscribe("right_arm/states", 10, &QNode::updateRightArmStates, this);
  left_arm_joint_states_sub_ = n.subscribe("left_arm/joint_states", 10, &QNode::updateLeftArmJointStates, this);
  right_arm_joint_states_sub_ = n.subscribe("right_arm/joint_states", 10, &QNode::updateRightArmJointStates, this);
  left_hands_kinematics_pose_sub_ = n.subscribe(left_hand_name_for_pose_ + "/kinematics_pose", 10, &QNode::updateLeftHandsKinematicPose, this);
  right_hands_kinematics_pose_sub_ = n.subscribe(right_hand_name_for_pose_ + "/kinematics_pose", 10, &QNode::updateRightHandsKinematicPose, this);
  motion_state_sub_ = n.subscribe("mode_state", 10, &QNode::updateModeState, this);

  // Client
  goal_joint_space_path_client_ = n.serviceClient<social_robot_arm_msgs::SetJointPosition>("goal_joint_space_path");
  goal_task_space_path_client_ = n.serviceClient<social_robot_arm_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_pose");
  goal_task_space_path_client_ = n.serviceClient<social_robot_arm_msgs::SetKinematicsPose>("goal_task_space_path");
  set_moveit_joint_position_client_ = n.serviceClient<social_robot_arm_msgs::SetJointPosition>("moveit/set_joint_position");
  set_moveit_kinematics_pose_client_ = n.serviceClient<social_robot_arm_msgs::SetKinematicsPose>("moveit/set_kinematics_pose");
  goal_tool_control_client_ = n.serviceClient<social_robot_arm_msgs::SetJointPosition>("goal_tool_control");
  set_actuator_state_client_ = n.serviceClient<social_robot_arm_msgs::SetActuatorState>("set_actuator_state");

  start();
  return true;
}

void QNode::run() {
  // rate
  ros::Rate loop_rate(50);
  //loop
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "] ";

  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

/*********************
 ** Data Get Function
 **********************/
std::map<std::string, double> QNode::getLeftArmJointAngle()
{
  return left_arm_joint_angle_;
}

std::map<std::string, double> QNode::getRightArmJointAngle()
{
  return right_arm_joint_angle_;
}

std::vector<double> QNode::getLeftHandKinematicsPose()
{
  std::vector<double> result;

  result.push_back(left_hand_kinematics_pose_.pose.position.x);
  result.push_back(left_hand_kinematics_pose_.pose.position.y);
  result.push_back(left_hand_kinematics_pose_.pose.position.z);

  Eigen::Quaterniond quat(left_hand_kinematics_pose_.pose.orientation.w,
                         left_hand_kinematics_pose_.pose.orientation.x,
                         left_hand_kinematics_pose_.pose.orientation.y,
                         left_hand_kinematics_pose_.pose.orientation.z);

  Eigen::Vector3d rpy = convertQuaternionToRPYVector(quat);

  result.push_back(rpy(0));
  result.push_back(rpy(1));
  result.push_back(rpy(2));
  return result;
}

std::vector<double> QNode::getRightHandKinematicsPose()
{
  std::vector<double> result;

  result.push_back(right_hand_kinematics_pose_.pose.position.x);
  result.push_back(right_hand_kinematics_pose_.pose.position.y);
  result.push_back(right_hand_kinematics_pose_.pose.position.z);

  Eigen::Quaterniond quat(right_hand_kinematics_pose_.pose.orientation.w,
                         right_hand_kinematics_pose_.pose.orientation.x,
                         right_hand_kinematics_pose_.pose.orientation.y,
                         right_hand_kinematics_pose_.pose.orientation.z);

  Eigen::Vector3d rpy = convertQuaternionToRPYVector(quat);

  result.push_back(rpy(0));
  result.push_back(rpy(1));
  result.push_back(rpy(2));
  return result;
}

bool QNode::usingLeftArm()
{
  if(left_arm_states_sub_.getNumPublishers()>0)
    return true;
  else
    return false;
}

bool QNode::usingRightArm()
{
  if(right_arm_states_sub_.getNumPublishers()>0)
    return true;
  else
    return false;
}

bool QNode::getLeftArmMovingState()
{
  return left_arm_moving_state_;
}

bool QNode::getRightArmMovingState()
{
  return right_arm_moving_state_;
}

bool QNode::getLeftArmActuatorState()
{
  return left_arm_actuator_state_;
}

bool QNode::getRightArmActuatorState()
{
  return right_arm_actuator_state_;
}

std::string QNode::getModeState()
{
  return mode_state_;
}

uint32_t QNode::getMotionStorageSize()
{
  return motion_storage_.left_arm_motion.size();
}

std::map<std::string, double> QNode::getMotionStorage(uint32_t index)
{
  std::map<std::string, double> result;

  if(motion_storage_.left_arm_motion.size() <= index)
    return result;

  for(int i=0; i<motion_storage_.left_arm_motion.at(index).name.size(); i++)
  {
    result.insert(std::make_pair(motion_storage_.left_arm_motion.at(index).name.at(i), motion_storage_.left_arm_motion.at(index).position.at(i)));
  }
  for(int i=0; i<motion_storage_.right_arm_motion.at(index).name.size(); i++)
  {
    result.insert(std::make_pair(motion_storage_.right_arm_motion.at(index).name.at(i), motion_storage_.right_arm_motion.at(index).position.at(i)));
  }
  result.insert(std::make_pair("time", motion_storage_.path_time.at(index)));

  return result;
}

bool QNode::saveMotion(uint32_t index, double time)
{
  if(left_arm_joint_angle_.empty() && right_arm_joint_angle_.empty())
  {
    log( Info , "Robot is not enabled.");
    return false;
  }
  sensor_msgs::JointState left_arm_joint_angle;
  if(!left_arm_joint_angle_.empty())
  {
    for(auto it=left_arm_joint_angle_.begin(); it!=left_arm_joint_angle_.end();it++)
    {
      left_arm_joint_angle.name.push_back(it->first);
      left_arm_joint_angle.position.push_back(it->second);
      left_arm_joint_angle.velocity.push_back(0.0);
      left_arm_joint_angle.effort.push_back(0.0);
    }
    for(auto it=left_hand_position_.begin(); it!=left_hand_position_.end();it++)
    {
      left_arm_joint_angle.name.push_back(it->first);
      left_arm_joint_angle.position.push_back(it->second);
      left_arm_joint_angle.velocity.push_back(0.0);
      left_arm_joint_angle.effort.push_back(0.0);
    }
  }
  motion_storage_.left_arm_motion.insert(motion_storage_.left_arm_motion.begin()+index,left_arm_joint_angle);

  sensor_msgs::JointState right_arm_joint_angle;
  if(!right_arm_joint_angle_.empty())
  {
    for(auto it=right_arm_joint_angle_.begin(); it!=right_arm_joint_angle_.end();it++)
    {
      right_arm_joint_angle.name.push_back(it->first);
      right_arm_joint_angle.position.push_back(it->second);
      right_arm_joint_angle.velocity.push_back(0.0);
      right_arm_joint_angle.effort.push_back(0.0);
    }
    for(auto it=right_hand_position_.begin(); it!=right_hand_position_.end();it++)
    {
      right_arm_joint_angle.name.push_back(it->first);
      right_arm_joint_angle.position.push_back(it->second);
      right_arm_joint_angle.velocity.push_back(0.0);
      right_arm_joint_angle.effort.push_back(0.0);
    }
  }
  motion_storage_.right_arm_motion.insert(motion_storage_.right_arm_motion.begin()+index,right_arm_joint_angle);

  motion_storage_.path_time.insert(motion_storage_.path_time.begin()+index, time);
  log( Info , "Motion Saved.");
  return true;
}

bool QNode::saveMotion(social_robot_arm_msgs::MotionSave motion_storage)
{
  motion_storage_ = motion_storage;
  log( Info , "Motion Read from YAML.");
  return true;
}

bool QNode::removeMotion(uint32_t index)
{
  if(motion_storage_.left_arm_motion.size()<=index)
  {
    log( Info , "Wrong Item Saved.");
    return false;
  }
  motion_storage_.left_arm_motion.erase(motion_storage_.left_arm_motion.begin()+index);
  motion_storage_.right_arm_motion.erase(motion_storage_.right_arm_motion.begin()+index);
  motion_storage_.path_time.erase(motion_storage_.path_time.begin()+index);
  log( Info , "Motion removed.");
  return true;
}

bool QNode::clearMotion()
{
  motion_storage_.left_arm_motion.clear();
  motion_storage_.right_arm_motion.clear();
  motion_storage_.path_time.clear();
  log( Info , "Motion Cleared.");
  return true;
}

void QNode::updateModeState(const std_msgs::String::ConstPtr &msg)
{
  mode_state_ = msg->data;
}

/*********************
 ** Subscriber callback
 **********************/

void QNode::updateLeftArmStates(const social_robot_arm_msgs::ManipulatorState::ConstPtr &msg)
{
  if(msg->manipulator_moving_state == msg->IS_MOVING)
    left_arm_moving_state_ = true;
  else
    left_arm_moving_state_ = false;

  if(msg->manipulator_actuator_state == msg->ACTUATOR_ENABLED)
    left_arm_actuator_state_ = true;
  else
    left_arm_actuator_state_ = false;
}

void QNode::updateRightArmStates(const social_robot_arm_msgs::ManipulatorState::ConstPtr &msg)
{
  if(msg->manipulator_moving_state == msg->IS_MOVING)
    right_arm_moving_state_ = true;
  else
    right_arm_moving_state_ = false;

  if(msg->manipulator_actuator_state == msg->ACTUATOR_ENABLED)
    right_arm_actuator_state_ = true;
  else
    right_arm_actuator_state_ = false;
}

void QNode::updateLeftArmJointStates(const sensor_msgs::JointState::ConstPtr &msg)
{
  left_arm_joint_angle_.clear();
  for(int i = 0; i < msg->name.size(); i++)
  {
    left_arm_joint_angle_.insert(std::make_pair(msg->name.at(i), msg->position.at(i)));
  }
}

void QNode::updateRightArmJointStates(const sensor_msgs::JointState::ConstPtr &msg)
{
  right_arm_joint_angle_.clear();
  for(int i = 0; i < msg->name.size(); i++)
  {
    right_arm_joint_angle_.insert(std::make_pair(msg->name.at(i), msg->position.at(i)));
  }
}

void QNode::updateLeftHandsKinematicPose(const social_robot_arm_msgs::KinematicsPose::ConstPtr &msg)
{
  left_hand_kinematics_pose_.pose = msg->pose;
}

void QNode::updateRightHandsKinematicPose(const social_robot_arm_msgs::KinematicsPose::ConstPtr &msg)
{
  right_hand_kinematics_pose_.pose = msg->pose;
}

void QNode::sendEStop()
{
  std_msgs::Empty msg;
  e_stop_pub_.publish(msg);
}

void QNode::sendSetMode(bool mode)
{
  social_robot_arm_msgs::SetMode msg;
  log( Info , "Mode Change." );
  if(mode)    //true
    msg.mode = msg.PLAY_MODE;
  else
   msg.mode = msg.EDIT_MODE;
  set_mode_pub_.publish(msg);
}

void QNode::sendMotionSave(uint32_t repeat)
{
  motion_storage_.repeat_count = repeat;
  motion_save_pub_.publish(motion_storage_);
  log( Info , "Send Saved Motion." );
  // Motion Clear
  motion_storage_.left_arm_motion.clear();
  motion_storage_.right_arm_motion.clear();
  motion_storage_.path_time.clear();
  motion_storage_.repeat_count = 1;
  log( Info , "Saved Motion Cleard." );
}

void QNode::sendMotionControl(uint8_t request)
{
  social_robot_arm_msgs::MotionControl msg;

  if(request==0)
  {
    log( Info , "Motion Play." );
    msg.request = msg.PLAY;
  }
  else if(request==1)
  {
    log( Info , "Motion Pause." );
    msg.request = msg.PAUSE;
  }
  else if(request==2)
  {
    log( Info , "Motion Clear." );
    msg.request = msg.CLEAR;
  }
  else if(request==3)
  {
    log( Info , "Motion stop." );
    msg.request = msg.STOP;
  }
  else if(request==4)
  {
    log( Info , "Motion stop cancel." );
    msg.request = msg.CANCEL_STOP;
  }
  else if(request==5)
  {
    log( Info , "Motion break." );
    msg.request = msg.BREAK;
  }
  else if(request==6)
  {
    log( Info , "Motion break cancel." );
    msg.request = msg.CANCEL_BREAK;
  }
  else if(request==7)
  {
    log( Info , "Return First Motion." );
    msg.request = msg.RETURN_FIRST;
  }
  else if(request==8)
  {
    log( Info , "Return Motion." );
    msg.request = msg.RETURN_MOTION;
  }
  else
  {
    log( Info , "E-STOP." );
    msg.request = msg.E_STOP;
  }
  motion_control_pub_.publish(msg);
}

/*********************
 ** ServiceClient call
 **********************/
bool QNode::sendLeftArmGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  social_robot_arm_msgs::SetJointPosition srv;

  srv.request.planning_group = "left_arm";
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  log( Info , "Send Left Arm Goal Joint Angle Msg" );
  if(goal_joint_space_path_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendRightArmGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  social_robot_arm_msgs::SetJointPosition srv;

  srv.request.planning_group = "right_arm";
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  log( Info , "Send Right Arm Goal Joint Angle Msg" );
  if(goal_joint_space_path_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendLeftArmKinematicPose(std::vector<double> kinematics_pose, double path_time)
{
  social_robot_arm_msgs::SetKinematicsPose srv;

  srv.request.planning_group = "left_arm";
  srv.request.end_effector_name = left_hand_name_for_pose_;
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  Eigen::Quaterniond quat = convertRPYToQuaternion(kinematics_pose.at(3), kinematics_pose.at(4), kinematics_pose.at(5));
  srv.request.kinematics_pose.pose.orientation.w  = quat.w();
  srv.request.kinematics_pose.pose.orientation.x  = quat.x();
  srv.request.kinematics_pose.pose.orientation.y  = quat.y();
  srv.request.kinematics_pose.pose.orientation.z  = quat.z();

  srv.request.path_time = path_time;
  log( Info , "Send Left Arm Goal Kinematics Pose Msg" );
  if(goal_task_space_path_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendRightArmKinematicPose(std::vector<double> kinematics_pose, double path_time)
{
  social_robot_arm_msgs::SetKinematicsPose srv;

  srv.request.planning_group = "right_arm";
  srv.request.end_effector_name = right_hand_name_for_pose_;
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  Eigen::Quaterniond quat = convertRPYToQuaternion(kinematics_pose.at(3), kinematics_pose.at(4), kinematics_pose.at(5));
  srv.request.kinematics_pose.pose.orientation.w  = quat.w();
  srv.request.kinematics_pose.pose.orientation.x  = quat.x();
  srv.request.kinematics_pose.pose.orientation.y  = quat.y();
  srv.request.kinematics_pose.pose.orientation.z  = quat.z();

  srv.request.path_time = path_time;
  log( Info , "Send Right Arm Goal Kinematics Pose Msg" );
  if(goal_task_space_path_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendLeftArmMoveitGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  social_robot_arm_msgs::SetJointPosition srv;

  srv.request.planning_group = "left_arm";
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;
  srv.request.joint_position.max_velocity_scaling_factor = 1.0;
  srv.request.joint_position.max_accelerations_scaling_factor = 1.0;
  log( Info , "Send Left Arm Goal Joint Angle Msg" );
  if(set_moveit_joint_position_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendRightArmMoveitGoalJointAngle(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  social_robot_arm_msgs::SetJointPosition srv;

  srv.request.planning_group = "right_arm";
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;
  srv.request.joint_position.max_velocity_scaling_factor = 1.0;
  srv.request.joint_position.max_accelerations_scaling_factor = 1.0;
  log( Info , "Send Right Arm Goal Joint Angle Msg" );
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendLeftArmMoveitKinematicPose(std::vector<double> kinematics_pose, double path_time)
{
  social_robot_arm_msgs::SetKinematicsPose srv;

  srv.request.planning_group = "left_arm";
  srv.request.end_effector_name = left_hand_name_for_pose_;
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  Eigen::Quaterniond quat = convertRPYToQuaternion(kinematics_pose.at(3), kinematics_pose.at(4), kinematics_pose.at(5));
  srv.request.kinematics_pose.pose.orientation.w  = quat.w();
  srv.request.kinematics_pose.pose.orientation.x  = quat.x();
  srv.request.kinematics_pose.pose.orientation.y  = quat.y();
  srv.request.kinematics_pose.pose.orientation.z  = quat.z();

  srv.request.path_time = path_time;
  srv.request.kinematics_pose.max_velocity_scaling_factor = 1.0;
  srv.request.kinematics_pose.max_accelerations_scaling_factor = 1.0;
  srv.request.kinematics_pose.tolerance = 0.001;
  log( Info , "Send Left Arm Goal Kinematics Pose Msg" );
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendRightArmMoveitKinematicPose(std::vector<double> kinematics_pose, double path_time)
{
  social_robot_arm_msgs::SetKinematicsPose srv;

  srv.request.planning_group = "right_arm";
  srv.request.end_effector_name = right_hand_name_for_pose_;
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  Eigen::Quaterniond quat = convertRPYToQuaternion(kinematics_pose.at(3), kinematics_pose.at(4), kinematics_pose.at(5));
  srv.request.kinematics_pose.pose.orientation.w  = quat.w();
  srv.request.kinematics_pose.pose.orientation.x  = quat.x();
  srv.request.kinematics_pose.pose.orientation.y  = quat.y();
  srv.request.kinematics_pose.pose.orientation.z  = quat.z();

  srv.request.path_time = path_time;

  srv.request.kinematics_pose.max_velocity_scaling_factor = 1.0;
  srv.request.kinematics_pose.max_accelerations_scaling_factor = 1.0;
  srv.request.kinematics_pose.tolerance = 0.001;
  log( Info , "Send Right Arm Goal Kinematics Pose Msg" );
  if(set_moveit_kinematics_pose_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendLeftHandPosition(std::vector<std::string> joint_name, std::vector<double> joint_angle)
{
  social_robot_arm_msgs::SetJointPosition srv;
  srv.request.planning_group = "left_arm";
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  log( Info , "Send Left Hand Goal Position Msg" );
  if(goal_tool_control_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendRightHandPosition(std::vector<std::string> joint_name, std::vector<double> joint_angle)
{
  social_robot_arm_msgs::SetJointPosition srv;
  srv.request.planning_group = "right_arm";
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  log( Info , "Send Right Hand Goal Position Msg" );
  if(goal_tool_control_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendLeftArmActuatorState(bool actuator_state)
{
  social_robot_arm_msgs::SetActuatorState srv;
  srv.request.planning_group = "left_arm";
  srv.request.set_actuator_state = actuator_state;
  log( Info , "Send Left Arm Actuaotor States Msg" );
  if(set_actuator_state_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::sendRightArmActuatorState(bool actuator_state)
{
  social_robot_arm_msgs::SetActuatorState srv;
  srv.request.planning_group = "right_arm";
  srv.request.set_actuator_state = actuator_state;
  log( Info , "Send Right Arm Actuaotor States Msg" );
  if(set_actuator_state_client_.call(srv))
  {
    if(srv.response.is_planned)
      log( Info , "Sucsses" );
    else
      log( Info , "Fail" );
    return srv.response.is_planned;
  }
  return false;
}

}  // namespace social_robot_arm_control_gui
