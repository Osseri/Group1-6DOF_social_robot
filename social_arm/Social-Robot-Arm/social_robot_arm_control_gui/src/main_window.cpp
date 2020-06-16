/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "social_robot_arm_control_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace social_robot_arm_control_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode_(argc,argv)
{
  ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  //Logging Set
  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  //Timer
  ui_timer_ = new QTimer;
  QObject::connect(ui_timer_, SIGNAL(timeout()), this, SLOT(timerLoop()));

  //Qnode Init
  std::string left_hand_name_for_pose;
  left_hand_name_for_pose = "LFinger_1";
  std::string right_hand_name_for_pose;
  right_hand_name_for_pose = "RFinger_1";

  qnode_.init(left_hand_name_for_pose, right_hand_name_for_pose);

  //UI box name Set
  left_joint_and_hand_box_adr_.insert(std::make_pair("LShoulder_Pitch" , ui_.left_shoulder_pitch));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LShoulder_Roll" , ui_.left_shoulder_roll));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LElbow_Pitch" , ui_.left_elbow_pitch));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LElbow_Yaw" , ui_.left_elbow_yaw));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LWrist_Pitch" , ui_.left_wrist_pitch));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LWrist_Roll" , ui_.left_wrist_roll));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LFinger_1" , ui_.left_finger1_position));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LFinger_2" , ui_.left_finger2_position));
  left_joint_and_hand_box_adr_.insert(std::make_pair("LFinger_3" , ui_.left_finger3_position));

  right_joint_and_hand_box_adr_.insert(std::make_pair("RShoulder_Pitch" , ui_.right_shoulder_pitch));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RShoulder_Roll" , ui_.right_shoulder_roll));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RElbow_Pitch" , ui_.right_elbow_pitch));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RElbow_Yaw" , ui_.right_elbow_yaw));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RWrist_Pitch" , ui_.right_wrist_pitch));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RWrist_Roll" , ui_.right_wrist_roll));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RFinger_1" , ui_.right_finger1_position));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RFinger_2" , ui_.right_finger2_position));
  right_joint_and_hand_box_adr_.insert(std::make_pair("RFinger_3" , ui_.right_finger3_position));

  left_goal_joint_box_adr_.insert(std::make_pair("LShoulder_Pitch" , ui_.left_goal_shoulder_pitch));
  left_goal_joint_box_adr_.insert(std::make_pair("LShoulder_Roll" , ui_.left_goal_shoulder_roll));
  left_goal_joint_box_adr_.insert(std::make_pair("LElbow_Pitch" , ui_.left_goal_elbow_pitch));
  left_goal_joint_box_adr_.insert(std::make_pair("LElbow_Yaw" , ui_.left_goal_elbow_yaw));
  left_goal_joint_box_adr_.insert(std::make_pair("LWrist_Pitch" , ui_.left_goal_wrist_pitch));
  left_goal_joint_box_adr_.insert(std::make_pair("LWrist_Roll" , ui_.left_goal_wrist_roll));

  left_goal_hand_box_adr_.insert(std::make_pair("LFinger_1" , ui_.left_goal_finger1_position));
  left_goal_hand_box_adr_.insert(std::make_pair("LFinger_2" , ui_.left_goal_finger2_position));
  left_goal_hand_box_adr_.insert(std::make_pair("LFinger_3" , ui_.left_goal_finger3_position));

  right_goal_joint_box_adr_.insert(std::make_pair("RShoulder_Pitch" , ui_.right_goal_shoulder_pitch));
  right_goal_joint_box_adr_.insert(std::make_pair("RShoulder_Roll" , ui_.right_goal_shoulder_roll));
  right_goal_joint_box_adr_.insert(std::make_pair("RElbow_Pitch" , ui_.right_goal_elbow_pitch));
  right_goal_joint_box_adr_.insert(std::make_pair("RElbow_Yaw" , ui_.right_goal_elbow_yaw));
  right_goal_joint_box_adr_.insert(std::make_pair("RWrist_Pitch" , ui_.right_goal_wrist_pitch));
  right_goal_joint_box_adr_.insert(std::make_pair("RWrist_Roll" , ui_.right_goal_wrist_roll));

  right_goal_hand_box_adr_.insert(std::make_pair("RFinger_1" , ui_.right_goal_finger1_position));
  right_goal_hand_box_adr_.insert(std::make_pair("RFinger_2" , ui_.right_goal_finger2_position));
  right_goal_hand_box_adr_.insert(std::make_pair("RFinger_3" , ui_.right_goal_finger3_position));

  motion_box_adr_.insert(std::make_pair("LShoulder_Pitch" , ui_.motion_shoulder_pitch_left));
  motion_box_adr_.insert(std::make_pair("LShoulder_Roll" , ui_.motion_shoulder_roll_left));
  motion_box_adr_.insert(std::make_pair("LElbow_Pitch" , ui_.motion_elbow_pitch_left));
  motion_box_adr_.insert(std::make_pair("LElbow_Yaw" , ui_.motion_elbow_yaw_left));
  motion_box_adr_.insert(std::make_pair("LWrist_Pitch" , ui_.motion_wrist_pitch_left));
  motion_box_adr_.insert(std::make_pair("LWrist_Roll" , ui_.motion_wrist_roll_left));
  motion_box_adr_.insert(std::make_pair("LFinger_1" , ui_.motion_finger1_left));
  motion_box_adr_.insert(std::make_pair("LFinger_2" , ui_.motion_finger2_left));
  motion_box_adr_.insert(std::make_pair("LFinger_3" , ui_.motion_finger3_left));
  motion_box_adr_.insert(std::make_pair("RShoulder_Pitch" , ui_.motion_shoulder_pitch_right));
  motion_box_adr_.insert(std::make_pair("RShoulder_Roll" , ui_.motion_shoulder_roll_right));
  motion_box_adr_.insert(std::make_pair("RElbow_Pitch" , ui_.motion_elbow_pitch_right));
  motion_box_adr_.insert(std::make_pair("RElbow_Yaw" , ui_.motion_elbow_yaw_right));
  motion_box_adr_.insert(std::make_pair("RWrist_Pitch" , ui_.motion_wrist_pitch_right));
  motion_box_adr_.insert(std::make_pair("RWrist_Roll" , ui_.motion_wrist_roll_right));
  motion_box_adr_.insert(std::make_pair("RFinger_1" , ui_.motion_finger1_right));
  motion_box_adr_.insert(std::make_pair("RFinger_2" , ui_.motion_finger2_right));
  motion_box_adr_.insert(std::make_pair("RFinger_3" , ui_.motion_finger3_right));
  motion_box_adr_.insert(std::make_pair("time" , ui_.motion_saved_time));

  ui_.motion_file_name->setText("motion.yaml");
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

void MainWindow::roadYamljointPositionData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  double mov_time = doc["mov_time"].as<double>();

  // left arm joint angle
  YAML::Node left_arm_joint_angle = doc["left_arm_joint_angle"];
  std::vector<std::string> left_joint_names;
  std::vector<double> left_joint_angles;
  for(YAML::iterator it = left_arm_joint_angle.begin(); it!=left_arm_joint_angle.end(); it++)
  {
    left_joint_names.push_back(it->first.as<std::string>());
    left_joint_angles.push_back(it->second.as<double>()*DEG2RAD);
  }
  qnode_.sendLeftArmGoalJointAngle(left_joint_names, left_joint_angles, mov_time);

  // left arm end_effector angle
  YAML::Node left_arm_end_effector_angle = doc["left_arm_end_effector_angle"];
  std::vector<std::string> left_end_effector_names;
  std::vector<double> left_end_effector_angles;
  for(YAML::iterator it = left_arm_end_effector_angle.begin(); it!=left_arm_end_effector_angle.end(); it++)
  {
    left_end_effector_names.push_back(it->first.as<std::string>());
    left_end_effector_angles.push_back(it->second.as<double>()*DEG2RAD);
  }
  qnode_.sendLeftHandPosition(left_end_effector_names, left_end_effector_angles);

  // right arm joint angle
  YAML::Node right_arm_joint_angle = doc["right_arm_joint_angle"];
  std::vector<std::string> right_joint_names;
  std::vector<double> right_joint_angles;
  for(YAML::iterator it = right_arm_joint_angle.begin(); it!=right_arm_joint_angle.end(); it++)
  {
    right_joint_names.push_back(it->first.as<std::string>());
    right_joint_angles.push_back(it->second.as<double>()*DEG2RAD);
  }
  qnode_.sendRightArmGoalJointAngle(right_joint_names, right_joint_angles, mov_time);

  // right arm end_effector angle
  YAML::Node right_arm_end_effector_angle = doc["right_arm_end_effector_angle"];
  std::vector<std::string> right_end_effector_names;
  std::vector<double> right_end_effector_angles;
  for(YAML::iterator it = right_arm_end_effector_angle.begin(); it!=right_arm_end_effector_angle.end(); it++)
  {
    right_end_effector_names.push_back(it->first.as<std::string>());
    right_end_effector_angles.push_back(it->second.as<double>()*DEG2RAD);
  }
  qnode_.sendRightHandPosition(right_end_effector_names, right_end_effector_angles);
}

void MainWindow::roadYamlMotion(const std::string &path, social_robot_arm_msgs::MotionSave *motion_storage)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load Motion yaml file.");
    return;
  }

  int motion_size = doc["motion_size"].as<int>();
  int repeat_cnt = doc["repeat_cnt"].as<int>();
  ui_.motion_repeat->setValue(repeat_cnt);

  for(int i = 0; i < motion_size; i++)
  {
    std::string cnt_str = std::to_string(i);
    YAML::Node pose = doc[cnt_str];
//    std::cout << "<<Motion " << cnt_str << ">>" << std::endl;
    for(YAML::iterator pose_it = pose.begin(); pose_it!=pose.end(); pose_it++)
    {
      // Read
      if(pose_it->first.as<std::string>() == "left_arm")
      {
//        printf("left_arm:\n");
        YAML::Node left = pose_it->second;
        sensor_msgs::JointState left_arm_joint_angle;
        for(YAML::iterator left_it = left.begin(); left_it!=left.end(); left_it++)
        {
//          std::cout << "\t" << left_it->first.as<std::string>() << " : " << left_it->second.as<double>() << std::endl;
          left_arm_joint_angle.name.push_back(left_it->first.as<std::string>());
          left_arm_joint_angle.position.push_back(left_it->second.as<double>()*DEG2RAD);
          left_arm_joint_angle.velocity.push_back(0.0);
          left_arm_joint_angle.effort.push_back(0.0);
        }
        motion_storage->left_arm_motion.push_back(left_arm_joint_angle);
      }
      else if(pose_it->first.as<std::string>() == "right_arm")
      {
//        printf("right_arm:\n");
        YAML::Node right = pose_it->second;
        sensor_msgs::JointState right_arm_joint_angle;
        for(YAML::iterator right_it = right.begin(); right_it!=right.end(); right_it++)
        {
//          std::cout << "\t" << right_it->first.as<std::string>() << " : " << right_it->second.as<double>() << std::endl;
          right_arm_joint_angle.name.push_back(right_it->first.as<std::string>());
          right_arm_joint_angle.position.push_back(right_it->second.as<double>()*DEG2RAD);
          right_arm_joint_angle.velocity.push_back(0.0);
          right_arm_joint_angle.effort.push_back(0.0);
        }
        motion_storage->right_arm_motion.push_back(right_arm_joint_angle);
      }
      else if(pose_it->first.as<std::string>() == "mov_time")
      {
//        std::cout << "Time : " << pose_it->second.as<double>() << std::endl;
        motion_storage->path_time.push_back(pose_it->second.as<double>());
      }
    }
  }

}


void MainWindow::updateLoggingView() {
  ui_.view_logging->scrollToBottom();
}

void MainWindow::timerLoop()
{
  usingArm();
  updateLeftArmStates();
  updateRightArmtStates();
  updateLeftArmJointStates();
  updateRightArmJointStates();
  updateLeftHandKinematicPose();
  updateRightHandKinematicPose();
  updatePresentMode();
  updateSavedMotion();
}

void MainWindow::usingArm()
{
  if(qnode_.usingLeftArm())
    ui_.left_arm->setEnabled(true);
  else
    ui_.left_arm->setEnabled(false);

  if(qnode_.usingRightArm())
    ui_.right_arm->setEnabled(true);
  else
    ui_.right_arm->setEnabled(false);
}


void MainWindow::updateLeftArmStates()
{
  if(qnode_.getLeftArmActuatorState())
    ui_.left_arm_actuator_state->setText("Enabled");
  else
    ui_.left_arm_actuator_state->setText("Disabled");
  if(qnode_.getLeftArmMovingState())
    ui_.left_arm_moving_state->setText("Moving");
  else
    ui_.left_arm_moving_state->setText("Stopped");
}

void MainWindow::updateRightArmtStates()
{
  if(qnode_.getRightArmActuatorState())
    ui_.right_arm_actuator_state->setText("Enabled");
  else
    ui_.right_arm_actuator_state->setText("Disabled");
  if(qnode_.getRightArmMovingState())
    ui_.right_arm_moving_state->setText("Moving");
  else
    ui_.right_arm_moving_state->setText("Stopped");
}

void MainWindow::updateLeftArmJointStates()
{
  auto joint_angle = qnode_.getLeftArmJointAngle();
  for(auto it = joint_angle.begin(); it != joint_angle.end(); it++)
  {
    if(left_joint_and_hand_box_adr_.find(it->first) != left_joint_and_hand_box_adr_.end())
    {
      left_joint_and_hand_box_adr_.at(it->first)->setText(QString::number(joint_angle.at(it->first)*RAD2DEG, 'f', 1));
    }
  }
}

void MainWindow::updateRightArmJointStates()
{
  auto joint_angle = qnode_.getRightArmJointAngle();
  for(auto it = joint_angle.begin(); it != joint_angle.end(); it++)
  {
    if(right_joint_and_hand_box_adr_.find(it->first) != right_joint_and_hand_box_adr_.end())
    {
      right_joint_and_hand_box_adr_.at(it->first)->setText(QString::number(joint_angle.at(it->first)*RAD2DEG, 'f', 1));
    }
  }
}

void MainWindow::updateLeftHandKinematicPose()
{
  auto pose = qnode_.getLeftHandKinematicsPose();
  ui_.left_x->setText(QString::number(pose.at(0), 'f', 3));
  ui_.left_y->setText(QString::number(pose.at(1), 'f', 3));
  ui_.left_z->setText(QString::number(pose.at(2), 'f', 3));
  ui_.left_roll->setText(QString::number(pose.at(3) * RAD2DEG, 'f', 1));
  ui_.left_pitch->setText(QString::number(pose.at(4) * RAD2DEG, 'f', 1));
  ui_.left_yaw->setText(QString::number(pose.at(5) * RAD2DEG, 'f', 1));
}

void MainWindow::updateRightHandKinematicPose()
{
  auto pose = qnode_.getRightHandKinematicsPose();
  ui_.right_x->setText(QString::number(pose.at(0), 'f', 3));
  ui_.right_y->setText(QString::number(pose.at(1), 'f', 3));
  ui_.right_z->setText(QString::number(pose.at(2), 'f', 3));
  ui_.right_roll->setText(QString::number(pose.at(3) * RAD2DEG, 'f', 1));
  ui_.right_pitch->setText(QString::number(pose.at(4) * RAD2DEG, 'f', 1));
  ui_.right_yaw->setText(QString::number(pose.at(5) * RAD2DEG, 'f', 1));
}

void MainWindow::on_start_button_clicked(void)
{
  ui_timer_->start(100);

}

void MainWindow::on_e_stop_button_clicked()
{
  qnode_.sendEStop();
}

void MainWindow::on_init_button_clicked(bool check)
{
  std::string ini_pose_path = ros::package::getPath("social_robot_arm_control_gui") + "/config/init_pose.yaml";
  roadYamljointPositionData(ini_pose_path);
}

void MainWindow::on_zero_button_clicked(bool check)
{
  std::string zero_pose_path = ros::package::getPath("social_robot_arm_control_gui") + "/config/zero_pose.yaml";
  roadYamljointPositionData(zero_pose_path);
}

void MainWindow::on_left_arm_enable_clicked(void)
{
  qnode_.sendLeftArmActuatorState(true);
}

void MainWindow::on_left_arm_disable_clicked(void)
{
  qnode_.sendLeftArmActuatorState(false);
}

void MainWindow::on_right_arm_enable_clicked(void)
{
  qnode_.sendRightArmActuatorState(true);
}

void MainWindow::on_right_arm_disable_clicked(void)
{
  qnode_.sendRightArmActuatorState(false);
}

void MainWindow::on_left_arm_read_clicked(void)
{
  if(ui_.left_arm_tab->currentIndex()==0)         //Joint Control
  {
    auto joint_angle = qnode_.getLeftArmJointAngle();
    for(auto it = joint_angle.begin(); it != joint_angle.end(); it++)
    {
      auto box_it = left_goal_joint_box_adr_.find(it->first);
      if(box_it != left_goal_joint_box_adr_.end())
        left_goal_joint_box_adr_.at(box_it->first)->setValue(joint_angle.at(it->first)*RAD2DEG);
    }
  }
  else if(ui_.left_arm_tab->currentIndex()==1)    //Task Control
  {
    auto pose = qnode_.getLeftHandKinematicsPose();
    ui_.left_goal_x->setValue(pose.at(0));
    ui_.left_goal_y->setValue(pose.at(1));
    ui_.left_goal_z->setValue(pose.at(2));
    ui_.left_goal_roll->setValue(pose.at(3) * RAD2DEG);
    ui_.left_goal_pitch->setValue(pose.at(4) * RAD2DEG);
    ui_.left_goal_yaw->setValue(pose.at(5) * RAD2DEG);
  }
  else if(ui_.left_arm_tab->currentIndex()==2)    //Hand Control
  {
    auto joint_angle = qnode_.getLeftArmJointAngle();
    for(auto it = joint_angle.begin(); it != joint_angle.end(); it++)
    {
      auto box_it = left_goal_hand_box_adr_.find(it->first);
      if(box_it != left_goal_hand_box_adr_.end())
        left_goal_hand_box_adr_.at(box_it->first)->setValue(joint_angle.at(it->first)*RAD2DEG);
    }
  }
}

void MainWindow::on_left_arm_send_clicked(void)
{
  if(ui_.left_arm_tab->currentIndex()==0)         //Joint Control
  {
    std::vector<std::string> joint_names;
    std::vector<double> joint_angles;
    double path_time;

    for(auto it = left_goal_joint_box_adr_.begin(); it != left_goal_joint_box_adr_.end(); it++)
    {
      joint_names.push_back(it->first);
      joint_angles.push_back(it->second->value()*DEG2RAD);
    }
    path_time = ui_.left_arm_time->value();

    if(!ui_.left_moveit_check->isChecked())
      qnode_.sendLeftArmGoalJointAngle(joint_names, joint_angles, path_time);
    else
      qnode_.sendLeftArmMoveitGoalJointAngle(joint_names, joint_angles, path_time);
  }
  else if(ui_.left_arm_tab->currentIndex()==1)    //Task Control
  {
    std::vector<double> kinematic_pose;
    double path_time;
    kinematic_pose.push_back(ui_.left_goal_x->value());
    kinematic_pose.push_back(ui_.left_goal_y->value());
    kinematic_pose.push_back(ui_.left_goal_z->value());
    kinematic_pose.push_back(ui_.left_goal_roll->value()*DEG2RAD);
    kinematic_pose.push_back(ui_.left_goal_pitch->value()*DEG2RAD);
    kinematic_pose.push_back(ui_.left_goal_yaw->value()*DEG2RAD);
    path_time = ui_.left_arm_time->value();

    if(!ui_.left_moveit_check->isChecked())
      qnode_.sendLeftArmKinematicPose(kinematic_pose, path_time);
    else
      qnode_.sendLeftArmMoveitKinematicPose(kinematic_pose, path_time);

  }
  else if(ui_.left_arm_tab->currentIndex()==2)    //Hand Control
  {
    std::vector<std::string> joint_names;
    std::vector<double> value;

    for(auto it = left_goal_hand_box_adr_.begin(); it != left_goal_hand_box_adr_.end(); it++)
    {
      joint_names.push_back(it->first);
      value.push_back(it->second->value()*DEG2RAD);
    }
    qnode_.sendLeftHandPosition(joint_names, value);
  }
}

void MainWindow::on_right_arm_read_clicked(void)
{
  if(ui_.right_arm_tab->currentIndex()==0)         //Joint Control
  {
    auto joint_angle = qnode_.getRightArmJointAngle();
    for(auto it = joint_angle.begin(); it != joint_angle.end(); it++)
    {
      auto box_it = right_goal_joint_box_adr_.find(it->first);
      if(box_it != right_goal_joint_box_adr_.end())
        right_goal_joint_box_adr_.at(box_it->first)->setValue(joint_angle.at(it->first)*RAD2DEG);
    }
  }
  else if(ui_.right_arm_tab->currentIndex()==1)    //Task Control
  {
    auto pose = qnode_.getRightHandKinematicsPose();
    ui_.right_goal_x->setValue(pose.at(0));
    ui_.right_goal_y->setValue(pose.at(1));
    ui_.right_goal_z->setValue(pose.at(2));
    ui_.right_goal_roll->setValue(pose.at(3) * RAD2DEG);
    ui_.right_goal_pitch->setValue(pose.at(4) * RAD2DEG);
    ui_.right_goal_yaw->setValue(pose.at(5) * RAD2DEG);
  }
  else if(ui_.right_arm_tab->currentIndex()==2)    //Hand Control
  {
    auto joint_angle = qnode_.getRightArmJointAngle();
    for(auto it = joint_angle.begin(); it != joint_angle.end(); it++)
    {
      auto box_it = right_goal_hand_box_adr_.find(it->first);
      if(box_it != right_goal_hand_box_adr_.end())
        right_goal_hand_box_adr_.at(box_it->first)->setValue(joint_angle.at(it->first)*RAD2DEG);
    }
  }
}

void MainWindow::on_right_arm_send_clicked(void)
{
  if(ui_.right_arm_tab->currentIndex()==0)         //Joint Control
  {
    std::vector<std::string> joint_names;
    std::vector<double> joint_angles;
    double path_time;

    for(auto it = right_goal_joint_box_adr_.begin(); it != right_goal_joint_box_adr_.end(); it++)
    {
      joint_names.push_back(it->first);
      joint_angles.push_back(it->second->value()*DEG2RAD);
    }
    path_time = ui_.right_arm_time->value();

    if(!ui_.right_moveit_check->isChecked())
      qnode_.sendRightArmGoalJointAngle(joint_names, joint_angles, path_time);
    else
      qnode_.sendRightArmMoveitGoalJointAngle(joint_names, joint_angles, path_time);
  }
  else if(ui_.right_arm_tab->currentIndex()==1)    //Task Control
  {
    std::vector<double> kinematic_pose;
    double path_time;
    kinematic_pose.push_back(ui_.right_goal_x->value());
    kinematic_pose.push_back(ui_.right_goal_y->value());
    kinematic_pose.push_back(ui_.right_goal_z->value());
    kinematic_pose.push_back(ui_.right_goal_roll->value()*DEG2RAD);
    kinematic_pose.push_back(ui_.right_goal_pitch->value()*DEG2RAD);
    kinematic_pose.push_back(ui_.right_goal_yaw->value()*DEG2RAD);
    path_time = ui_.right_arm_time->value();

    if(!ui_.right_moveit_check->isChecked())
      qnode_.sendRightArmKinematicPose(kinematic_pose, path_time);
    else
      qnode_.sendRightArmMoveitKinematicPose(kinematic_pose, path_time);
  }
  else if(ui_.right_arm_tab->currentIndex()==2)    //Hand Control
  {
    std::vector<std::string> joint_names;
    std::vector<double> value;

    for(auto it = right_goal_hand_box_adr_.begin(); it != right_goal_hand_box_adr_.end(); it++)
    {
      joint_names.push_back(it->first);
      value.push_back(it->second->value()*DEG2RAD);
    }
    qnode_.sendRightHandPosition(joint_names, value);
  }
}

void MainWindow::updatePresentMode()
{
  ui_.present_mode->setText(QString::fromStdString(qnode_.getModeState()));
}

void MainWindow::on_play_mode_button_clicked()
{
  qnode_.sendSetMode(true);
}

void MainWindow::on_edit_mode_button_clicked()
{
  qnode_.sendSetMode(false);
}

void MainWindow::updateSavedMotionList()
{
  ui_.motion_list->clear();
  uint32_t size = qnode_.getMotionStorageSize();
  for(uint32_t i=0; i<size; i++)
  {
    ui_.motion_list->addItem(QString::fromStdString("pose_"+std::to_string(i+1)));
  }
//  ui_.motion_list->setCurrentRow(size-1);
}

void MainWindow::updateSavedMotion()
{
  int selected_num = ui_.motion_list->currentRow();
  if(selected_num < 0)
    selected_num=0;
  auto motion_joint_state = qnode_.getMotionStorage(selected_num);
  for(auto it=motion_joint_state.begin(); it!=motion_joint_state.end();it++)
  {
    if(it->first != "time")
      motion_box_adr_.at(it->first)->setText(QString::number(it->second*RAD2DEG, 'f', 1));
    else
      motion_box_adr_.at(it->first)->setText(QString::number(it->second, 'f', 1));
  }
}

void MainWindow::on_motion_save_clicked()
{
  int selected_num = ui_.motion_list->currentRow();
  if(selected_num < 0)
    selected_num=0;
  else
    selected_num = selected_num+1;

  double time = ui_.motion_time->value();
  if(!qnode_.saveMotion(selected_num ,time))
    return;

  updateSavedMotionList();
  ui_.motion_list->setCurrentRow(selected_num);
  return;
}

void MainWindow::on_motion_remove_clicked()
{
  uint32_t selected_num = ui_.motion_list->currentRow();
  if(selected_num < 0)
    selected_num=0;

  if(!qnode_.removeMotion(selected_num))
    return;

  updateSavedMotionList();
  ui_.motion_list->setCurrentRow(selected_num);
  return;
}

void MainWindow::on_motion_send_clicked()
{
  uint32_t repeat_cnt = ui_.motion_repeat->value();
  qnode_.sendMotionSave(repeat_cnt);
  ui_.motion_list->clear();
}

void MainWindow::on_motion_clear_clicked()
{
  qnode_.sendMotionControl(2);
}

void MainWindow::on_motion_play_clicked()
{
  qnode_.sendMotionControl(0);
}

void MainWindow::on_motion_pause_clicked()
{
  qnode_.sendMotionControl(1);
}

void MainWindow::on_motion_stop_clicked()
{
  qnode_.sendMotionControl(3);
}

void MainWindow::on_motion_break_clicked()
{
  qnode_.sendMotionControl(5);
}

void MainWindow::on_motion_return_first_clicked()
{
  qnode_.sendMotionControl(7);
}

void MainWindow::on_motion_stop_cancel_clicked()
{
  qnode_.sendMotionControl(4);
}

void MainWindow::on_motion_break_cancel_clicked()
{
  qnode_.sendMotionControl(6);
}

void MainWindow::on_motion_return_motion_clicked()
{
  qnode_.sendMotionControl(8);
}

void MainWindow::on_motion_e_stop_clicked()
{
  qnode_.sendMotionControl(9);
}

void MainWindow::on_motion_read_clicked()
{
  social_robot_arm_msgs::MotionSave motion_storage;

  // Path read
  std::string motion_path = ros::package::getPath("social_robot_arm_control_gui") + "/motion/" + ui_.motion_file_name->text().toStdString();
  // File road
  roadYamlMotion(motion_path, &motion_storage);
  // Save
  qnode_.saveMotion(motion_storage);
  updateSavedMotionList();
  return;
}

void MainWindow::on_saved_motion_clear_clicked()
{
  qnode_.clearMotion();
  updateSavedMotionList();
  return;
}

}  // namespace social_robot_arm_control_gui

