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

#ifndef SOCIAL_ROBOT_ARM_CONTROL_GUI_MAIN_WINDOW_H
#define SOCIAL_ROBOT_ARM_CONTROL_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
// QT
#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QTimer>

// yaml
#include <yaml-cpp/yaml.h>

// Motion
#include "social_robot_arm_msgs/MotionSave.h"
#include <eigen3/Eigen/Eigen>

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace social_robot_arm_control_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/

class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();
  void closeEvent(QCloseEvent *event); // Overloaded function

  void roadYamljointPositionData(const std::string &path);
  void roadYamlMotion(const std::string &path, social_robot_arm_msgs::MotionSave* motion_storage);

public Q_SLOTS:
  void updateLoggingView(); // no idea why this can't connect automatically

  void timerLoop();
  /******************************************
   ** Set display values
   *******************************************/
  void usingArm();
  void updateLeftArmStates();
  void updateRightArmtStates();
  void updateLeftArmJointStates();
  void updateRightArmJointStates();
  void updateLeftHandKinematicPose();
  void updateRightHandKinematicPose();


  /******************************************
   ** Button Clicked
   *******************************************/
  void on_start_button_clicked(void);
//  void on_quit_button_clicked(void);
  void on_e_stop_button_clicked(void);

  void on_init_button_clicked(bool check);
  void on_zero_button_clicked(bool check);

  void on_left_arm_enable_clicked(void);
  void on_left_arm_disable_clicked(void);
  void on_right_arm_enable_clicked(void);
  void on_right_arm_disable_clicked(void);

  void on_left_arm_read_clicked(void);
  void on_left_arm_send_clicked(void);
  void on_right_arm_read_clicked(void);
  void on_right_arm_send_clicked(void);

  /******************************************
   ** Motion
  *******************************************/
  void updatePresentMode();
  void on_play_mode_button_clicked(void);
  void on_edit_mode_button_clicked(void);

  void updateSavedMotionList();
  void updateSavedMotion();
  void on_motion_save_clicked(void);  //
  void on_motion_remove_clicked(void);
  void on_motion_send_clicked(void);

  void on_motion_play_clicked(void);  //
  void on_motion_pause_clicked(void);
  void on_motion_clear_clicked(void); //
  void on_motion_stop_clicked(void);  //
  void on_motion_break_clicked(void);
  void on_motion_return_first_clicked(void);
  void on_motion_stop_cancel_clicked(void);
  void on_motion_break_cancel_clicked(void);
  void on_motion_return_motion_clicked(void);
  void on_motion_e_stop_clicked(void);

  void on_motion_read_clicked(void);
  void on_saved_motion_clear_clicked(void);

private:
  Ui::MainWindowDesign ui_;
  QNode qnode_;
  QTimer* ui_timer_;

  std::map<std::string, QLineEdit*> left_joint_and_hand_box_adr_;
  std::map<std::string, QLineEdit*> right_joint_and_hand_box_adr_;
  std::map<std::string, QLineEdit*> motion_box_adr_;

  std::map<std::string, QDoubleSpinBox*> left_goal_joint_box_adr_;
  std::map<std::string, QDoubleSpinBox*> right_goal_joint_box_adr_;

  std::map<std::string, QDoubleSpinBox*> left_goal_hand_box_adr_;
  std::map<std::string, QDoubleSpinBox*> right_goal_hand_box_adr_;

//  std::vector<std::string> joint_name;
//  QList<QAbstractSpinBox *> joint_box;
//  QList<QAbstractSpinBox *> cr_joint_box;
};

}  // namespace social_robot_arm_control_gui

#endif // SOCIAL_ROBOT_ARM_CONTROL_GUI_MAIN_WINDOW_H
