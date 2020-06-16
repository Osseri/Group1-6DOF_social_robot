#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from social_robot_arm_msgs.msg import SetJointTorque
import time

class SocialArmTorqueRelease():

    def __init__(self):
        rospy.init_node('social_arm_torque_release')
        __cmd_pos_publisher = None
        self.__torque_publisher = rospy.Publisher('/arm/set_joint_torque', SetJointTorque, queue_size=10)

        time.sleep(1)
        arm_joint_list = [
                        'LElbow_Pitch', 
                        'LElbow_Yaw', 
                        'LShoulder_Pitch', 
                        'LShoulder_Roll', 
                        'LWrist_Pitch', 
                        'LWrist_Roll',
                        'LFinger_1', 
                        'LFinger_2', 
                        'LFinger_3', 
                        'RElbow_Pitch', 
                        'RElbow_Yaw', 
                        'RShoulder_Pitch', 
                        'RShoulder_Roll',
                        'RWrist_Pitch', 
                        'RWrist_Roll', 
                        'RFinger_1', 
                        'RFinger_2', 
                        'RFinger_3
                        ]
        msg = SetJointTorque()
        msg.joint_name = arm_joint_list
        msg.torque = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
        rospy.loginfo("check Torque Release!!")
        # msg.data = "arm_motion_control_module"
        self.__torque_publisher.publish(msg)

if __name__ == '__main__':
    SocialArmTorqueRelease()