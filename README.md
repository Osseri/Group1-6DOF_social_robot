# Social Robot Manual - 6DOF



## ARM

**1.**   **소스코드**

A.   https://drive.google.com/file/d/1MpNsggDDUMQMCl1nTIR5iNQmHIADcInN/view?usp=sharing

B.   위 첨부 파일에는 아래 패키지들이 포함되어 있습니다.

C.   Social-Robot-Arm (private repository)

D.   Dynamixel SDK ([master branch](https://github.com/ROBOTIS-GIT/DynamixelSDK))

E.   dynamixel-workbench ([master branch](https://github.com/ROBOTIS-GIT/dynamixel-workbench))

F.   dynamixel-workbench-msgs ([master branch](https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs))

G.   industrial_core ([kinetic-devel branch](https://github.com/ros-industrial/industrial_core))

H.   robotis_manipulator ([feature-dynamics branch](https://github.com/ROBOTIS-GIT/robotis_manipulator/tree/feature-dynamics))

 

**2.**   **중력보상 실행 방법**

A.   Controller 실행

$ roslaunch social_robot_arm_controller social_robot_arm_controller.launch

B.   GUI 프로그램 실행

$ roslaunch social_robot_arm_control_gui social_robot_arm_control_gui.launch

C.   GUI 노드 활성화

“Start” 버튼 클릭 후 현재 조인트 값이 Left Arm, Right Arm 에 정상적으로 표시되는지 확인

![image.png](http://ep.gst-in.com/_WorkCrewUpload/_MAIL/robocare.co.kr/dj_jin1/attached/2020011016112025371.eml/image003.jpg@01D5C7D0.90C4F6D0.jpg)

D.   중력보상 모드로 전환

Motion > Mode 의 “Edit Mode” 버튼 클릭

(모드 전환 시 잠깐 동안 다이나믹셀의 토크가 꺼졌다가 켜집니다. 파손 방지를 위해 양 팔을 받쳐 든 상태로 모드를 바꾸는 것을 권장합니다.)

![image.png](http://ep.gst-in.com/_WorkCrewUpload/_MAIL/robocare.co.kr/dj_jin1/attached/2020011016112025371.eml/image005.jpg@01D5C7D0.90C4F6D0.jpg)

E.   이제 중력보상 모드에서 팔을 임의대로 움직일 수 있습니다.

F.   모션재생 모드로 전환

Motion > Mode 의 “Play Mode” 버튼 클릭

(마찬가지로 모드 전환 시 잠깐 동안 다이나믹셀의 토크가 꺼졌다가 켜집니다. 주의바랍니다.)

 

**3.**   **모드 전환을 위한 ROS topic 설명**

A.   위에서 “Edit Mode” 와 “Play Mode” 버튼 클릭 시 각각 publish 되는 topic 은 아래와 같습니다.

B.   Topic Name : “social_robot/set_mode”

C.   Topic File : social_robot_arm_msgs/msg/SetMode.msg

------------------SetMode.msg----------------

\########################################

\# CONSTANTS

\########################################

string PLAY_MODE = "normal_mode"

string EDIT_MODE = "gravity_compensation_mode"

 

\########################################

\# Messages

\########################################

string mode

\---------------------------------------------

D.   위 topic 의 mode 에 string 으로 “normal_mode” 나 “gravity_compensation_mode” 를 넣거나, msg.PLAY_MODE 혹은 msg.EDIT_MODE 를 넣어서 publish 하면 됩니다.

E.   보다 상세한 설명은 09/05 에 보내드렸던 명세서를 참고하면 됩니다. (본 메일에 첨부파일로 다시 첨부 함)

 

**4.**   **HAND** **로 물체 파지 시 세팅**

A.   HAND 에는 1축짜리 전류제어가 가능한 모델과, 2축짜리 전류제어가 안 되는 모델이 들어가 있습니다.

B.   1축짜리 전류제어 가능한 모델의 Control Table 은 [XM430-W350](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/) 을 참고하시면 됩니다.

C.   2축짜리 전류제어가 불가능한 모델은 [2XL430-W250](http://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/) 을 참고하시면 됩니다. (실제로는 2XL 이 아닙니다.)

D.   전류제어가 불가능한 2축짜리는 최대한 Load 가 덜 걸리는 각도로 맞춰주고, 전류 제어가 가능한 1축 짜리의 위치값을 적당히 조절하여 물체를 잡도록 조정..

E.   1축과 2축 모두 Profile Velocity : 200, Profile Acceleration : 20 정도로 맞춰 두고, 1축 모델에 Goal Current 를 200~300 정도로 맞추고 물체에 따라 적당한 위치값을 조절해서 테스트 해 보는 것이 좋을 듯 합니다.

F.   2축 모델이 Current Control 이 없어서 Shutdown 이 일어날 확률이 높으므로 1축 모델 쪽으로 Load 가 더 걸리는 것이 유리합니다.

G.   Shutdown 이 일어날 경우 복구를 위해서는 로봇 전체를 껐다 켜는 방법 외에 [reboot instruction](http://emanual.robotis.com/docs/en/dxl/protocol2/#reboot) 을 이용하여 shutdown 이 일어난 다이나믹셀만 reboot 해 주는 방법을 사용할 수 있습니다.


