# Jarabot 이동시키기
* 목차
  1. jarabot 접속하기
  2. jarabot에서 실행
  3. PC에서 실행
  4. jarabot 움직이기

##  1. jarabot 접속하기
* SSH 접속
  * ID, passwd로 접속

##  2. jarabot(RPi)에서 실행
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_node bringup.launch.py
```

##  3. PC(내 노트북)에서 실행
* jarabot 조정
```bash
sudo apt install ros-humble-serial-driver ros-humble-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/keyboard/cmd_vel
```

* rviz2 실행
```bash
# 새 터미널
ros2 run rviz2 rviz2 
```

##  4. jarabot 움직이기
* teleop_twist_keyboard 를 실행한 화면에서 키보드로 조정하기
