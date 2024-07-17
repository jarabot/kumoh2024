# Jarabot 이동시키기
* 목차
  1. jarabot 접속하기
  2. jarabot에서 실행
  3. PC에서 실행
  4. jarabot 움직이기
  5. 지도 저장하기

##  1. jarabot 접속하기
* SSH 접속
  * ID, passwd로 접속

##  2. jarabot에서 실행
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_node bringup.launch.py
```

* 새로운 터미널에서
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_node cartographer.launch.py
```

##  3. PC에서 실행
* jarabot 조정
```bash
sudo apt install ros-humble-serial-driver ros-humble-teleop-twist-keyboard

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/keyboard/cmd_vel
```

* rviz2 실행
```bash
# 새 터미널 rviz2 실행
rviz2 -d ~/ros2_ws/src/jarabot/jarabot_cartographer/rviz/jarabot_cartographer.rviz

# rviz 설정파일 불러오기
ros2 run rviz2 rviz2 ~/ros2_ws/src/jarabot/jarabot_cartographer/rviz/jarabot_cartographer.rviz
```

##  4. jarabot 움직이기
* teleop_twist_keyboard 를 실행한 화면에서 키보드로 조정하기

##  5. 지도 생성확인
* rviz2 에서 확인

##  6. 지도 저장하기
* 충분히 지도가 생성되면 현재 지도를 저장하기 [참고](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html#save-the-map)
* turtlebot3와 같은 방법으로 저장하기
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

* turtlebot4와 같은 방법으로 저장하기
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```

* 