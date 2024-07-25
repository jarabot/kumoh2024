# Navigation 하기
* 목차
  1. 사전에 지도를 만들어 놓았다.
  1. 지도 upload하기
  1. jarabot 접속하기
  1. jarabot에서 실행
  1. PC에서 실행
  1. Jarabot 이동시키기

## 지도 Upload하기

## jaraot 접속하기
* SSH 접속
  * ID, passwd로 접속

## jarabot에서 실행
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_node bringup.launch.py #절대경로 사용
```

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch jarabot_node navigate.launch.py map:=$HOME/map.yaml #절대경로 사용
```

## PC에서 실행
* rviz2 실행
```bash
ros2 run rviz2 rviz2   # 이 명령 사용하면 2D Nav Goal 버튼이 안생겨요 ㅠㅠ. 아래 명령 수행해주세요.

# 혹은 rviz 설정파일 불러오기
rviz2 -d ~/ros2_ws/src/jarabot/jarabot_navigation2/rviz/jarabot_navigation2.rviz
ros2 run rviz2 rviz2 ~/ros2_ws/src/jarabot/jarabot_navigation2/rviz/jarabot_navigation2.rviz
```

## Jarabot 이동시키기
* 2D Pose Estimate 수행
* 2D Nav Goal 수행

