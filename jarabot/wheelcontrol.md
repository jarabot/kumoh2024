# 모터 동작 확인 방법

## 1. bringup 실행
```bash
. ~/ros2_ws/install/setup.bash
ros2 launch jarabot_node bringup.launch.py
```

## 2. 모터에 커맨드 보내기
```bash
# 새로운 터미널에서
. ~/ros2_ws/install/setup.bash
# 전진
ros2 topic pub /cmd jarabot_interfaces/msg/Cmd "{linear_input: 1750, angular_input: 1500}"
# 후진
ros2 topic pub /cmd jarabot_interfaces/msg/Cmd "{linear_input: 1250, angular_input: 1500}"
# 정지
ros2 topic pub /cmd jarabot_interfaces/msg/Cmd "{linear_input: 1500, angular_input: 1500}"
```
