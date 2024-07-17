# ROS_DOMAIN_ID로 통신 가능 여부 확인
* PC <-> RPi가 서로 데이터를 주고 받는지 확인하기
## PC쪽에서 실행
```bash
ros2 run demo_nodes_cpp talker
```

## JaraBot쪽에서 실행
```bash
ros2 run demo_nodes_py listener
```

## 결과
* 서로 값을 주고 받는 경우 ROS_DOMAIN_ID 설정에 성공!
