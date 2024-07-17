# data 기록 및 재생
1. 개요
2. 실습
   1. 설정
   2. topic 선택하기
   3. ros2 bag record
   4. ros2 bag info
   5. ros2 bag play

## 1. 개요
* ros2 bag 명령
   * topic에서 publish되는 data를 저장
* 나중에 저장된 data를 재생
* 개발하면서 테스트 및 실험 목적

* 준비
```bash
sudo apt-get install ros-humble-ros2bag \
                     ros-humble-rosbag2-converter-default-plugins \
                     ros-humble-rosbag2-storage-default-plugins
```

## 2. 실습
### 2-1 설정
* 2개 turtlesim nodes 구동시키기 : /turtlesim ,  /teleop_turtle 

* 새 터미널에서 아래 명령 실행
```bash
ros2 run turtlesim turtlesim_node
```

* 새 터미널에서 아래 명령 실행
```bash
ros2 run turtlesim turtle_teleop_key

* 새 터미널을 열고 data를 저장할 새 디렉토리 만들기
```bash
mkdir bag_files
cd bag_files
```

### 2-2 topic 선택하기
* ros2 bag은 topics으로 publish되는 messages만 기록한다.

* 새 터미널에서 현재 시스템에 있는 topics 목록을 보는 명령 실행
```bash
ros2 topic list
```

```결과
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

* /turtle_teleop node가 /turtle1_cmd_vel topic으로 명령을 전달하여 turtle을 이동시킨다는 것을 배웠다.

* /turtle1/cmd_vel이 publish하는 data 내용을 보기 위해서 아래 명령 실행
```bash
ros2 topic echo /turtle1/cmd_vel
```

* 실행 후 메시지 확인을 위해서는 teleop를 실행한 터미널로 가서 화살표로 움직여보기

* 결과
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

### 2-3 ros2 bag record
* 특정 topic으로 publish되는 data 기록하는 명령 형태
```bash
ros2 bag record <topic_name>
```

* /turtle1/cmd_vel topic 저장하는 명령 실행
```bash
ros2 bag record /turtle1/cmd_vel
```

* 결과
```
[INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

* ros2 bag 명령으로 /turtle1/cmd_vel topic에서 publish되는 data를 기록하기 시작한다. teleop 터미널로 와서 turtle 이동시키기. 
* 나중에 replay를 위해서 움직인 패턴을 기억해 두자!

![](https://docs.ros.org/en/humble/_images/record.png)

   * 저장되는 data 파일 형태 : rosbag2_year_month_day-hour_minute_second
   * 추가로 metadata.yaml 파일을 포함

* Ctrl+C로 종료하기 

### 2-3-1 여러 topics 저장하기
* ros2 bag으로 저장하는 파일 이름 설정하는 명령 실행
```bash
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```
   * -o 옵션 : bag 파일 이름 지정

* 결과
```
[INFO] [rosbag2_storage]: Opened database 'subset'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/pose'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

### 2-4 ros2 bag info
* ros bag 파일에 대한 상세 정보 명령 형식
```bash
ros2 bag info <bag_file_name>
```

* subset이라는 bag 파일에 대해서 명령 실행
```bash
ros2 bag info subset
```

* 결과
```
Files:             subset.db3
Bag size:          228.5 KiB
Storage id:        sqlite3
Duration:          48.47s
Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
End                Oct 11 2019 06:09:57.60 (1570799397.60)
Messages:          3013
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                 Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
```
   * 참고 : 개별 message는 sqlite3로 DB를 열어서 확인 가능

### 2-5 ros2 bag play
* bag 파일을 재생하기 위한 준비
   * teleop를 실행하는 터미널 stop 시키기(Ctrl+C)
   * turtlesim 창이 보이도록 위치시키기

* subset 이라는 bag 파일 재생 명령
```bash
ros2 bag play subset
```

* 결과
```
[INFO] [rosbag2_storage]: Opened database 'subset'.
```

![](https://docs.ros.org/en/humble/_images/playback.png)

