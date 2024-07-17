# turtlesim과 rqt 사용하기
1. 소개
2. 실습
   1. turtlesim 설치하기
   2. turtlesim 구동시키기
   3. turtlesim 사용하기
   4. rqt 설치하기
   5. rqt 사용하기
   6. remapping
   7. turtlesim 종료하기

## 1. 소개
* turtlesim package
  * ROS 2를 학습을 도와주기 위해서 제공하는 간단한 시뮬레이터
  * ROS 2 개념을 소개하기 위해서 가장 흔히 사용
* ros2 도구
  * terminal을 이용하여 ROS2 시스템에 접근
  * ROS 2를 설치하면 기본으로 제공
* rqt 도구
  * GUI 도구
  * ros2 도구를 GUI 환경에서 실행하고 결과 확인 가능

## 2. 실습
### 2-1 turtlesim 설치하기
```bash
sudo apt update

sudo apt install ros-humble-turtlesim
```

* package 설치 여부 확인
```bash
ros2 pkg executables turtlesim
```

* 결과
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

### 2-2 turtlesim 구동시키기
* turtlesim 실행
```bash
ros2 run turtlesim turtlesim_node
```
![](https://docs.ros.org/en/humble/_images/turtlesim.png)

* 터미널 출력 결과
```
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

### 2-3 turtlesim 사용하기
* 새 터미널 열고 source 명령 실행
* 위에 실행한 turtle 조정하기
```bash
ros2 run turtlesim turtle_teleop_key
```

* 현재 3개 창이 열린 상태
  * turtlesim_node 실행 창
  * turtle_teleop_key 실행 창
  * turtlesim 시뮬레이션 화면 창
* 주의 :
  * turtle_teleop_key 실행 창에서 화살표 키보드를 눌러야 제어 가능!!

* 다양한 정보 확인하기(각 명령 하나씩 실행하기)
```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### 2-4 rqt 설치하기
* rqt 설치하기
```bash
sudo apt update

sudo apt install ~nros-humble-rqt*
```

* rqt 실행하기
```bash
rqt
```

### 2-5 rqt 사용하기
* 상단 메뉴 Plugins > Services > Service Caller 로 이동

![](https://docs.ros.org/en/humble/_images/rqt.png)

* spawn service 해보기

![](https://docs.ros.org/en/humble/_images/spawn.png)
  * 새로운 turtle을 생성
  * 새로운 turtle의 이름과 위치를 설정 가능

* set_pen service 해보기
  * turtle1에게 pen을 부여하기 - /set_pen service 이용

![](https://docs.ros.org/en/humble/_images/set_pen.png)
  * rgb : red, green, blue 설정해보기
  * turtle_teleop 터미널 창에서 화살표 키보드 눌러서 이동시키기

* 결과

![](https://docs.ros.org/en/humble/_images/new_pen.png)


### 2-6 Remapping
* 새 터미널에서 아래 실행
```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

### 2-7 turtlesim 종료하기
* 각 터미널에서 Ctrl + C 키를 누르면 종료된다.


# Quiz
 - rqt를 사용하여 'myname'의 거북이를 생성하고, remap을 사용하여 키보드로 'myname' 거북이를 제어 할 수있도록 적용하세요