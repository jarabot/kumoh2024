# action 이해
1. 소개
2. 실습
   1. 설정
   2. action 사용하기
   3. ros2 node info
   4. ros2 action list
   5. ros2 action info
   6. ros2 interface show
   7. ros2 action send_goal

## 1. 개요
* ROS graph 상에서 nodes 사이에서의 또다른 통신 방법
  * long running task
  * 3개 부분으로 구성
    * goal
    * feedback
    * result
  * topics과 service 기반
  * service와의 차이점
    * 실행 중에 cancel이 가능!
    * 주기적인 feedback 제공
  * client-server model
  * client가 server에게 goal을 전송하고 server는 ack, feedback, result를 전송

![](https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif)

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
```

### 2-2 action 사용하기
* /teleop_turtle node를 실행할때 보게 되는 화면 :
```
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```

* 2번째 줄에서 
  * G|B|V|C|D|E|R|T 는 키보드에서 F를 기준으로 Box형태 방향으로 의미
  * 'E' 키를 누르면 왼쪽 상단 방향으로 회전
```
[INFO] [turtlesim]: Rotation goal completed successfully
```

* 'C' 키 누르고 바로 F 키를 누르면 아래와 같은 메시지 출력
```
[INFO] [turtlesim]: Rotation goal canceled
```

* 'D'키 누르고 바로 G 키를 누르면 아래와 같은 메시지 출력
```
[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
```

### 2-3 ros2 node info
* node가 제공하는 action의 목록을 보는 명령

* 새 터미널을 열어서 /turtlesim node의 action 목록을 보는 명령 실행
```bash
ros2 node info /turtlesim
```

* 결과 (/turtlesim 의 publisher, subscriber, services, action servers, action clients 목록)
```
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Services:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```
   * Action Servers는 /turtle1/rotate_absolute action에 대해서 feedback을 제공

* /teleop_turtle node 정보 보기 명령 실행
```bash
ros2 node info /teleop_turtle
```
* 결과
```
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Services:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```
   * Action Clients는 해당 action name으로 goal을 server에게 전송

### 2-4 ros2 action list
* ROS graph 상에 있는 모든 actions의 목록을 보는 명령 실행하기
```bash
ros2 action list
```

* 결과
```
/turtle1/rotate_absolute
```
  * 현재 1개 action만 존재

### 2-4-1 ros2 action list -t
* action의 type 보기 (/turtle1/rotate_absolute의 type보기) 명령 실행하기
```bash
ros2 action list -t
```

* 결과(/turtle1/rotate_absolute의 type보기)
```
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```
  * action type : turtlesim/action/RotateAbsolute


### 2-5 ros2 action info
* /turtle1/rotate_absolute action에 대한 상세 정보 보기 명령
```bash
ros2 action info /turtle1/rotate_absolute
```

* 결과
```
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```
  * /teleop_turtle node : action client
  * /turtlesim node : action server

### 2-6 ros2 interface show
* 터미널에서 직접 action을 전송하려면 상세 정보가 필요
* ros2 action list -t 를 사용하여 /turtle1/rotate_absolute 의 type 확인 가능

* /turtle1/rotate_absolute 의 action type인 turtlesim/action/RotateAbsolute 의 상세 정보 보기 명령 실행
```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

* 결과
```
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

### 2-7 ros2 action send_goal
* 터미널에서 action goal을 전송하는 명령 형식
```bash
ros2 action send_goal <action_name> <action_type> <values>
```
   * values 부분은 YAML 문법을 따른다.

* turtlesim 창을 보면서 아래 명령을 실행 (goal을 전송)
```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

* 결과 (turtle이 회전하며 아래 메시지가 터미널에 출력)
```
Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
```
   * 모든 goals은 return messags 내부에 UID를 가진다. 
   * result인 delta를 볼 수 있으며 starting position으로 교체

----

* goal에 대한 feedback을 보려면 --feedback 옵션을 추가하면 된다.

* 이전 명령으로 theta 1.57 radians가 되었으므로 새로운 theta를 줘야지 움직이게 된다.
* 아래와 같은 명령을 실행
```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

* 결과
```
Sending goal:
   theta: -1.57

Goal accepted with ID: e6092c831f994afda92f0086f220da27

Feedback:
  remaining: -3.1268222332000732

Feedback:
  remaining: -3.1108222007751465

…

Result:
  delta: 3.1200008392333984

Goal finished with status: SUCCEEDED
```

* goal이 완료될때까지 계속해서 feedback으로 remaining radians를 수싢나다.
