# parameter 이해
1. 개요
2. 실습
   1. 설정
   2. ros2 param list
   3. ros2 param get
   4. ros2 param set
## 1. 개요
* ROS 2에서 parameter를 가져오기, 설정하기, 저장하고, 불러오기 배우기
* parameter란?
  * node의 설정 값
  * type
    * integers, floats, booleans, strings, lists
  * 각 node는 자신의 parameters를 유지관리한다.

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

### 2-2 ros2 param list
* 실행 중인 nodes에 속하는 parameters를 확인하는 명령 실행
```
ros2 param list
```

* 결과 (각 node namespace에 속하는 parameters)
```
/teleop_turtle:
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
```
   * /turtlesim의 parameters는 turtlesim 창의 배경 색상을 결정

### 2-3 ros2 param get
* parameter의 type과 현재 값 표시하는 명령 형식
```bash
ros2 param get <node_name> <parameter_name>
```

* /turtlesim의 background_g parameter 값 표시하는 명령 실행
```bash
ros2 param get /turtlesim background_g
```

* 결과
```bash
Integer value is: 86
```

* 동일한 명령을 background_r, background_b에 대해서도 실행해보자.

### 2-4 ros2 param set
* 실시간으로 현재 parameter 값 변경하는 명령 형식
```bash
ros2 param set <node_name> <parameter_name> <value>
```

* /turtlesim 의 배경색 변경하는 명령 실행
```bash
ros2 param set /turtlesim background_r 150
```

* 결과
```
Set parameter successful
```

![](https://docs.ros.org/en/humble/_images/set.png)
   * 배경색상이 변경

* 주의 :
  * set은 현재 실행 중인 프로그램에만 영향을 미친다. 즉 새로 node를 실행하는 경우 set으로 설정한 값이 보존되지 않는다.

### 2-5 ros2 param dump
* node의 현재 parameters 값을 파일로 저장하는 명령 형식
```bash
ros2 param dump <node_name>
```

* /turtlesim 의 parameters의 현재 설정을 저장하는 명령 실행
```bash
ros2 param dump /turtlesim
```
* 결과 (turtlesim.yaml 파일로 저장)
```
Saving to:  ./turtlesim.yaml
```

* 현재 디렉토리에서 저장된 turtlesim.yaml 파일 열어보기 (Visual Studio Code 사용하기)
```yaml
turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
    use_sim_time: false
```
* 이렇게 저장한 parameters를 node에 그대로 적용할 수 있다!

### 2-6 ros2 param load
* 현재 실행 중인 node의 parameter를 load하기 명령 형식
```bash
ros2 param load <node_name> <parameter_file>
```

* ros2 param dump로 생성한 turtlesim.yaml 파일을 /turtlesim node parameter로 사용하는 명령 실행
```bash
ros2 param load /turtlesim ./turtlesim.yaml
```

* 결과
```
Set parameter background_b successful
Set parameter background_g successful
Set parameter background_r successful
Set parameter use_sim_time successful
```

### 2-7 node 시작시킬때 parameter 파일을 로드시키기
* 저장한 parameter 값을 사용하여 동일한 node을 구동시키는 명령 형식
```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

* 실행중인 turtlesim node를 중지시키고 저장한 parameters로 구동시키기 명령 실행
```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
```

* 결과 보기
