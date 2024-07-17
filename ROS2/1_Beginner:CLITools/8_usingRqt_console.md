# rqt_console 사용해서 logs 보기
1. 개요
2. 실습
   1. 설정
   2. rqt_console에서 messages
   3. Logger level

## 1. 개요
* log
  * node의 내부 상태를 출력
  * 사용자가 내부 상태를 확인하는데 도움
* rqt_console
  * log 메시지를 조사하는 GUI 도구
  * 터미널에서 출력되는 메시지를 좀더 조직적이고 똑똑한 방법으로 확인 가능
  * 필터링, 저장, reload

## 2. 실습

### 2-1 설정
* rqt_console 시작 명령 실행
```bash
ros2 run rqt_console rqt_console
```

* 결과

![](https://docs.ros.org/en/humble/_images/console.png)

* 새 터미널에서 turtlesim 구동시키기 명령 실행
```bash
ros2 run turtlesim turtlesim_node
```

### 2-2 rqt_console에서 messages
* rqt_console에 log message를 출력시키기 위해서 turtle을 벽에 충돌시켜보자!
  
* 새 터미널 열어서 아래 명령 실행
```bash
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
```
  * 일정한 주기로 topic을 publish시켜서 turtle이 계속 벽에 충돌시킨다. 

* rqt_console
  * 'Warn' 레벨 이상의 메시지를 계속 확인 가능

![](https://docs.ros.org/en/humble/_images/warn.png)


* ros2 topic pub 명령을 실행하고 있는 터미널에서 Ctrl+C 로 종료시키기
  
### 2-3 Logger Level
* 중요도에 따른 레벨
```
Fatal : 시스템을 종료시킬만큼의 상황
Error : 시스템의 중요 issue 발생 상황
Warn  : 예상하지 못한 동작 혹은 이상적인 않은 결과 상황
Info : event나 status가 발생 상황
Debug : 시스템 실행시 단계별 실행을 모니터링 할때
```

### 2-3-1 기본 logger level 설정
* 처음 /turtle을 실행시킬때 기본 logger level을 설정하여 실행시킬 수 있다.

```bash
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```
   * 이제 Info 레벨 메시지는 출력하지 않게 된다.
   * 즉 일정 수준 중요도가 있는 메시지만 출력

