# Jarabot SW 구조 
* 1. 패키지 구조
* 2. jarabot_interfaces
* 3. jarabot_node
* 4. jarabot_cartographer
* 5. jarabot_navigation2
* [자료](https://1drv.ms/p/s!Aigic13_w6ElgpMxnw1-8k1wuFNOgQ?e=OfWzIT)

## 1. 패키지 구조

* Jarabot package 구조
  * jarabot_interfaces: Node 간 통신 인터페이스 파일 포함
  * jarabot_node: ROS Nodes 포함
    * jara_controller: Jarabot 이동 (또는 회전) 속도 조절 
    * jara_driver: Jarabot 속도 조절을 위한 모터 제어 및 바퀴 회전량 측정
    * jara_odometry: 좌우 바퀴 회전량으로부터 Jarabot 위치 추정 
  ![](./jarabot_packages.jpg)

## 2. jarabot_interfaces

* interface files
  * Cmd.msg - Jarabot 속도 조절 명령
  ```cpp
    int32 linear_input
    int32 angular_input
  ```

  * Ecd.msg - Motor 회전량
  ```cpp
    std_msgs/Header header
    int32 left_encoder_val
    int32 right_encoder_val
  ```

 
## 3. jarabot_node
