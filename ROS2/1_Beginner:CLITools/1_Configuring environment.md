# 환경설정
1. 개요
2. 실습

## 1. 개요
* workspace
  * SW개발 용어로, 프로젝트를 진행하고 코드를 관리 하는 공간
* uderlay
  * ROS2 시스템에 대한 라이브러리 및 기본 패키지를 포함 하고 있는 곳 
  * /opt/ros/humble/
* overlay
  * ROS2 시스템 위에서 우리가 ROS2를 개발하고 테스트하는 곳
  * ~/ros2_ws/

* 우리가 입력한 명령어가 실행되게 하기!

## 2. 실습
### 2-1 setup 파일을 source 하기
* 새로운 터미널을 열어서 아래 실행
```bash
source /opt/ros/humble/setup.bash
```

### 2-2 터미널 시작시 자동으로 setup.bash 파일 source 시키기
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2-3 환경변수 확인하기
```bash
printenv | grep -i ROS
```

* 결과
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

### 2-3-1 ROS_LOCALHOST_ONLY 변수
* ROS_LOCALHOST_ONLY 변수 설정하기
```bash
export ROS_LOCALHOST_ONLY=1
```

* 터미널 시작시 자동으로 설정되도록 하기
```bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```