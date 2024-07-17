# colcon 명령으로 packages 빌드하기
1. 개요
2. 실습

## 1. 개요
* colcon 명령으로 ROS 2의 workspace를 빌드하는 방법 익히기
* colcon은
  * ROS build 도구(catkin_make, catkin_make_isolated, catkin_tools, ament_tools 통합)

* colcon 설치 명령 실행
```bash
sudo apt install python3-colcon-common-extensions
```

## 2. 실습
### 2-1 기초
### 2-1-1 workspace 생성
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2-1-2 sources 추가하기
* 예제 코드를 src에 추가하기
```bash
cd ~/ros2_ws
git clone https://github.com/ros2/examples src/examples -b humble
```

* 결과 : workspace 내부 구조
```
.
└── src
    └── examples
        ├── CONTRIBUTING.md
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files
```

### 2-1-3 underlay를 source하기
```bash
source /opt/ros/humble/setup.bash
```

### 2-1-4 workspace 빌드하기
* 빌드하기 명령 실행
```bash
colcon build --symlink-install
```
  * --symlink-install : install된 파일 중에서 변경된 파일을 변경 (python 파일이나 리소스 파일)

* 결과 (빌드 후 결과)
```
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```

### 2-1-5 environment source 하기
* colcon으로 빌드를 성공적으로 마치면 빌드 결과가 install 디렉토리에 위치하게 된다. 
* 빌드된 실행자나 lib를 사용하기 전에 path를 추가해 줘야 한다. 이를 위해서 install/setup.bash를 실행한다.

* setup.bash source 명령 실행
```bash
source install/setup.bash
```

### 2-1-6 빌드한 실행자 실행하기
* subscriber node 실행 명령
```bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```

* 새 터미널에서 publisher node 실행 명령 
```bash
source install/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```


