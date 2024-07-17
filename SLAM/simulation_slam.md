# Turtlebot 4 Common
## Source 설치
```bash
sudo rosdep init
rosdep update

mkdir -p ~/turtlebot4_ws/src
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4.git -b humble

cd ~/turtlebot4_ws
rosdep install --from-path src -yi --rosdistro humble

colcon build --symlink-install
```

## Navigation
* turtlebot 4 packages는 SLAM과 navigation을 사용하기 위한 launch와 configuration을 포함
* Launch 파일
```
Nav2 : Nav2 stack을 실행
SLAM : slam_toolbox를 실행 (online mapping)
Localization : 주어진 map에서 localization을 실행
```

* 동기식 SLAM 실행
```bash
ros2 launch turtlebot4_navigation slam.launch.py
```

* 비동기식 SLAM 실행
```bash
ros2 launch turtlebot4_navigation slam.launch.py sync:=false
```

* 기존 map으로 localization 실행
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/path/to/map.yaml
```


* Nav2 stack 실행
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

# Turtlebot4 Simulation 환경 설정
* 목차
  1. Dev Tools
  2. Iginition Gazebo
  3. Install from Source or Package

## Dev Tools
```bash
sudo apt install ros-dev-tools
```

## Iginition Gazebo
```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

## Install from Source or Package
```bash
mkdir -p ~/turtlebot4_ws/src 
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git -b humble

cd ~/turtlebot4_ws
rosdep install --from-path src -yi

colcon build --symlink-install
```

## Gazebo Ignition 실행
* 기본 설정으로 실행
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
```

* Nav2와 동기화 SLAM 실행
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```

[![Video](http://img.youtube.com/vi/go7jszqFSi0/0.jpg)](http://www.youtube.com/watch?v=go7jszqFSi0)

* 사용 방법
  * 실행된 Gazebo에서 왼쪽 아래 'Play' 아이콘 클릭 
  * HMI 조작
    * '3', '4' : 위/아래 메뉴 조작
    * '1', '2' : '1'은 선택, '2'는 취소


## Gazebo Ignition GUI toolbox
* turtlebot4_ignition_toolbox package
  * HMI node 역할
    * turtlebot4_node와 ros_ign_birdge 사이의 bridge 역할
      * Turtlebot4 messages를 표준 message로 변환


