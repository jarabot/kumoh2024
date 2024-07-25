# 설정
* RPLiDAR 및 jarabot ROS 2 패키지 설치
```bash
sudo apt install ros-humble-serial-driver ros-humble-teleop-twist-keyboard

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/jarabot/jarabot.git
cd ~/ros2_ws
colcon build --symlink-install
```

## ROS_DOMAIN_ID 설정
* Jarabot과 내 PC가 서로 통신이 가능하게 하기 위해서 양쪽 모두 동일한 ID로 설정
```bash
#export ROS_LOCALHOST_ONLY=1 이부분은 코멘트처리하기

#~/.bashrc 파일에 맨 아래에 추가 ID 값은 각 조별로 1~10까지 설정
export ROS_DOMAIN_ID=2
```

## 방화벽 풀기(ROS_DOMAIN_ID 통신을 위해서)
* PC<->RPi 통신을 위해
```bash
#ufw가 설치되어 있지 않은 경우에 설치 : sudo apt-get install ufw
sudo ufw enable

sudo ufw allow 1:65535/tcp
sudo ufw allow 1:65535/udp

sudo ufw status # 확인

sudo reboot
```

## Visual Studio Code로 JaraBot 접속하기
* Visual Studio Code  : Extension -> Remote-SSH 설치
* Visual Studio Code에서 SSH로 연결하기 (RPi의 IP주소를 사용한다.)
  * myid@rpi_ip_address 로 연결
  * passwd 입력
* Visual Studio Code에서 파일 열기

[![Video](http://img.youtube.com/vi/7kum46SFIaY/0.jpg)](http://www.youtube.com/watch?v=7kum46SFIaY)
