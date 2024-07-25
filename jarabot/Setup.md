# Rapsberry Pi 4 (4GB)
* HW
  * memory : 32GB 이상
* SW
  * Ubuntu 22.04.x (현재 최신 22.04.3)

## SW 설치
* 첫 부팅 후 wifi 설정하기 및 통신 확인하기

* 첫 부팅 후 터미널
```bash
sudo apt update
sudo apt upgrade
```

* network 관련 설치
```bash
sudo apt install git  
sudo apt install net-tools # ifconfg  명령 사용 가능
sudo apt install openssh-server  #ssh 접속 가능
```

* network 관련 설정(ssh 연결 가능하도록 설정)
```bash
sudo systemctl status ssh  # SSH 서버 실행 중인지 상태 확인
sudo systemctl start ssh # SSH 서버가 실행시키기 
sudo ufw allow 22
sudo ufw allow ssh
sudo ufw status
sudo ufw enable
sudo systemctl restart ssh

sudo reboot
```

* RPi의 network IP 알아오기
```bash
ifconfig
```
  * RPi의 IP를 기록해 두었다가 PC에서 RPi에 기록한 IP를 사용하여 연결하기
```bash
ssh 192.168.xx.xx
```

* [ROS 2 Humble Desktop 설치](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


## RPLiDAR 및 jarabot ROS 2 패키지 설치
```bash
sudo apt install ros-humble-serial-driver ros-humble-teleop-twist-keyboard

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/jarabot/jarabot.git
cd ~/ros2_ws
colcon build --symlink-install

sudo cp ~/ros2_ws/src/jarabot/jarabot_node/rule/99-jarabot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo reboot  # 리부팅하기

ls /dev/     # /dev/ttyUSB0 확인
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
