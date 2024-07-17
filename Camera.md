# [Camera](https://navigation.ros.org/tutorials/docs/camera_calibration.html) 설정
* 목표 : Ubuntu 22.04 ROS 2 Humbe에서 Camera 사용하기(camera calibration parameters 얻기)
1. RPi Camera 설치
2. Camera 관련 ROS 설치
3. Camera calibraton
4. rviz2에서 /image 보기
  
## 1. RPi Camera 설치
* RPi와 Camera 인터페이스 활성화 시키기
```bash
sudo apt install raspi-config

sudo raspi-config # 인터페이스 활성화 프로그램 실행
```
  * legacy interface enable 설정하기
  * /boot/firmware/config.txt 파일을 열어서 아래 내용 추가하기
    ```bash
    sudo nano /boot/firmware/config.txt
    ```

    ```
    ...
    start_x=1
    ...
    gpu_mem=128
    ```
* 개발을 위한 프로그램 설치
```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt install cmake
```

* 빌드하기
```bash
mkdir ~/projects
cd ~/projects
sudo git clone https://github.com/raspberrypi/userland
cd userland
sudo ./buildme --aarch64  # --aarch64 없으면 error남!
```

* ~/.bashrc 파일에 아래 내용 추가
```bash
export PATH=$PATH:/opt/vc/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vc/lib
```

* ldconfig 실행(반드시 실행!)
```bash
source ~/.bashrc
sudo ldconfig
```

* 리부팅하기

* Cheese 카메라 앱 실행하여 카메라가 정상동작하는지 확인하기
## 2. Camera 관련 ROS 설치
* 설치
```bash
sudo apt install ros-humble-camera-calibration-parsers

sudo apt install ros-humble-camera-info-manager

sudo apt install ros-humble-launch-testing-ament-cmake
```

* Image pipeline은 소스 빌드 필요함
```bash
mkdir -p ~/cam_ws/src
cd ~/cam_ws/src
git clone https://github.com/ros-perception/image_pipeline.git -b humble
```
* 판준비
  * 7x9 checkerboard (20mm 정사각형)
  * [판](https://calib.io/pages/camera-calibration-pattern-generator)

## 3. Camera calibraton
0. 판준비
  * 7x9 checkerboard (20mm 정사각형)
  * [판](https://calib.io/pages/camera-calibration-pattern-generator)
1. terminal 실행
2. 해당 카메라에 대한 ROS driver 실행
3. 확인하기
```bash
ros2 topic list
ros2 topic hz /camera/image_raw
```
1. camera calibration node 실행
```bash
cd ~/cam_ws
source install/setup.bash
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.02 --ros-args -r image:=/my_camera/image_raw -p camera:=/my_camera
```

1. checkerboard를 카메라 프레임내에 들어가도록 위치 시키기


## 확인 필요(설치하기)
* 설치하기
```bash
sudo apt install libraspberrypi-bin v4l-utils ros-humble-v4l2-camera ros-humble-image-transport-plugins
sudo usermod -aG video $USER
```
* camera 확인하기
```bash
v4l2-ctl --list-devices
```
* camera 이미지 보기
```bash
ros2 run rqt_image_view rqt_image_view
```

* RPi camera ROS 2 node 실행하기
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[160,120]" -p camera_frame_id:=camera_optical_link
```

* camera 이미지 보기
```bash
ros2 run rqt_image_view rqt_image_view
```


## 자료
* [ROS2 picam](https://github.com/christianrauch/raspicam2_node)
* [pycam ros2](https://github.com/Misterblue/ros2_raspicam_node)
