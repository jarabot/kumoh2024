# parameter 사용하기 (Python)
1. 개요
2. 실습

## 1. 개요
* ROS parameter를 사용하기
* node를 작성할때 해당 node에서 필요한 parameters가 필요한 경우가 있다. 

## 2. 실습
### 2-1 package 생성하기
* package 생성하기
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
```
### 2-1-1 package.xml 업데이트하기
* --dependencies 옵션을 사용하여 package를 생성하였으므로 rclpy를 직접 추가할 필요없음 
* package.xml 열고 수정하기
```xml
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
```

### 2-2 Python node 작성하기
* ros2_ws/src/python_parameters/python_parameters/python_parameters_node.py 파일 생성
```python
import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        self.declare_parameter('my_parameter', 'world')

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 2-2-1 entry point 추가하기
* setup.py 파일 열고 아래 부분 수정하기
```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Python parameter tutorial',
license='Apache License 2.0',
```
* entry_points 필드의 console_scripts 브라켓 내에 아래 내용 추가하기
```python
entry_points={
    'console_scripts': [
        'minimal_param_node = python_parameters.python_parameters_node:main',
    ],
},
```

### 빌드 및 실행하기
* 의존성 검사하기
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

* package 빌드하기
```bash
colcon build --packages-select python_parameters
```

* node 실행하기 - 새 터미널 열고 source 및 실행
```bash
source install/setup.bash
ros2 run python_parameters minimal_param_node
```

* 결과
```
[INFO] [parameter_node]: Hello world!
```

### 2-3-1 console로 변경하기
* node가 실행되어 있는 상태(실행된 상태가 아니라면 아래 명령으로 실행)
```bash
ros2 run python_parameters minimal_param_node
```

* 새 터미널에서 아래 명령 실행
```bash
source install/setup.bash
ros2 param list
```

* 커스텀 paramter인 my_parameter을 earth로 변경
```bash
ros2 param set /minimal_param_node my_parameter earth
```

* 결과
```
Set parameter successful
```

### 2-3-2 launch 파일로 변경하기
* launch 파일에 parameter를 설정하기

* ros2_ws/src/python_parameters/launch/python_parameters_launch.py 파일 생성
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='minimal_param_node',
            name='custom_minimal_param_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])
```

* parameter_node를 실행시킬때 my_parameter를 earth로 설정

* setup.py 파일에 추가 (import, data files 부분 수정)
```python
import os
from glob import glob
# ...

setup(
  # ...
  data_files=[
      # ...
      (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ]
  )
```

* 새 package 빌드하기
```bash
colcon build --packages-select python_parameters
```

* 새 터미널 열고 lauch 파일 실행
```bash
source install/setup.bash
ros2 launch python_parameters python_parameters_launch.py
```

* 결과
```
[INFO] [custom_minimal_param_node]: Hello earth!
```
