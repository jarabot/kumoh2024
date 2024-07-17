# [Listener 작성하기(Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html)
1. 목표
1. 배경지식
1. 사전 준비
1. 실행
1. 요약

## 목표
* 프레임 transformation을 얻기 위한 tf2 사용법 학습하기

## 배경지식
이전 튜토리얼에서는, turtle2의 자세를 publish하는 broadcaster를 작성했다.
이번 튜토리얼에서는 tf2를 사용하기 위한 tf2 listener를 작성할 것이다.

## 사전 준비
* Introducing tf2, tf2 static broadcaster 튜토리얼 완료.
* learning_tf2_py 패키지 필요

## 실습
### 1. Listener 노드 작성하기
소스 파일을 만들어 보자. src/learning_tf2_py/learning_tf2_py 위치에 아래의 명령어로 listener 코드 예제를 다운로드 받자.

```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py
```

### 1.1 코드 분석
```python
import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'turtle1').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
                try:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())
                except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)

                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)

                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True
                else:
                    self.get_logger().info('Spawn is not finished')
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                request = Spawn.Request()
                request.name = 'turtle2'
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
```

turtle을 spawn하는 서비스의 동작을 이해하기 위해, [writing a simple service and client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) 튜토리얼을 참고하자.
이제 프레임 transformation에 접근하는 코드를 보자. tf2_ros는 transform의 수신을 더 쉽게 만들어 주는 TransformListener 헤더 파일 구현을 포함하고 있다.

```python
from tf2_ros.transform_listener import TransformListener
```

아래에서는, TransformListener를 생성한다. Listener가 생성되면, tf2 transformationㅇ르 수신하기 시작하고, 10초 동안 버퍼에 유지시킨다.

```python
self.tf_listener = TransformListener(self.tf_buffer, self)
```

마지막으로, 특정 transformation에 대해 listener를 query한다. 그러기 위해, 아래의 인자들로 lookup_transform 메소드를 호출한다.

1. Target frame
1. Source frame
1. transform한 시점

rclpy.time.Time()를 사용하면 최신 transform을 얻을 수 있다. 또한 exception을 처리하기 위해 try-except문으로 랩핑된다.

```python
t = self.tf_buffer.lookup_transform(
    to_frame_rel,
    from_frame_rel,
    rclpy.time.Time())
```

### 1.2 진입점 추가
ros2 run 명령이 노드를 실행할수 있도록 setup.py에 진입점을 추가해야 한다(src/learning_tf2_py 디렉토리에 있음).

```python
'turtle_tf2_listener = learning_tf2_py.turtle_tf2_listener:main',
```

### 2. 런치파일 수정하기
turtle_tf2_demo.launch.py파일을 열고, launch description에 두개의 새로운 노드를 추가한다. 인자와 import도 추가해 준다. 최종 수정된 파일은 아래와 같을 것이다.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])
```

위 코드에서 traget_frame 런치 인자를 선언하고, spawn될 두 번째 turtle에 대한 broadcaster와 그 transformation을 subscribe할 listener를 시작할 것이다.

### 3. 빌드
빠진 의존성이 있는지 체크하기 위해 워크스페이스의 루트 위치에서 rosdep을 실행해 주자.

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

그리고 업데이트된 패키지를 빌드해 주자.

```bash
colcon build --packages-select learning_tf2_py
```

### 4. 실행
새로운 터미널을 열고, setup 파일을 source 한다.

```bash
. install/setup.bash
```

이제 전체 turtle demo를 실행할 준비가 되었다.

```bash
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py
```

실행하면, 두 개의 turtle이 turtlesim과 함께 보일 것이다. 새로운 터미널 창을 열어 아래 명령어를 실행하자.

```bash
ros2 run turtlesim turtle_teleop_key
```

정상적으로 실행 되었다면, 화살표 키로 turtle을 움직여 보자(시뮬레이션 창이 아닌, turtle_teleop_key 터미널 창이 활성화 되어야 함). 두 번째 turtle이 첫 번째 turtle을 따라가는 것을 볼 수 있을 것이다.

## 요약
이번 튜토리얼에서는 프레임 transformation을 얻기 위해 tf2를 사용하는 법을 학습했다. 또한 Introduting tf2 튜토리얼에서 실행했던 데모를 완성했다.