# [프레임 추가하기(Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Adding-A-Frame-Py.html)
1. 목표
1. 배경지식
1. tf2 tree
1. 실행
1. 요약

## 목표
* tf2에 프레임 추가하는 방법 학습하기.

## 배경지식
이전 튜토리얼에서는, tf2 broadcaster와 tf2 listener를 작성하여 turtle 데모를 재현해 보았다. 이번 튜토리얼에서는 transformation 트리에 fixed 프레임과 dynamic 프레임을 추가하는 방법을 배울 것이다. 사실, tf2에 프레임을 추가하는 것은 tf2 broadcaster를 추가하는 것과 매우 유사하지만, 이 예제는 tf2의 추가적인 기능을 보여줄 것이다.

많은 transformation 관련 작업에서는, local frame안에서 생각하는 것이 더 쉽다. 예를 들어, 레이저 스캐너 중앙에 있는 프레임에서 레이저 스캔의 측정값을 추론하는 것이 가장 쉬운 방법이다. tf2는 시스템의 각 센서, 링크, 조인트들에 로컬 프레임을 정의하게 해준다. tf2는 하나의 프레임에서 다른 프레임으로 tranform할 때, 모든 중간 과정의 프레임 transformation을 처리해 줄 것이다.

## tf2 tree
tf2는 프레임들의 트리 구조를 구성해 주고, 폐루프를 허용하지 않는다. 이는 각 프레임이 하나의 부모만을 가질 수 있고, 여러 명을 자식을 가질 수 있다는 것을 의미한다. 현재 우리가 만든 트리는 3개의 프레임(world, turtle1, turtle2)을 포함하고 있다. 두 turtle 프레임은 world 프레임의 자식 프레임이다. 만약 우리가 새로운 프레임을 추가하고 싶다면, 기존의 3개 프레임 중 하나가 부모 프레임이 되어야 하고, 새로운 프레임은 자식 프레임이 되어야 한다.

![](https://docs.ros.org/en/humble/_images/turtlesim_frames.png)

## 실행
## 1. fixed frame broadcaster 작성하기
우리는 우리가 작성한 turtle 예제에서, turtle1의 자식 프레임이 될 새로운 프레임 carrot1을 추가할 것이다. 이 프레임은 두 번째 turtle의 goal 역할을 맡게 된다.

먼저 소스파일을 생성해보자. 이전 튜토리얼에서 작성한 learning_tf2_py 패키지로 가서, 아래의 명령어로 fixed 프레임 broadcaster 코드를 다운로드 받자.

```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/fixed_frame_tf2_broadcaster.py
```

### 1.1 코드 분석
```python
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
       self.tf_broadcaster = TransformBroadcaster(self)
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'turtle1'
       t.child_frame_id = 'carrot1'
       t.transform.translation.x = 0.0
       t.transform.translation.y = 2.0
       t.transform.translation.z = 0.0
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
```

이 코드는 tf2 broadcaster 튜토리얼 예제와 매우 유사하고, transform이 변하지 않는다는 것이 차이점이다.

이제 핵심 라인들을 살펴보자. 아래에서 우리는 turtle1을 부모로하고 carrot1을 자식으로 하는 새로운 transform을 만든다. carrot1 프레임은 turtle1 프레임에 비해 y축방향으로 2 [m] 오프셋을 가진다.

```bash
t = TransformStamped()

t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = 'turtle1'
t.child_frame_id = 'carrot1'
t.transform.translation.x = 0.0
t.transform.translation.y = 2.0
t.transform.translation.z = 0.0
```

### 1.2 진입첨 추가
ros2 run 명령어로 노드를 실행하기 위해 setup.py에 진입점을 추가해야 한다(src/learning_tf2_py 디렉토리에 있음).

마지막으로 다음 라인을 'console_scripts': 대괄호 사이에 추가하자.

```python
'fixed_frame_tf2_broadcaster = learning_tf2_py.fixed_frame_tf2_broadcaster:main',
```

### 1.3 런치파일 작성하기
이제 런치파일을 작성해 보자. turtle_tf2_fixed_frame_demo.launch.py라는 이름으로 새로운 파일을 생성하고 다음 라인들을 추가해 주자.

```python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
        )

    return LaunchDescription([
        demo_nodes,
        Node(
            package='learning_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
        ),
    ])
```

이 런치파일은 필요한 패키지들을 import한 다음, 이전 튜토리얼의 launch 파일에서 생성했던 노드들을 저장할 demo_nodes 변수를 생성한다.

코드의 마지막 부분은 fixed_frame_tf2_broadcaster를 사용해서 turtlesim 월드에 fixed carrot1 프레임을 추가할 것이다.

```python
Node(
    package='learning_tf2_py',
    executable='fixed_frame_tf2_broadcaster',
    name='fixed_broadcaster',
),
```
### 1.4 빌드

놓친 의존성을 체크하기 위해, 워크스페이스의 루트에서 rosdep을 실행하자.


```bash
rosdep install -i --from-path src --rosdistro humble -y
```

그리고, 업데이트 된 패키지를 빌드한다.

```bash
colcon build --packages-select learning_tf2_py
```

### 1.5 실행
새로운 터미널을 열고 워크스페이스를 source 한다.

```bash
. install/setup.bash
```

이제 turtle broadcaster 데모를 실행할 수 있다:

```bash
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py
```

실행하면, 새로운 carrot1 프레임이 transformation 트리에 추가 된 것을 확인해보자.

![](https://docs.ros.org/en/humble/_images/turtlesim_frames_carrot.png)

새로운 프레임을 추가했지만 첫 번째 turtle을 조종해도 이전 튜토리얼가 변한 것이 없다는 것을 확인할 수 있다. 이는 프레임을 추가한 것이 다른 프레임에 영향을 미치지 않기 때문이며, listener 또한 여전히 이전에 정의된 프레임을 사용하고 있기 때문이다.

그러므로, 만약 두 번째 turtle이 첫 번째 turtle 이 아닌 carrot을 따라가길 원한다면, target_frame값을 바꿔 줄 필요가 있다. 이것은 두 가지 방식으로 가능하다. 첫 번재는 console에서 런치파일에 직접 target_frame 인자를 넘겨주는 것이다.

```bash
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py target_frame:=carrot1
```

두 번째 방식은 런치파일을 수정하는 것이다. turtle_tf2_fixed_frame_demo.launch.py 파일을 열고 launch_arguments 인자를 통해 'target_frame': 'carrot1' 파라미터를 추가하자.

```python
def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        ...,
        launch_arguments={'target_frame': 'carrot1'}.items(),
        )
```

이제 패키지를 다시 빌드하고, turtle_tf2_fixed_frame_demo.launch.py를 재실행 해보자. 두 번째 turtle이 첫 번째 turtle 이 아니라 carrot을 따라가는 것을 볼 수 있을 것이다.

![](https://docs.ros.org/en/humble/_images/carrot_static.png)

### 2 dynamic 프레임 broadcaster 작성하기

이번 튜토리얼에서 publish한 추가 프레임은 부모프레임과의 관계가 변하지 않는 fixed 프레임이다. 그러나 만약 움직이는 frame을 publish하고 싶다면, 시간에 따라 변하는 프레임을 broadacaster를 작성할 수도 있다. 이제  turtle1에 대해 시간에 따라 변하는 carrot1 프레임을 작성해 보자. 이를 위해 아래 명령어로 dynamic frame broadcaster 코드를 다운로드 하자.

```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/dynamic_frame_tf2_broadcaster.py
```

### 2.1 코드 분석
```python
import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        seconds, _ = self.get_clock().now().seconds_nanoseconds()
        x = seconds * math.pi

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        t.transform.translation.x = 10 * math.sin(x)
        t.transform.translation.y = 10 * math.cos(x)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
```

우리는 고정된 x, y 오프셋 대신, carrot1이 계속해서 변하도록 시간에 관한 sin(), cos() 함수를 사용할 것이다.

```python
seconds, _ = self.get_clock().now().seconds_nanoseconds()
x = seconds * math.pi
...
t.transform.translation.x = 10 * math.sin(x)
t.transform.translation.y = 10 * math.cos(x)
```

### 2.2 진입점 추가
ros2 run 명령어로 노드를 실행하기 위해 setup.py에 진입점을 추가해야 한다(src/learning_tf2_py 디렉토리에 있음).

마지막으로 다음 라인을 'console_scripts': 대괄호 사이에 추가하자.

```python
'dynamic_frame_tf2_broadcaster = learning_tf2_py.dynamic_frame_tf2_broadcaster:main',
```

### 2.3 런치파일 작성
이 코드를 테스트하지 위해, 새로운 런치 파일 turtle_tf2_dynamic_frame_demo.launch.py를 생성하고 아래 코드를 집어 넣자.

```python
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
       launch_arguments={'target_frame': 'carrot1'}.items(),
       )

    return LaunchDescription([
        demo_nodes,
        Node(
            package='learning_tf2_py',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
        ),
    ])
```

### 2.4 빌드
놓친 의존성을 체크하기 위해, 워크스페이스의 루트에서 rosdep을 실행하자.

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

그리고, 업데이트된 패키지를 빌드하자.

```bash
colcon build --packages-select learning_tf2_py
```

### 2.5 실행
새로운 터미널을 열고, setup 파일을 source 하자.

```bash
. install/setup.bash
```

이제 dynamic 프레임 데모를 실행할 수 있다.

```bash
ros2 launch learning_tf2_py turtle_tf2_dynamic_frame_demo.launch.py
```

두 번째 turtle이 지속적으로 변하는 carrot의 위치를 따라가는 것을 확인할 수 있다.

![](https://docs.ros.org/en/humble/_images/carrot_dynamic.png)

## 요약
이번 튜토리얼에서는 tf2 transformation tree와 그 구조 및 기능에 대해 학습했다. 그리고 local 프레임에서 생각하는 것이 가장 쉽다는 것과, local 프레임에 대한 fixed 및 dynamic 프레임을 추가하는 방법을 배웠다.
