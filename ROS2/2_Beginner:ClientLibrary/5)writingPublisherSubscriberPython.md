# 간단한 publisher와 subscriber 작성하기 (Python)
1. 소개
2. 실습
   1. package 생성하기
   2. publisher node 작성하기
   3. subscriber node 작성하기
   4. 빌드 및 실행
## 1. 소개
* Python으로  publisher와 subscriber node를 생성하고 실행하기

* node는
  * ROS graph 상에서 서로 통신하는 실행가능한 process
  * topic 상으로 message를 전송/수신하는 역할
    * talker
    * listener

## 2. 실습
### 2-1 package 생성
* ros2_ws/src 디렉토리 내에 새로운 package를 생성
* py_pubsub package를 생성하는 명령실행
```bash
ros2 pkg create --build-type ament_python py_pubsub
```

### 2-2 publisher node 작성하기
* 아래 명령으로 talker 코드 다운받기
```bash
cd ~/ros2_ws/src/py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

* Visual Studio Code로 다운받은 publisher_member_function.py 파일 열어보기 (같은 폴더에 __init__.py 있음)
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2-2-1 의존성(dependencies) 추가
* ros2_ws/src/py_pubsub 디렉토리로 아래 파일들
  * setup.py
  * setup.cfg
  * package.xml

* package.xml 파일 열기(Visual Studio Code)
```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
  * description, maintainer, license 채우기

* 위 코드 바로 밑에 아래 코드 복사해서 붙여넣기
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### 2-2-2 entry point 추가하기
* setup.py 파일 열고 수정하기(package.xml 파일과 동일하게 작성)
```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
* entry_points 필드 부분에 talker 추가하기(추가 후 저장하기)
```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
### 2-2-3 setup.cfg 확인하기
* setup.cfg
```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```
* setuptool이 실행될때 lib 내에 실행자를 넣으라고 지시
  * 결국 ros2 run 실행시 path를 제대로 찾게 해주는 역할

* setup.bash을 source하면 실행자의 path를 찾을 수 있게 된다.

### 2-3 subscriber node 작성하기
* 새 node를 생성하기 위해서 ros2_ws/src/py_pubsub/py_pubsub 로 이동하고 아래 명령 실행
```bash
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
* 명령 실행 후 디렉토리 내부 파일 (subscriber_member_function.py 추가된 것을 확인 목적)
```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

### 2-3-1 subscriber_member_function.py 파일
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2-3-2 entry point 추가하기
* setup.py 파일 다시 열고 listener 추가하기
```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

### 2-4 빌드 및 실행하기
* 먼저 의존성 체크하기
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

* 새 package 빌드하기
```bash
colcon build --packages-select py_pubsub
```

* source 하기
```bash
source install/setup.bash
```

* talker node 실행하기
```bash
ros2 run py_pubsub talker
```

* 결과
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
...
```

* listener
```bash
ros2 run py_pubsub listener
```
* 결과
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```


# Quiz 
정수를 1씩 증가시켜 Publisher에서 Subscriber로 전송하는 pub-sub 작성하세요 
