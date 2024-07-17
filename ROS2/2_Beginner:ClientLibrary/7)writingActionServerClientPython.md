# service와 client 작성하기 (Python)
1. 개요
2. 실습
   1. package 생성하기
   2. service node 작성하기
   3. client node 작성하기
   4. 빌드 및 실행
## 1. 개요
* Python으로 service와 client nodes를 생성하고 빌드하기
* service를 사용해서 nodes간 통신 구현 방법 이해
* client node
  * request를 보내는 node
* server node
  * request를 수신하고 나서 이에 대한 response를 보내는 node
* request와 response는 .srv 파일로 정의

## 2. 실습
### 2-1 package 생성하기
* py_srvcli package 만들기
```bash
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

* exampel_interfaces package 내에 .srv 파일 (service에서 주고 받는 구조)
```
int64 a
int64 b
---
int64 sum
```

### 2-1-1 package.xml 업데이트하기
* package.xml
```xml
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

### 2-1-2 setup.py 업데이트
* setup.py
```python
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```

### 2-2 service node 작성하기
* ros2_ws/src/py_srvcli/py_srvcli/service_member_function.py 파일 만들기
* service_member_function.py 파일 열어서 아래 코드 붙여넣기
```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
### 2-2-1 entry point 추가하기
* ros2 run 명령으로 service 를 실행시키기 위해서 entry point 설정 필요 (setup.py 파일)
 
* ros2_ws/src/py_srvcli/setup.py 파일 열기
* 'console_scripts': 부분에 아래 추가하기
```
'service = py_srvcli.service_member_function:main',
```

### 2-3 client node 작성하기
* ros2_ws/src/py_srvcli/py_srvcli/client_member_function.py 파일 추가하기
```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2-3-1 entry point 추가하기
* setup.py 파일 내부에 entry_points 필드에 client 부분 추가하기
```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

### 2-4 빌드 및 실행
* 의존성 검사하기
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

* py_srvcli package 빌드하기
```bash
cd ~/ros2_ws
colcon build --packages-select py_srvcli
```

* service node 실행하기 - 새 터미널 열고 아래 명령 실행
```bash
source install/setup.bash
ros2 run py_srvcli service
```

* client node 실행하기 - 새 터미널 열고 아래 명령 실행
```bash
source install/setup.bash
ros2 run py_srvcli client 2 3
```

* client 실행 터미널 결과
```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```

* server 실행 터미널 결과
```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```

* 실행 종료하기
  * Ctrl + C
