# [broadcaster 작성하기(C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)
1. 목표
1. 배경지식
1. 사전 준비
1. 실행
1. 요약

## 목표
* 로봇의 pose(위치/방향) 정보를 Transform으로 broadcast하기

## 사전 준비
* ROS2 동작 원리.
* Introducing tf2, tf2 static broadcaster 튜토리얼 완료.
* 워크스페이스, 패키지 생성 방법.
* learning_tf2_cpp패키지 필요

## 실습
### 1. broadcaster 노드 작성하기

* 새 터미널 실행 후, learning_tf2_cpp/src 로 이동
```bash
   cd ~/ros_ws/src/learning_tf2_cpp/src
```
* broadcaster 코드 예제 다운로드 
```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_broadcaster.cpp
```

### 1.1 코드 분석
```cpp
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
  }

private:
  void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_.c_str();

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
```

* Q. turtlesim node의 위치/방향을 어떻게 추적할 수 있을까?

* turtlesim node의 pose 정보를 구독하기 위해, turtlename 파라미터 설정

```cpp
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");
```
* turtlesim node의 pose Topic을 구독
* 이후, pose Topic을 수신하면 handle_turtle_pose() 콜백 함수가 실행된다

```cpp
  subscription_ = this->create_subscription<turtlesim::msg::Pose>(
  topic_name, 10,
  std::bind(&FramePublisher::handle_turtle_pose, this, _1));
```

* 콜백함수에서는 먼저, TransformStamped 오브젝트를 생성하고, 다음과 같은 메타데이터를 채운다.

1. 타임스탬프 : 현재 시간을 적용한다.
1. 부모 프레임의 이름 : world
1. 자식 프레임의 이름 : turtleX

```cpp
geometry_msgs::msg::TransformStamped t;

// Read message content and assign it to
// corresponding tf variables
t.header.stamp = this->get_clock()->now();
t.header.frame_id = "world";
t.child_frame_id = turtlename_.c_str();
```

* 수신된 pose 정보로부터, transform의 각 변수에 값을 할당한다

```cpp
// Turtle only exists in 2D, thus we get x and y translation
// coordinates from the message and set the z coordinate to 0
t.transform.translation.x = msg->x;
t.transform.translation.y = msg->y;
t.transform.translation.z = 0.0;

// For the same reason, turtle can only rotate around one axis
// and this why we set rotation in x and y to 0 and obtain
// rotation in z axis from the message
tf2::Quaternion q;
q.setRPY(0, 0, msg->theta);
t.transform.rotation.x = q.x();
t.transform.rotation.y = q.y();
t.transform.rotation.z = q.z();
t.transform.rotation.w = q.w();
```

TransformBroadcaster::sendTransfrom() 메서드 호출

```cpp
// Send the transformation
tf_broadcaster_->sendTransform(t);
```

### 1.2 CMakeLists.txt
* 코드 편집기로 learning_tf2_cpp의 CMakeLists.txt 열기
* 실행자 추가 - turtle_tf2_broadcaster

```cmake
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
    turtle_tf2_broadcaster
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    turtlesim
)
```

* install(TARGETS...) 추가

```cmake
install(TARGETS
    turtle_tf2_broadcaster
    DESTINATION lib/${PROJECT_NAME})
```
### 2. 런치파일 작성하기
* 새 터미널 실행 후, 프로젝트 디렉토리로 이동
```bash
    cd ~/ros_ws/src/learning_tf2_cpp
```
* launch 디렉토리 생성
```bash
    mkdir launch
```

* launch 파일 생성
```bash
    touch turtle_tf2_demo.launch.py
```

* 코드 편집기로 turtle_tf2_demo.launch.py 열기
* 아래 코드 복사
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
    ])
```

### 2.1 코드 설명
* launch, launch_ros 패키지로부터 필요한 모듈을 import
  * launch 패키지: generic launching 프레임워크(ROS2와 별개)
  * launch_ros 패키지: node와 같은 ROS2에 관련 내용 포함

```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

* turtlesim & turtle_tf2_broadcaster 실행

```python
Node(
    package='turtlesim',
    executable='turtlesim_node',
    name='sim'
),
Node(
    package='learning_tf2_cpp',
    executable='turtle_tf2_broadcaster',
    name='broadcaster1',
    parameters=[
        {'turtlename': 'turtle1'}
    ]
),
```

### 2.2 Add dependencies
* 코드 편집기로 learning_tf2_cpp 디렉토리 내 package.xml 열기
* launch의 import문에 해당하는 의존성 추가

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

### 2.3 CMakeLists.txt
* CMakeLists.txt 에 다음 내용 추가 

```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
```

### 3. 빌드
* 빠진 의존성이 있는지 체크하기 위해 워크스페이스의 루트 위치에서 rosdep을 실행해 주자.

```bash
rosdep install -i --from-path src --rosdistro humble -y
```
learning_tf2_cpp package 빌드 


```bash
colcon build --packages-select learning_tf2_cpp
```

### 4. 실행
새로운 터미널을 열고, setup 파일을 source 한다.

```bash
. install/setup.bash
```

이제 turtlsim 시뮬레이션 노드와 turtle_tf2_broadcaster노드를 시작할 런치 파일을 실행 한다.

```bash
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
```

새로운 터미널을 열고, 다음 명령어를 실행한다. 

```bash
ros2 run turtlesim turtle_teleop_key
```

이제 조종할 수 있는 turtle 하나와 함께 시작된 turtlesim 시뮬레이션을 확인 할 수 있다.

![](https://docs.ros.org/en/humble/_images/turtlesim_broadcast.png)

이제 tf2_echo 툴을 이용해 turtle 자세가 실제로 tf2로 broadcast되고 있는지 확인해 보자.

```bash
ros2 run tf2_ros tf2_echo world turtle1
```

첫 번째 turtle의 자세가 나타날 것이다. 화살표 키를 이용해 turtle을 움직여 보자(시뮬레이션 창이 아닌, turtle_teleop_key 터미널 창이 활성화 되어야 함). 콘솔 출력은 아래와 비슷할 것이다.

```
At time 1670400305.709085308
- Translation: [8.435, 5.871, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.112, 0.994]
- Rotation: in RPY (radian) [0.000, -0.000, 0.224]
- Rotation: in RPY (degree) [0.000, -0.000, 12.834]
- Matrix:
  0.975 -0.222  0.000  8.435
  0.222  0.975  0.000  5.871
  0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000
At time 1670400306.701062321
- Translation: [8.435, 5.871, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.112, 0.994]
- Rotation: in RPY (radian) [0.000, -0.000, 0.224]
- Rotation: in RPY (degree) [0.000, -0.000, 12.834]
- Matrix:
  0.975 -0.222  0.000  8.435
  0.222  0.975  0.000  5.871
  0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000
```

만약 world와 turtle2에 대한 transform을 tf2_echo로 실행해 보면, 아무것도 보이지 않을 것이다. 왜냐하면 두 번째 turtle은 아직 없기 때문이다. 하지만 다음 튜토리얼에서 두 번째 turtle을 추가하면 turtle2의 자세도 broadcast될 것이다.

## 요약
이번 튜토리얼에서는 로봇의 자세(turtle의 위치와 방향)를 broadcast하는 방법을 학습하고, tf2_echo를 사용하는 법을 배웠다. 실제로 tf2에 broadcast된 transform을 사용하기 위해서는, 다음 튜토리얼에서 tf2 listener를 다뤄야 한다.
