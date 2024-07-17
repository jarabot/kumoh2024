# [tf2_ros::MessageFilter를 활용한 stamped 자료형 사용하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html#)

1. 목표
1. 배경지식
1. 실습
1. 요약

## 목표
* stamped 자료형을 처리하기 위한 tf2_ros::MessageFilter 사용법 학습하기.

## 배경지식
이번 튜토리얼에서는 tf2로 센서 데이터를 사용하는 방법을 설명한다. 센서 데이터의 실제 예는 카메라(모노, 스테레오), 레이저 스캔 등이 있다.

odometry기능이 없는, turtle3이라는 새로운 turtle을 가정해보자. 또한, 오버헤드가 있는 카메라가 turtle의 위치를 추적하고 있고, 이를 world 프레임 기준으로 PointStamped 메세지로 publish하고 있다고 가정해보자.

turtle1은 자신 기준으로 turtle3가 어디에 위치해 있는지 알고 싶다.

이를 위해, turtle1은 turtle3의 자세를 publish하는 토픽을 듣고, 원하는 프레임에 대한 transform이 준비될때 까지 기다린 다음에서야 자신의 동작을 할 수 있다. tf2_ros::MessageFilter를 사용하면 이런 과정을 더 쉽게 처리할 수 있다. tf2_ros::MessageFilter는 헤더가 있는 어떤 ROS2 메세지든 subscribe해서 타겟 프레임에 대한 transform이 완료될 때 까지 캐쉬에 저장한다.

## 실습
### 1. PointStamped 메세지 broadcaster 노드 작성하기
이번 튜토리얼을 위해 우리는 turtle3의 위치를 PointStamped 메세지로 publish하는 파이썬 노드를 가진 데모 앱을 셋업할 것이다.

먼저 소스 파일을 만들어 보자.

src/learning_tf2_**py**/learning_tf2_**py** 디렉토리에 다음 명령어로 센서 메세지 broadcaster 코드 예제를 다운로드 받자.

```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_message_broadcaster.py
```

### 1.1 코드 분석 
```python
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class PointPublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_message_broadcaster')

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False
        # if the topics of turtle3 can be subscribed
        self.turtle_pose_cansubscribe = False

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                self.turtle_pose_cansubscribe = True
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
                request.name = 'turtle3'
                request.x = 4.0
                request.y = 2.0
                request.theta = 0.0
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info('Service is not ready')

        if self.turtle_pose_cansubscribe:
            self.vel_pub = self.create_publisher(Twist, 'turtle3/cmd_vel', 10)
            self.sub = self.create_subscription(Pose, 'turtle3/pose', self.handle_turtle_pose, 10)
            self.pub = self.create_publisher(PointStamped, 'turtle3/turtle_point_stamped', 10)

    def handle_turtle_pose(self, msg):
        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        vel_msg.angular.z = 1.0
        self.vel_pub.publish(vel_msg)

        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.point.x = msg.x
        ps.point.y = msg.y
        ps.point.z = 0.0
        self.pub.publish(ps)


def main():
    rclpy.init()
    node = PointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
```

이제 코드를 보자. 먼저, on_timer 콜백 함수에서 turtlesim의 Spawn 서비스를 비동기 호출함으로써 turtle3을 스폰한다. 그리고 서비스가 준비되었으면, 위치를 (4, 2, 0)으로 초기화 한다.

```python
# Initialize request with turtle name and coordinates
# Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
request = Spawn.Request()
request.name = 'turtle3'
request.x = 4.0
request.y = 2.0
request.theta = 0.0
# Call request
self.result = self.spawner.call_async(request)
```

그후, 노드는 turtle3/cmd_vel, turtle3/turtle_point_stamped 토픽을 publish 하고, turtlr3/pose를 subscribe하여, handle_turtle_pose 콜백 함수를 실행한다.

```python
self.vel_pub = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)
self.sub = self.create_subscription(Pose, '/turtle3/pose', self.handle_turtle_pose, 10)
self.pub = self.create_publisher(PointStamped, '/turtle3/turtle_point_stamped', 10)
```

마지막으로, handle_turtle_pose 콜백함수에서 turtle3의 Twist 메세지를 초기화 한 후 이를 publish하고, 이는 turtle3가 원을 그리며 움직이게 할 것이다. 그 후, subscribe한 Pose 메세지를 이용해 PointStamped 메세지를 채우고 이를 publish한다.

```python
vel_msg = Twist()
vel_msg.linear.x = 1.0
vel_msg.angular.z = 1.0
self.vel_pub.publish(vel_msg)

ps = PointStamped()
ps.header.stamp = self.get_clock().now().to_msg()
ps.header.frame_id = 'world'
ps.point.x = msg.x
ps.point.y = msg.y
ps.point.z = 0.0
self.pub.publish(ps)
```

### 1.2 런치파일 작성하기
이 데모를 실행하기 위해, launch 디렉토리에 turtle_tf2_sensor_message.launch.py 런치 파일을 생성하고 아래와 같은 내용을 입력하자.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle3'}
            ]
        ),
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_message_broadcaster',
            name='message_broadcaster',
        ),
    ])
```

### 1.3 진입점 추가
ros2 run이 노드를 실행하기 위해, setup.py에 진입점을 추가해야만 한다.

아래 라인을 'console_scripts': 대괄호 사이에 추가하자.

```python
'turtle_tf2_message_broadcaster = learning_tf2_py.turtle_tf2_message_broadcaster:main',
```
### 1.4 빌드

놓친 의존성을 체크하기 위해 워크스페이스의 루트에서 rosdep을 실행한다.

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

그리고 패키지를 빌드한다.

```bash
colcon build --packages-select learning_tf2_**py**
```

### 2. message filter/listener 노드 작성하기
이제 turtle1의 프레임에서 turtle3의 PointStamped 데이터를 신뢰성 있게 얻기 위해, message filter/listener 노드의 소스 코드를 만들 것이다.

src/learning_tf_**cpp**/src/ 디렉토리에 아래의 명령어로 turtlr_tf2_message_filter.**cpp** 파일을 다운로드 받자.

```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_message_filter.cpp
```

### 2.1 코드 분석
```cpp
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#ifdef TF2_CPP_HEADERS
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
  #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

using namespace std::chrono_literals;

class PoseDrawer : public rclcpp::Node
{
public:
  PoseDrawer()
  : Node("turtle_tf2_pose_drawer")
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    point_sub_.subscribe(this, "/turtle3/turtle_point_stamped");
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
      point_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
      this->get_node_clock_interface(), buffer_timeout);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&PoseDrawer::msgCallback, this);
  }

private:
  void msgCallback(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr)
  {
    geometry_msgs::msg::PointStamped point_out;
    try {
      tf2_buffer_->transform(*point_ptr, point_out, target_frame_);
      RCLCPP_INFO(
        this->get_logger(), "Point of turtle3 in frame of turtle1: x:%f y:%f z:%f\n",
        point_out.point.x,
        point_out.point.y,
        point_out.point.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        // Print exception which was caught
        this->get_logger(), "Failure %s\n", ex.what());
    }
  }

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseDrawer>());
  rclcpp::shutdown();
  return 0;
}
```

먼저, 이전에 사용된 tf2, ros2관련 헤더를 include하고, tf2_ros 패키지로부터 tf2_ros::MessageFilter 헤더를 include한다. 

```cpp
#include "geometry_msgs/msg/point_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#ifdef TF2_CPP_HEADERS
  #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
  #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
```

그리고, tf2_ros::Buffer, tf2_ros::TransformListener, tf2_ros::MessageFilter의 영구 인스턴스가 필요하다.

```cpp
std::string target_frame_;
std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_;
```

세 번재로는, ROS2 message_filters::Subscribe가 토픽으로 초기화 되어야 하며, tf2_ros::MessageFilter가 그 Subscriber 오브젝트로 초기화 되어야 한다. MessageFilter 생성자에서 주의해야할 다른 인자는 traget_frame과 콜백 함수이다. 타겟 프레임은 canTransForm이 성공하도록 하는 프레임이다. 콜백 함수는 데이터가 준비되었을 때 호출되는 함수이다.

```cpp
PoseDrawer()
: Node("turtle_tf2_pose_drawer")
{
  // Declare and acquire `target_frame` parameter
  target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

  std::chrono::duration<int> buffer_timeout(1);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  point_sub_.subscribe(this, "/turtle3/turtle_point_stamped");
  tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
    point_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
    this->get_node_clock_interface(), buffer_timeout);
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&PoseDrawer::msgCallback, this);
}
```

마지막으로 콜백 메서드는 데이터가 준비되었을 때 tf2_buffer_->transform을 호출하고, 콘솔에 출력한다.

```cpp
private:
  void msgCallback(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr)
  {
    geometry_msgs::msg::PointStamped point_out;
    try {
      tf2_buffer_->transform(*point_ptr, point_out, target_frame_);
      RCLCPP_INFO(
        this->get_logger(), "Point of turtle3 in frame of turtle1: x:%f y:%f z:%f\n",
        point_out.point.x,
        point_out.point.y,
        point_out.point.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        // Print exception which was caught
        this->get_logger(), "Failure %s\n", ex.what());
    }
  }
```

### 2.2 의존성 추가
learning_tf2_**cpp** 패키지를 빌드하기 전에, package.xml 파일에 아래 두개의 의존성을 추가하자.

```xml
<depend>message_filters</depend>
<depend>tf2_geometry_msgs</depend>
```

### 2.3 CMakeLists.txt
CMakeLists.txt에도 기존 의존성 아래에 다음 두 라인을 추가해 주자.

```cmake
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
```

아래 라인들은 ROS distribution 간에 차이를 처리해 준다.

```cmake
if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)
```

그 후, ros2 run 명령어로 사용할 수 있도록, turtlr_tf2_mesage_filter executable을 추가해 준다.

```cmake
add_executable(turtle_tf2_message_filter src/turtle_tf2_message_filter.cpp)
ament_target_dependencies(
  turtle_tf2_message_filter
  geometry_msgs
  message_filters
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(turtle_tf2_message_filter PUBLIC -DTF2_CPP_HEADERS)
endif()
```

마지막으로 ros2 run이 executable을 찾을 수 있도록, install(TARGETS...) 섹션을 추가해 준다.

```cmake
install(TARGETS
  turtle_tf2_message_filter
  DESTINATION lib/${PROJECT_NAME})
```

### 2.4 빌드
놓친 의존성을 체크하기 위해 워크스페이스의 루트에서 rosdep을 실행한다.

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

그리고 패키지를 빌드한다.

```bash
colcon build --packages-select learning_tf2_cpp
```

### 3. 실행
새로운 터미널을 열고, setup 파일을 source해 준다.

```bash
. install/setup.bash
```

먼저, turtle_tf2_sensor_message.launch.py 파일을 런치함으로써, PointStamed 메세지 broadcaster 노드를 포함해 몇개의 노드들을 실행해 줄 필요가 있다.

```bash
ros2 launch learning_tf2_py turtle_tf2_sensor_message.launch.py
```

실행되면, turtlesim 창이 뜨고, turtle1은 움직이지 않으며, turtle3은 원을 그린다. turtle_teleop_key 노드를 새로운 터미널에서 실행해서 turtle1을 움직여보자.

```bash
ros2 run turtlesim turtle_teleop_key
```

![](https://docs.ros.org/en/humble/_images/turtlesim_messagefilter.png)

이제 turtle3/turtle_point_stamped 토픽을 echo해보면,

```bash
ros2 topic echo /turtle3/turtle_point_stamped
```

아래와 같은 출력이 나올 것이다.

```
header:
  stamp:
    sec: 1629877510
    nanosec: 902607040
  frame_id: world
point:
  x: 4.989276885986328
  y: 3.073937177658081
  z: 0.0
---
header:
  stamp:
    sec: 1629877510
    nanosec: 918389395
  frame_id: world
point:
  x: 4.987966060638428
  y: 3.089883327484131
  z: 0.0
---
header:
  stamp:
    sec: 1629877510
    nanosec: 934186680
  frame_id: world
point:
  x: 4.986400127410889
  y: 3.105806589126587
  z: 0.0
---
```

데모가 실행중인 상태에서, 새로운 터미널을 열고 message filter/listener 노드를 실행해보자. 

```bash
ros2 run learning_tf2_cpp turtle_tf2_message_filter
```

만약 정상적으로 실행되었다면, 아래와 같은 데이터가 나올 것이다.

```
[INFO] [1630016162.006173900] [turtle_tf2_pose_drawer]: Point of turtle3 in frame of turtle1: x:-6.493231 y:-2.961614 z:0.000000

[INFO] [1630016162.006291983] [turtle_tf2_pose_drawer]: Point of turtle3 in frame of turtle1: x:-6.472169 y:-3.004742 z:0.000000

[INFO] [1630016162.006326234] [turtle_tf2_pose_drawer]: Point of turtle3 in frame of turtle1: x:-6.479420 y:-2.990479 z:0.000000

[INFO] [1630016162.006355644] [turtle_tf2_pose_drawer]: Point of turtle3 in frame of turtle1: x:-6.486441 y:-2.976102 z:0.000000
```

## 요약
이번 튜토리얼에서는 tf2에서 센서 데이터/메세지를 사용하는 법을 학습했다. 특히, PointStamped 메세지를 publish하고, subscribe한 메세지를 tf2ros::MessageFilter로 PointStamped 프레임을 transform하는 방법을 학습했다. 
