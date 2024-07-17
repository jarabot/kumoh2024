# [Listener 작성하기(C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)
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
* learning_tf2_cpp패키지 필요

## 실습
### 1. Listener 노드 작성하기
소스 파일을 만들어 보자. learning_tf2_cpp/src 위치에 아래의 명령어로 listener 코드 예제를 다운로드 받자.

```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp
```

### 1.1 코드 분석
```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ =
      this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";

    if (turtle_spawning_service_ready_) {
      if (turtle_spawned_) {
        geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        geometry_msgs::msg::Twist msg;

        static const double scaleRotationRate = 1.0;
        msg.angular.z = scaleRotationRate * atan2(
          t.transform.translation.y,
          t.transform.translation.x);

        static const double scaleForwardSpeed = 0.5;
        msg.linear.x = scaleForwardSpeed * sqrt(
          pow(t.transform.translation.x, 2) +
          pow(t.transform.translation.y, 2));

        publisher_->publish(msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully spawned");
        turtle_spawned_ = true;
      }
    } else {
      // Check if the service is ready
      if (spawner_->service_is_ready()) {
        // Initialize request with turtle name and coordinates
        // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 4.0;
        request->y = 2.0;
        request->theta = 0.0;
        request->name = "turtle2";

        // Call request
        using ServiceResponseFuture =
          rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (strcmp(result->name.c_str(), "turtle2") == 0) {
              turtle_spawning_service_ready_ = true;
            } else {
              RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
            }
          };
        auto result = spawner_->async_send_request(request, response_received_callback);
      } else {
        RCLCPP_INFO(this->get_logger(), "Service is not ready");
      }
    }
  }

  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  bool turtle_spawned_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
```

turtle을 spawn하는 서비스의 동작을 이해하기 위해, writing a simple service and client(C++) 튜토리얼을 참고하자.
이제 프레임 transformation에 접근하는 코드를 보자. tf2_ros는 transform의 수신을 더 쉽게 만들어 주는 TransformListener 헤더 파일 구현을 포함하고 있다.

```cpp
#include "tf2_ros/transform_listener.h"
```

아래에서는, TransformListener를 생성한다. Listener가 생성되면, tf2 transformationㅇ르 수신하기 시작하고, 10초 동안 버퍼에 유지시킨다.

```cpp
tf_listener_ =
  std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

마지막으로, 특정 transformation에 대해 listener를 query한다. 그러기 위해, 아래의 인자들로 lookup_transform 메소드를 호출한다.

1. Target frame
1. Source frame
1. transform한 시점

tf2::TimePointZero()를 사용하면 최신 transform을 얻을 수 있다. 또한 exception을 처리하기 위해 try-catch문으로 랩핑된다.

```cpp
t = tf_buffer_->lookupTransform(
  toFrameRel, fromFrameRel,
  tf2::TimePointZero);
```

### 1.2 CMakeLists.txt
learning_tf2_cpp 디렉토리로 이동한 후, CMakeLists.txt에 ros2 run으로 사용하게 될 turtle_tf2_Listener라는 이름의 executable을 추가한다.

```cmake
add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
ament_target_dependencies(
    turtle_tf2_listener
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    turtlesim
)
```

마지막으로, ros2 run이 executable을 찾을 수 있도록 install(TARGETS...)을 추가한다.

```cmake
install(TARGETS
    turtle_tf2_listener
    DESTINATION lib/${PROJECT_NAME})
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
            package='learning_tf2_cpp',
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
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf2_cpp',
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
colcon build --packages-select learning_tf2_cpp
```

### 4. 실행
새로운 터미널을 열고, setup 파일을 source 한다.

```bash
. install/setup.bash
```

이제 전체 turtle demo를 실행할 준비가 되었다.

```bash
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
```

실행하면, 두 개의 turtle이 turtlesim과 함께 보일 것이다. 새로운 터미널 창을 열어 아래 명령어를 실행하자.

```bash
ros2 run turtlesim turtle_teleop_key
```

정상적으로 실행 되었다면, 화살표 키로 turtle을 움직여 보자(시뮬레이션 창이 아닌, turtle_teleop_key 터미널 창이 활성화 되어야 함). 두 번째 turtle이 첫 번째 turtle을 따라가는 것을 볼 수 있을 것이다.

## 요약
이번 튜토리얼에서는 프레임 transformation을 얻기 위해 tf2를 사용하는 법을 학습했다. 또한 Introduting tf2 튜토리얼에서 실행했던 데모를 완성했다.