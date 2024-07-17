# 커스텀 msg와 srv 파일 생성하기
1. 소개
2. 실습
   1. 새 package 생성하기
   2. custom 정의 생성
   3. CMakeLists.txt
   4. package.xml
   5. tutorial_interfaces package 빌드하기
   6. msg와 srv 생성 확인
   7. 새 interface 테스트하기

## 1. 소개
* 사용자가 원하는 .msg와 .svr 파일 정의하여 사용하는 방법
* tutorial_interfaces package 내부에 msg와 srv 넣기
## 2. 실습
### 2-1 새 package 생성하기
* 생성한 .msg와 .srv 파일을 사용하는 package 생성

* 새로운 package 생성
```bash
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```

* .msg와 .srv 파일은 각각 msg와 srv 디렉토리 내에 위치한다.
* ros2_ws/src/tutorial_interfaces 내에 아래 명령 수행
```bash
mkdir msg srv
```

### 2-2 커스텀 정의 생성하기

### 2-2-1 msg 정의
* tutorial_interfaces/msg/Num.msg 파일 생성하기
* Num.msg 파일 내부
```
int64 num
```

* tutorial_interfaces/msg/Sphere.msg 파일 생성하기
* Sphere.msg 파일 내부
```
geometry_msgs/Point center
float64 radius
```
  * 다른 message package(geometry_msgs/Point)에 있는 message를 사용 

### 2-2-2 srv 정의
* tutorial_interfaces/srv/AddThreeInts.srv 파일 생성하기
* AddThreeInts.srv 파일 내부
```
int64 a
int64 b
int64 c
---
int64 sum
```

### 2-3 CMakeLists.txt 
* CMakeLists.txt 파일 수정하기
```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
```

### 2-4 package.xml 수정하기
* package.xml 내에 <package> element 내부에 아래 코드를 추가
```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### 2-5 tutorial_interfaces package 빌드하기
```bash
colcon build --packages-select tutorial_interfaces
```

### 2-6 msg와 srv 생성 확인
* source 실행
```bash
source install/setup.bash
```

* ros2 interface show 명령으로 확인하기
```bash
ros2 interface show tutorial_interfaces/msg/Num
```

* 결과
```
int64 num
```

* ros2 interface show 명령으로 확인하기
```bash
ros2 interface show tutorial_interfaces/msg/Sphere
```

* 결과
```
geometry_msgs/Point center
        float64 x
        float64 y
        float64 z
float64 radius
```

* ros2 interface show 명령으로 확인하기
```bash
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

* 결과
```
int64 a
int64 b
int64 c
---
int64 sum
```

### 2-7 새로운 interface 테스트
* CMakeLists.txt와 package.xml 파일 수정

### 2-7-1 Num.msg 테스팅 (pub/sub)
* 이전에 작성한 publisher/subscriber package를 수정 
* Publisher
```c++
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"     // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);    // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Num();                               // CHANGE
    message.num = this->count_++;                                        // CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;         // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

* Subscriber
```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"     // CHANGE
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(          // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);              // CHANGE
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

* CMakeLists.txt (아래 코드 추가)
```cmake
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

* package.xml (아래 코드 추가)
```xml
<depend>tutorial_interfaces</depend>
```

* package 빌드하기
```bash
colcon build --packages-select cpp_pubsub
```
* 새 터미널 열고 source 후 실행 (talker 실행)
```bash
ros2 run cpp_pubsub talker
```
* 새 터미널 열고 source 후 실행 (listener 실행)
```bash
ros2 run cpp_pubsub listener
```

* 결과
```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

### service는 Homework!
