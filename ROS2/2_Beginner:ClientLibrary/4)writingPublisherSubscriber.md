# 간단한 publisher와 subscriber 작성하기 (C++)
1. 소개
2. 실습
   1. package 생성하기
   2. publisher node 작성하기
   3. subscriber node 작성하기
   4. 빌드 및 실행

## 1. 소개
* C++로  publisher와 subscriber node를 생성하고 실행하기

* node는
  * ROS graph 상에서 서로 통신하는 실행가능한 process
  * topic 상으로 message를 전송/수신하는 역할
    * talker
    * listener

## 2. 실습
### 2-1 package 생성
* ros2_ws/src 디렉토리 내에 새로운 package를 생성하기

* cpp_pubsub package를 생성하는 명령 실행
```bash
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

### 2-2 publisher node 작성하기
* 아래 명령 실행하여 talker 코드 다운받기 (ros2_ws/src/cpp_pubsub/src 아래)
```bash
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp
```

* Visual Studio Code로 publisher_member_function.cpp 파일 열기
```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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

### 2-2-1 의존성(dependencies) 추가
* ros2_ws/src/cpp_pubsub 디렉토리 아래 CMakeLists.txt와 package.xml 파일이 존재

* package.xml 파일 열기
```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

* ament_cmake buildtool dependency 뒤에 아래 코드 추가
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

### 2-2-2 CMakeLists.txt
* CMakeLists.txt 파일 열고 수정하기(find_package(ament_cmake REQUIRED) 바로 뒤에 아래 코드 추가)
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

* 이어서 추가하기
```cmake
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

* 마지막으로 install(TARGETS...) 섹션 추가하기
```cmake
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

* 최종 CMakeLists.txt 파일
```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### 2-3 subscriber node 작성하기
* 다음 node를 생성하기 위해서 ros2_ws/src/cpp_pubsub/src로 이동하여 아래 명령 실행하기
```bash
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp
```

* 파일 확인하기 위해 ls명령 실행
```bash
ls
```
* 결과
```
publisher_member_function.cpp  subscriber_member_function.cpp
```

* Visual Studio Code로 subscriber_member_function.cpp 파일 열기
```c++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### 2-3-1 CMakeLists.txt
* CMakeLists.txt 파일 열어서 수정하기(publish entries 부분 아래에 추가)
```cmake
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

### 2-4 빌드 및 실행
* ~/ros2_ws 로 이동하여 rosdep 실행하기 (의존성 설치)
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

* 특정 package만 빌드하기
```bash
colcon build --packages-select cpp_pubsub
```

* 새 터미널 열어서 아래 명령 실행(overlay source)
```bash
. install/setup.bash
```

* talker node 실행하는 명령 실행
```bash
ros2 run cpp_pubsub talker
```

* 결과
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

* 새 터미널 열어서 listener 실행하는 명령 실행
```bash
ros2 run cpp_pubsub listener
```

* 결과
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

* 종료 방법
  * Ctrl + C

# Quiz

### Publisher에서 발행한 거리 값을 Subscriber에서 수신하도록 하는 pub-sub을 C++ 코드를 작성하십시오.

* 참고 사항:

* Publisher 노드의 이름은 "distance_publisher"로, 토픽 이름은 "distance"로 설정합니다.
* Subscriber 노드의 이름은 "distance_subscriber"로, 토픽 * 이름은 "distance"로 설정합니다.
* 메시지 타입은 std_msgs/msg/Float64를 사용합니다.
* Publisher는 1초마다 거리 값을 발행하며, Subscriber는 이 값을 수신하여 화면에 출력합니다.

아래의 코드를 수정하세요 
distance_publisher.cpp
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class DistancePublisher : public rclcpp::Node
{
public:
    DistancePublisher()
        : Node("distance_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("distance", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DistancePublisher::publish_distance, this));
    }

private:
    void publish_distance()
    {
        auto message = std_msgs::msg::Float64();
        message.data = 3.14;
        RCLCPP_INFO(this->get_logger(), "Publishing Distance: %.2f", message.data);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistancePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

distance_subscriber.cpp
```cpp

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

void distance_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("distance_subscriber"), "Received Distance: %.2f", msg->data);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("distance_subscriber");
    auto subscription = node->create_subscription<std_msgs::msg::Float64>(
        "distance", 10, distance_callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

```

setup.py 수정 

setup.py
```python
entry_points={
        'console_scripts': [
                'number_publisher = my_publisher_subscriber.publisher_subscriber_node:main',
                'number_subscriber = my_publisher_subscriber.publisher_subscriber_node:main',
        ],
```


Cmake수정
```bash
add_executable(distance_pub src/distance_publisher.cpp)
ament_target_dependencies(distance_pub rclcpp std_msgs)

add_executable(distance_sub src/distance_subscriber.cpp)
ament_target_dependencies(distance_sub rclcpp std_msgs)

install(TARGETS
  talker
  listener
  distance_pub
  distance_sub
  DESTINATION lib/${PROJECT_NAME})
```
