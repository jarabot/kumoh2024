# C++ for ROS2
* [google doc](https://docs.google.com/presentation/d/1cuTsp51UCc-lbhLXpOOKxJ3TDfk4fxK7852rgH3iyVI/edit#slide=id.p)

## include
  * 의존성
## [rclcpp](https://docs.ros2.org/foxy/api/rclcpp/index.html)란?
* ROS client Library for C++
* ROS system과 상호 동작하는 프로그램 작성을 위한 C++ API 제공
* 여러 components로 구성
  * Node
  * Publisher
  * Subscription
  * Service Client
  * Service Server
  * Timer
  * Parameters
  * Rate
  * Executors
  * CallbackGrouops
  * ....
* [std::chrono_literals](https://learn.microsoft.com/ko-kr/cpp/standard-library/chrono-literals?view=msvc-170)

## 상속 : class MinimalPublisher : public rclcpp::Node
* [상속 예제](./src/inheritance.cpp)

## constructor(생성자), 생성자에서 멤버 변수 초기화
* 생성자에서 하는 일
* 멤버 변수 초기화
* [생성자 예제](./src/constructor.cpp)

## this 키워드
* class에서 자기 자신
* 자기 자신
* [this 예제](./src/this.cpp)
* [링크](https://codemasterkimc.tistory.com/14)

## auto
* 변수에 타입을 컴파일러가 알아서 유추해서 넣어줌
```c++
auto a = 10;
int a = 10;
```
## template
* [template 예제](./src/template.cpp)


## std::bind [링크](https://yhwanp.github.io/2019/09/15/std-function-and-std-bind/)
* 함수를 변수처럼 사용하거나 
* 함수의 특정 파라미터를 고정시킨 새로운 함수를 생성할 수도 있게 해준다.
* 함수의 주소(pointer)를 C++에서 사용하는 객체로 변환
* 객체로 변환하여 type이나 변수처럼 인자로 전달가능!
* #include <functional> import 하기
* 함수 pointer가 안되는 이유는 state를 사용하므로!!!


```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp" //rclcpp ROS client library C++
#include "std_msgs/msg/string.hpp" 

using namespace std::chrono_literals; //std::chrono_literals 를 안붙여도 된다. 

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); //this를 꼭써야하나?
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

## using namespace
* [using namespace 사용하지 않는 경우 예제](./src/namespace_before.cpp)
* [using namespace 예제](./src/namespace.cpp)


## std::shared_ptr , std::make_shared
* #include <memory> import 하기
* std::make_shared<int>는 메모리 할당
  * new int()와 차이점은?
    * 순수하게 int 메모리만 할당
    * make_shared<int>는 int 메모리 할당 + 참조 카운터 관리를 위한 데이터 구조까지 
    * -> 안전한 프로그래밍이 가능!!!
    * new, delete 할 필요없음!
* [링크](https://webnautes.tistory.com/1451)
* [shared ptr before 예제](./src/sharedptr_before.cpp)
* [shared ptr 예제](./src/sharedptr.cpp)


