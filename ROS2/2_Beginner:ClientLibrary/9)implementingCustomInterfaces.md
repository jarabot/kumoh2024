# [커스텀 interfaces 구현하기](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. package 생성하기
   2. msg 파일 생성하기
   3. 동일한 package에서 interface 사용하기
   4. 해보기
   5. 기존에 정의한 interface 사용하기
5. 요약

## 목표
* ROS2에서 커스텀 interfaces 구현하는 다양한 방법 배워보기
* 내 package에서 interfaces를 선언/생성하여 코드에서 사용하는 경우(추천하는 방법은 아니지만 편해서 이렇게 많이 사용함)
## 배경지식
* 추천 : interface package 위치
  * 해당 interface package에 선언되는 것
  * 하지만 솔직히 해당 interface를 사용하는 package에서 선언/생성해서 사용하는 것이 편하다. 
* 커스텀 msg와 srv interfaces 만드는 방법 이해
* 추천하는 방식 :
  * 하나의 package에 모든 interface를 선언하고 생성하는 것이 편할 수 있다.
  * 왜???
* interfaces는 CMake packages에서만 정의할 수 있었다. 하지만 Python libraries와 CMake packages 내에 nodes를 가질 수 있다.(ament_cmake_python 사용)
  * 하나의 package 내부에 interfaces와 Python nodes를 하나의 package로 정의할 수 있다.
* 간단하게 CMake package와 C++ nodes를 사용할 것이다.

* 이 튜터리얼에서는 msg interface type만 집중해서 다룬다. 여기서 배운 것을 다른 interface types에 대해서 동일하게 적용할 수 있다.
## 사전준비
* 커스텀 msg와 srv 파일 생성하는 방법 및 개념 이해
* ROS2 설치
* workspace 및 package 이해
## 실습
###  1. package 생성하기
* workspace의 src 디렉토리 내에 more_interfaces package를 생성하고 msg 파일을 저장할 디렉토리를 만든다.
```
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg
```

###  2. msg 파일 생성하기
* more_interfaces/msg 내부에 AddressBook.msg 파일 생성하고
* 이 파일 내부에 message를 생성하기 위해서 아래 코드를 붙여넣자.

```
bool FEMALE=true
bool MALE=false

string first_name
string last_name
bool gender
uint8 age
string address
```

* 이 message는 5개 필드로 구성된다.
  * first_name : string type
  * last_name : string type
  * gender : bool type, MALE or FEMALE
  * age : uint8 type
  * address : string type

* message 정의 내부에 각 필드에 대한 기본값을 설정하는 것도 가능하다!
  * [관련 문서](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)

* 다음으로 msg 파일이 사용할 수 있도록 프로그래밍 언어로 변환되어야 한다.
  * msg -> C++ 소스 파일로 변환이 되어야 C++ 에서 사용 가능

### 2.1 msg 파일 빌드하기
* package.xml 파일 열어서 다음 내용 붙여넣기
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
* 빌드 시점에 rosidl_default_generators가 필요하다.
* 실행 시점에 rosidl_default_runtime만 필요로 한다. 
  
* CMakeLists.txt 열어서 다음 내용 추가하기 :
* msg/srv 파일로부터 message code를 생성하는 해당 package 찾기
```
find_package(rosidl_default_generators REQUIRED)
```

* 생성하고자 하는 message의 목록을 선언한다.
```
set(msg_files
  "msg/AddressBook.msg"
)
```
* .msg 파일을 수동으로 추가해서 CMake는 다른 .msg 파일을 추가한 후에 해당 project를 재설정해야이 필요한 때를 알게 해야한다.
* message 생성하기 :
```
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
```
* message runtime 의존성을 export하기
```
ament_export_dependencies(rosidl_default_runtime)
```
* 이제 msg 정의로부터 소스 파일을 생성할 준비가 되었다. 4번째 step에서 한꺼번에 컴파일할 예정이다.

### 2.2 (옵션) 여러 interfaces 설정하기
* set을 이용하여 모든 interfaces의 목록을 편리하게 사용할 수 있다.
```
set(msg_files
  "msg/Message1.msg"
  "msg/Message2.msg"
  # etc
  )

set(srv_files
  "srv/Service1.srv"
  "srv/Service2.srv"
   # etc
  )
```
* 그리고 아래처럼하면 한번에 모든 목록을 생성한다.
```
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
```

###  3. 동일한 package에서 interface 사용하기
* 자 이제 이 message를 이용하여 코드를 작성해보자.

* more_interfaces/src 내부에 publish_address_book.cpp 파일을 생성하고 아래 코드를 붙여넣자.
```c++
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
  : Node("address_book_publisher")
  {
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::AddressBook();

        message.first_name = "John";
        message.last_name = "Doe";
        message.age = 30;
        message.gender = message.MALE;
        message.address = "unknown";

        std::cout << "Publishing Contact\nFirst:" << message.first_name <<
          "  Last:" << message.last_name << std::endl;

        this->address_book_publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();

  return 0;
}
```

### 3.1 code 설명
* include
  * AddressBook.msg가 변환된 헤더 파일
* node와 AddressBook publisher 생성하기
```c++
using namespace std::chrono_literals;

class AddressBookPublisher : public rclcpp::Node
{
public:
  AddressBookPublisher()
  : Node("address_book_publisher")
  {
    address_book_publisher_ =
      this->create_publisher<more_interfaces::msg::AddressBook>("address_book");
```
* 주기적으로 messages를 publish하기 위한 callback 정의
```c++
auto publish_msg = [this]() -> void {
```
* AddressBook message 인스턴스 생성
```
auto message = more_interfaces::msg::AddressBook();
```

* AddressBook 필드 채우기
```c++
message.first_name = "John";
message.last_name = "Doe";
message.age = 30;
message.gender = message.MALE;
message.address = "unknown";
```
* 주기적으로 message를 publish하기
```c++
timer_ = this->create_wall_timer(1s, publish_msg);
```
* 1초마다 publish_msg 함수를 호출하는 1초 타이머 생성

### 3.2 publisher 빌드하기
* CMakeLists.txt 파일에 이 node를 위한 새로운 target 생성이 필요하다.
```
find_package(rclcpp REQUIRED)

add_executable(publish_address_book
  src/publish_address_book.cpp
)

ament_target_dependencies(publish_address_book
  "rclcpp"
)

install(TARGETS publish_address_book
 DESTINATION lib/${PROJECT_NAME})
```

### 3.3 interface에 대한 Link
* 동일한 package 내부에서 생성한 messages를 사용하기 위해서 아래와 같은 CMake 코드를 사용
```
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(publish_address_book "${cpp_typesupport_target}")
```

* AddressBook.msg로부터 생성된 C++ 코드를 찾아서 target link를 할 수 있게 한다.
  * 동일한 package 내부에서 interface를 사용할려고 하는 경우에 이 부분이 필요하다.
  * 즉 다른 package에서 빌드된 interface를 사용하는 경우에는 이 부분이 필요없다.

###  4. 해보기
* package를 빌드하기 위해서 workspace 내부로 이동하여 빌드하기
```
cd ~/ros2_ws
colcon build --packages-up-to more_interfaces
```
* 다음으로 workspace를 source 명령실행 후에 publisher를 실행한다.
```
. install/local_setup.bash
ros2 run more_interfaces publish_address_book
```
* publish_address_book.cpp에서 설정한 값으로 msg가 publish되는 것을 확인해야만 한다.
* message가 address_book topic으로 publish되는 것을 확인하기 위해서 새 터미널을 열고, workspace를 source하고 topic echo 를 호출한다.
```
. install/setup.bash
ros2 topic echo /address_book
```

* 이 튜터리얼에서는 subscriber를 생성하지는 않지만 직접 subscriber를 작성하는 연습을 해보자.

###  5. (옵션)기존에 정의한 interface 사용하기
* 
## 요약
* 이 튜터리얼에서 하나의 package 내에서 정의한 interface 빌드와 사용을 하는 경우를 알아보았다.
* field type에 기존에 정의된 다른 interface를 사용하는 방법
* interface를 사용하기 위해서 수정해야 하는 부분 : package.xml, CMakeLists.txt, #include 구문을 사용하는 방법도 알아보았다.