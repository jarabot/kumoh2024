# [간단한 service와 client 작성하기](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. package 생성하기
   2. service node 작성하기
   3. client node 작성하기
   4. 빌드 및 실행
5. 요약

## 목표
* service와 client nodes를 C++로 생성하기
* 생성한 service와 client nodes 실행하기

## 배경지식
* nodes가 services를 사용하여 통신하는 경우
  * data에 대한 request를 보내는 node
    * client node
  * request에 대한 response를 보내는 node
    * service node
* request와 response 구조
  * .srv 파일로 결정

* 여기서 사용된 예제의 동작
  * 간단한 정수 더하는 시스템
  * clinet node는 server에게 2개 정수값을 전달하면서 sum의 결과를 요청(request)
  * service node는 요청에 대한 결과를 응답(response)으로 전달

## 사전준비
* workspace 생성 이해
* package 생성 이해 

## 실습
###  1. package 생성하기
* 새 터미널 열기
* ROS2 환경 source 하기
* ros2 명령 동작 확인하기
* 이전 튜터리얼에서 ros2_ws 디렉토리로 가기
* packages는 반드시 src 디렉토리에 생성해야만 한다. (workspace에 하지 않도록 주의!)
* ros2_ws/src로 가서 새로운 package 생성하기
```
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
```
* 정상적으로 실행된 경우 cpp_srvcli package 생성 및 필요한 파일과 디렉토리가 생성되었다는 메시지 확인

* --dependencies 인자를 사용하면 자동으로 package.xml과 CMakeLists.txt에 필요한 의존성을 추가시킨다.
* example_interfaces package는 request와 response에 사용할 .srv 파일을 포함하고 있다.
```
int64 a
int64 b
---
int64 sum
```
* 첫번째 2줄은 request이고 ---는 아래 response 부분과 구분을 짓는데 사용된다.

### 1.1 package.xml 업데이트하기
* package 생성시에 --dependencies 옵션을 사용했으므로 수동으로 package.xml과 CMakeLists.txt를 수정할 필요가 없다.
* 가급적이면 package.xml 파일에 다음 정보를 추가하도록 하자.
```xml
<description>C++ client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

###  2. service node 작성하기
* ros2_ws/src/cpp_srvcli/src 디렉토리 내부에 add_two_ints_server.cpp 파일을 새로 생성하자. 
* 아래 내용을 add_two_ints_server.cpp 파일에 넣자.
```c++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```
### 2.1 code 살펴보기
* 맨 위에 2개 #include 구문은 내가 작성한 package가 의존하는 부분이다.
* add 함수는 request로 받은 2개 정수를 더하고 그 합을 response로 사용한다. log를 통해서 status를 콘솔에 알린다.
```c++
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
        request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}
```

* main 함수는 한줄씩 다음을 수행한다.

  * ROS2 C++ client library 초기화 : 
```c++
rclcpp::init(argc, argv);
```

  * add_two_ints_server 라는 node를 생성 : 
```c++
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
```
  * 이 node에 대한 add_two_ints 라는 service 생성. &add method로 네트워크 상에서 advertise한다.
```c++
rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
```
  * 준비가 되면 log 메시지를 출력
```c++
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
```
  * node를 spin시키고 service 사용 가능하게 만든다
```c++
rclcpp::spin(node);
```

### 2.2 실행자 추가하기
* add_executable 매크로는 ros2 run을 사용해서 실행할 수 있는 실행자를 생성한다.
* CMakeLists.txt에서 server라는 실행자 이름을 생성한다.
```cmake
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
rclcpp example_interfaces)
```
* ros2 run은 실행자를 찾을 수 있다. 파일 끝 부분에 아래 내용을 추가한다. ament_package() 바로 전 위치 
```cmake
install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})
```

* 이제 package를 빌드한다. 로컬 환경에 대해서 source를 실행한다. 이렇게 하면 실행이 가능하다. 하지만 먼저 client node를 만들어야 전체 시스템으 동작하는 것을 볼 수 있다.

###  3. client node 작성하기
* ros2_ws/src/cpp_servcli/src 디렉토리 내부에서 add_two_ints_client.cpp라는 파일을 만들고 아래 내용을 붙여넣자.
```c++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```
### 3.1 code 둘러보기
* service node와 유사하게 다음 코드들은 node를 생성하고 해당 node에 대한 client를 생성한다.
```c++
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
  node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
```

* 다음으로 request를 생성한다. request의 구조는 .srv 파일에서 이전에 정의했었다.
```c++
auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
request->a = atoll(argv[1]);
request->b = atoll(argv[2]);
```

* while loop에서 client가 service node를 찾는데 1초가 주어진다. 만약 찾지 못하면 계속 기다리게 된다.
```c++
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
```

* 만약 client가 cancle되면(터미널에서 Ctrl+c 누르는 경우), 인터럽트가 발생했다는 error log message를 반환한다.
```c++
RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
  return 0;
```
* 다음으로 client는 request를 보내고, 해당 node는 response를 수신할때까지 spin한다.

### 3.2 실행자 추가
* CMakeLists.txt 에 처리
  * 실행자 추가하기
  * 새로운 node에 대한 target 추가
* 자동으로 생성된 CMakeLists.txt 에서 불필요한 코드들을 삭제하고 나면 아래와 같은 상태가 된다.
```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

###  4. 빌드 및 실행
* ros2_ws 디렉토리 내에서 rosdep를 실행한다.
* rosdep를 실행하면 빌드하기 전에 놓친 의존성을 검사한다.

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

* workspace로 돌아와서 새로운 package를 빌드한다.
```bash
colcon build --packages-select cpp_srvcli
```

* 새로운 터미널을 열어서 ros2_ws로 이동하고 설정 파일을 source 한다.
```bash
. install/setup.bash
```

* 이제 service node를 실행한다.
```
ros2 run cpp_srvcli server
```

* 터미널은 다음 메시지를 반환하고 나서 대기상태가 된다.
```
[INFO] [rclcpp]: Ready to add two ints.
```

* 새로운 터미널을 열어서 ros2_ws 내부에서 설정 파일을 source 한다. 
* 아래 명령을 실행하여 client node를 구동시킨다.
```
ros2 run cpp_srvcli client 2 3
```

* 2와 3을 선택하면 client는 아래와 같은 response를 받게 된다.
```
[INFO] [rclcpp]: Sum: 5
```

* service가 실행되고 있는 터미널로 돌아오자. request를 받으면 request로 수신한 data를 log message로 publish한다. 그리고 reponse를 보내준다.
```
[INFO] [rclcpp]: Incoming request
a: 2 b: 3
[INFO] [rclcpp]: sending back response: [5]
```

## 요약
* 2개 nodes를 생성하여 service를 이용하여 request와 response data를 주고 받았다. 
* 의존성과 실행자를 package 설정 파일에 추가하여 빌드하였다.
* service/client 시스템이 동작하는 것을 보았다.
