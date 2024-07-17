# service와 client 작성하기 (C++)
1. 개요
2. 실습
   1. package 생성하기
   2. service 작성하기
   3. client node 작성하기
   4. 빌드 및 실행하기
## 1. 개요
* C++로 service와 client nodes를 생성하고 빌드하기
* service를 사용해서 nodes간 통신 구현 방법 이해
* client node
  * request를 보내는 node
* server node
  * request를 수신하고 나서 이에 대한 response를 보내는 node
* request와 response는 .srv 파일로 정의

## 2. 실습
### 2-1 package 생성하기
* ros2_ws/src 디렉토리 내에 새로운 package를 생성하기

* cpp_srvcli package 생성하는 명령 실행
```bash
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
```
* exampel_interfaces package 내에 .srv 파일 (service에서 주고 받는 구조)

```
int64 a
int64 b
---
int64 sum
```

### 2-1-1 package.xml 업데이트하기
* package.xml 수정하기
```xml
<description>C++ client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

### 2-2 service node 작성하기
* ros2_ws/src/cpp_srvcli/src/add_two_ints_server.cpp 파일 생성하기

* add_two_ints_server.cpp 파일에 아래 내용 붙여넣기
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
### 2-2-1 실행자(executable) 추가하기
* CMakeLists.txt 파일 수정하기 (server라는 실행자를 생성하도록)
```cmake
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)
```

* 아래 코드 추가하기(ros2 run 명령으로 실행자 찾기 가능하도록)
```cmake
install(TARGETS
    server
  DESTINATION lib/${PROJECT_NAME})
```

### 2-3 client node 작성하기
* ros2_ws/src/cpp_srvcli/src/add_two_ints_client.cpp 파일 생성
* 아래 코드 붙여넣기
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

### 2-3-1 실행자(executable) 추가하기
* 최종 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### 2-4 빌드 및 실행하기
* 의존성 설치
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

* 빌드하기 (cpp_srvcli package만)
```bash
colcon build --parallel-workers 1 --packages-select cpp_srvcli
```
* 새 터미널 열고 source 하기
```bash
source install/setup.bash
```

* service node 실행하기
```bash
ros2 run cpp_srvcli server
```

* 결과
```
[INFO] [rclcpp]: Ready to add two ints.
```

* 새 터미널 열고 client node 실행하기
```bash
source install/setup.bash
ros2 run cpp_srvcli client 2 3
```

* 결과
```
[INFO] [rclcpp]: Sum: 5
```

* 결과(service node 터미널)
```
[INFO] [rclcpp]: Incoming request
a: 2 b: 3
[INFO] [rclcpp]: sending back response: [5]
```