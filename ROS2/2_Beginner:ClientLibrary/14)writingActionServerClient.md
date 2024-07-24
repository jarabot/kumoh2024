# Action server & client 구현하기
1. 소개
2. action_tutorials_cpp package 생성
3. action server node 작성
4. action client node 작성
5. 빌드 및 실행



## 1. 소개
* C++로 Action server와 action client node를 생성하고 실행하기

* Actions
  * [Quick review](https://github.com/jarabot/kumoh2024/blob/main/ROS2/1_Beginner%3ACLITools/7_actions.md) 
  * 오랜 시간이 소요되는 작업의 요청과 처리, 피드백과 결과 응답에 사용되는 Node 간 통신 메커니즘 
  * Action 구성 요소 (Topic과 Service로 구성)
    * Goal Service: 달성할 목표 (Request: Client > Server, Response: Server > Client)
    * Feedback Topic: 작업 진행 관련, 주기적으로 전달되는 피드백 (Server > Client)
    * Result Service: 작업 수행 결과 (Request: Client > Server, Response: Server > Client)
  * [Action이 어디에 쓰일 수 있을까?](https://drive.google.com/file/d/1P24yneCbRiMntZiYPjLHcVDJZ2-OIncM/view?usp=sharing)

 ## 2. action_tutorials_cpp package 생성 
 ### 2-1 action_tutorials_cpp package 생성
* action server, client 작성에 필요한 패키지 생성

* 새 터미널에서 workspace로 이동
```bash
cd ~/ros2_ws
```

* source 명령
```bash
source /opt/ros/humble/setup.bash
```
* 위 명령을 아래와 같이 ~/.bashrc에 추가하면 터미널을 실행할 때마다 자동으로 source 명령이 수행된다
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```
* src 디렉토리로 이동
```bash
cd ~/ros_ws/src
```

* action_tutorials_cpp package 생성
```bash
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
```
* Windows 환경일 경우, 빌드와 실행을 위해 visibility control 설정이 필요하다 ( [참고](http://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) )

 ## 3 action server node 작성

* action_tutorials_cpp package 위치로 이동
```bash
cd ~/ros2_ws/action_tutorials_cpp/src
```
* touch 명령으로 소스 파일 생성
```bash
touch fibonacci_action_server.cpp 
```

* 생성된 소스 파일을 코드 편집기로 연 후, 다음 코드 복사 및 저장

```c++
#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)


```

* include
```cpp
#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

```

* FibonacciActionServer class 생성자에서 Node 이름 ("fibonacci_action_server") 설정
```cpp
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
```

* FibonacciActionServer class 생성자에서 action server 객체 생성
  * Action type: Fibonacci
  * create_server() 함수: Fibonacci action type에 대한 action server 생성
  * action server node 이름: “fibonacci_action_server”
  * action 이름: "fibonacci"
  * Goals callback 함수 설정: handle_goal()
  * Cancel callback 함수 설정: handle_cancel()
  * Accep callback 함수: handle_accept()
    
  ```cpp
      this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this,
        "fibonacci",
        std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
        std::bind(&FibonacciActionServer::handle_cancel, this, _1),
        std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  ```

* 콜백 함수 정의
  * handle_goal(): 새로운 Goal 수신 시 호출됨
    * 수신된 Goal을 승인하고 GoalResponse 메시지 반환
    ```cpp
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Fibonacci::Goal> goal)
      {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
    ```
    
  * handle_cancel(): Cancel 수신 시 호출됨
   * 수신된 Cancel 메시지 승인, CancelResponse 메시지 반환 
   ```cpp
      rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFibonacci> goal_handle)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      }
    ```
  * handle_accepted(): Action을 실제로 실행할 때 호출됨
    * 새로운 thread를 실행하여 action을 처리하게 함
    * 실제 처리는 excute() 함수에서 이뤄짐    
    ```cpp
       void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
       {
         using namespace std::placeholders;
         // this needs to return quickly to avoid blocking the executor, so spin up a new thread
         std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
       }
    ```
* execute(): Action 실행 함수
  ```cpp
        void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
  ```
### 3-1 빌드 설정 - CMakeLists.txt
* 프로젝트 디렉토리 내 CMakeLists.txt 를 편집기로 연 후, 다음 내용 추가 (find_package 이후)
```cpp
  add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```
* 새 터미널 열고, workspace 디렉토리로 이동
```bash
cd ~/ros_ws
```
* 의존 패키지 확인&설치
```bash
rosdep install -i --from-path src --rosdistro humble -y
```

* colcon build
```bash
colcon build --packages-select action_tutorials_cpp
```

## 4 action client node 작성

* action_tutorials_cpp package 위치로 이동
```bash
cd ~/ros2_ws/action_tutorials_cpp/src
```
* touch 명령으로 소스 파일 생성
```bash
touch fibonacci_action_client.cpp 
```

* 생성된 소스 파일을 코드 편집기로 연 후, 다음 코드 복사 및 저장

```c++
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)

```


* FibonacciActionClient 생성자
  * Node 이름 설정: "fibonacci_action_client"
    ```cpp
    explicit FibonacciActionClient(const rclcpp::NodeOptions & options) : Node("fibonacci_action_client", options)
    ```
 
* FibonacciActionClient 생성자에서 action client 생성
  * Action type:  Fibonacci
  * Action 이름:  "fibonacci"
  ```cpp
      this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");
  ```
* send_goal() 함수 호출: action server에게 goal 요청 메시지 전달
  ```cpp
      this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
  ```
* send_goal()
  * timer 취소 (send_goal() 함수가 한 번만 호출되도록)  
  * action server가 이용 가능할 때까지 대기
  * Goal 메시지 생성
  * Goal response 콜백 함수 설정 - goal_response_callback()
  * Feedback 콜백 함수 - feedback_callback()
  * Result 콜백 함수 - result_callback()
  ```cpp
       void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  } 
  ```
* 콜백 함수 정의
  * goal_response_callback(): goal response 수신 시 호출됨
    * 요청된 Goal이 Action server에 의해 승인되었는지 (또는 거부되었는지) 확인
  ```cpp
    void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  ```
  
  * feedback_callback(): feedback 수신 시 호출됨
    * 요청된 action 처리 현황 (feedback) 출력  
  ```cpp
    void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
  ```
  * result_callback(): result 수신 시 호출됨
    * Action 처리 결과 출력  
  ```cpp
    void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
  ```
### 4.1 CMakeLists.txt
* 프로젝트 디렉토리 내 CMakeLists.txt 열기
* CMakeLists.txt에 다음 내용 추가 (find_package 이후)
```cpp
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
```

## 5. 빌드 및 실행
* 새 터미널 열고 프로젝트 디렉토리로 이동
```bash
   cd ~ros_ws
```

* colcon build
```bash
   colcon build
```

* 새 터미널 열고 프로젝트 디렉토리로 이동
```bash
   cd ~ros_ws
```

* action server 실행
  * 새 터미널 열고 프로젝트 디렉토리로 이동
  ```bash
     cd ~ros_ws 
  ```
  * source 명령 실행
  ```bash
  source install/setup.bash
  ```
  * action server node 실행
  ```bash
  ros2 run action_tutorials_cpp fibonacci_action_server
  ```

* action client 실행
  * 새 터미널 열고 프로젝트 디렉토리로 이동
  ```bash
     cd ~ros_ws 
  ```
  * source 명령 실행
  ```bash
  source install/setup.bash
  ```
  * action client node 실행
  ```bash
  ros2 run action_tutorials_cpp fibonacci_action_client
  ```

  * action server 실행 결과
  * Goal 요청 수신
  * Goal 실행 & 피드백 전달
  * Goal 결과 보고  
  ```bash
  [INFO] [1721540116.431433101] [fibonacci_action_server]: Received goal request with order 10
  [INFO] [1721540116.433206975] [fibonacci_action_server]: Executing goal
  [INFO] [1721540116.434414283] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540117.433753784] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540118.433717597] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540119.433747555] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540120.433697565] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540121.433683133] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540122.434108886] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540123.433729234] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540124.433796768] [fibonacci_action_server]: Publish feedback
  [INFO] [1721540125.435071946] [fibonacci_action_server]: Goal succeeded
  ``` 

* action client 실행 결과
  * Goal 요청 송신
  * Goal 피드백 수신
  * Goal 결과 수신 
  ```bash
  [INFO] [1721540116.429967406] [fibonacci_action_client]: Sending goal
  [INFO] [1721540116.433018183] [fibonacci_action_client]: Goal accepted by server, waiting for result
  [INFO] [1721540116.435119305] [fibonacci_action_client]: Next number in sequence received: 0 1 1 
  [INFO] [1721540117.433933963] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 
  [INFO] [1721540118.434021564] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 
  [INFO] [1721540119.434434228] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 
  [INFO] [1721540120.433912345] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 
  [INFO] [1721540121.433869762] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 
  [INFO] [1721540122.434901052] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 
  [INFO] [1721540123.433984938] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 
  [INFO] [1721540124.434138744] [fibonacci_action_client]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 55 
  [INFO] [1721540125.436054440] [fibonacci_action_client]: Result received: 0 1 1 2 3 5 8 13 21 34 55 
  ``` 
  
  
  
