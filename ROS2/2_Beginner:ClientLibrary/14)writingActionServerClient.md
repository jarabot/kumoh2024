# Action server & client 구현하기
* 1. 소개
* 2. action_tutorials_cpp package 생성
* 3. action server 작성
* 4. action client 작성
* 5. 빌드 및 실행
* Quiz


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

 ### 2-2 action server node 작성

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

* FibonacciActionServer class 생성자에서 Node 이름 ("FibonacciActionServer") 설정
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
  *   수신된 Goal을 승인하고 GoalResponse 메시지 반환

  * handle_cancel()
  * handle_accepted()
 
* 

