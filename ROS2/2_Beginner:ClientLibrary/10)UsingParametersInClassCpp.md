# parameter 사용하기 (C++)
1. 개요
2. 실습

## 1. 개요
* ROS parameter를 사용하기
* node를 작성할때 해당 node에서 필요한 parameters가 필요한 경우가 있다. 

## 2. 실습
### 2-1 package 생성하기

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```

### 2-1-1 package.xml 업데이트하기
* package 생성시에 --dependencies 옵션을 사용하여서 package.xml과 CMakeLists.txt에 의존성을 추가하지 않아도 된다.
* package.xml 수정하기
```xml
<description>C++ parameter tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
### 2-2 C++ node 작성하기
* ros2_ws/src/cpp_parameters/src/cpp_parameters_node.cpp 파일 생성
```c++
#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    this->declare_parameter("my_parameter", "world");

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
```
### 2-2-1 실행자(executable) 추가하기
* CMakeLists.txt 수정하기 (find_package(rclcpp REQUIRED) 아래 추가하기)
```cmake
add_executable(minimal_param_node src/cpp_parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)

install(TARGETS
    minimal_param_node
  DESTINATION lib/${PROJECT_NAME}
)
```

### 2-3 빌드 및 실행하기
* 의존성 확인하는 명령 실행
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

* 새 package를 빌드하기
```bash
cd ~/ros2_ws
colcon build --packages-select cpp_parameters
```

* 새 터미널 열고 source 하기
```bash
source install/setup.bash
```

* node 실행하기
```bash
ros2 run cpp_parameters minimal_param_node
```
* 결과
```
[INFO] [minimal_param_node]: Hello world!
```

### 2-3-1 console로 변경하기
* node 실행하기
```bash
ros2 run cpp_parameters minimal_param_node
```
* 새 터미널 열고 source 실행한 후에 아래 명령 실행

```bash
ros2 param list
```

* my_parameter 설정하기
```bash
ros2 param set /minimal_param_node my_parameter earth
```

* 결과
```
Set parameter successful
```

* 새 터미널에서 source 후 아래 명령 실행해보기
```bash
ros2 run cpp_parameters minimal_param_node
```

* 결과
```
[INFO] [minimal_param_node]: Hello earth!
```

### 2-3-2 launch 파일로 변경하기
* ros2_ws/src/cpp_parameters/ 디렉토리 내부에 launch 디렉토리 생성
* launch 디렉토리 내부에 cpp_parameters_launch.py 파일 (아래 내용 붙여넣기)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
```

* CMakeLists.txt 파일에 아래 코드 추가하기
```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

* 빌드하기
```bash
colcon build --packages-select cpp_parameters
```

* 새 터미널에서 source 하기
```bash
source install/setup.bash
```

* node 실행하기
```bash
ros2 launch cpp_parameters cpp_parameters_launch.py
```

* 결과
```
[INFO] [custom_minimal_param_node]: Hello earth!
```

