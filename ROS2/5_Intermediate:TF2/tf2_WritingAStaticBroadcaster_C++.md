# [static broadcaster 작성하기(C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html)
1. 목표
1. 배경지식
1. 사전 준비
1. static transform을 publish하는 적절한 방법
1. 요약

## 목표
* static coordinate frame을 broadcast하는 방법 배우기.

## 배경지식
static transform을 publish하는 것은, 로봇의 base와 센서 및 움직임이 없는 부분 사이에 관계를 정의할 때 유용하다. 예를 들면, 레이저 스캐너의 중앙에 위치한 프레임에서 레이저 스캔 측정값을 추론하는 것이 가장 쉽다.

이 튜토리얼에서는 코드를 작성하여 static transform을 publish하는 방법과 static_transform_publisher라는 command line tool 사용법을 다룬다.

## 사전준비
* 워크스페이스 및 패키지 만드는 방법 숙지

## 실습
### 1. 패키지 만들기
먼저, 이 튜토리얼 및 다음 튜토리얼에서 사용될 패키지를 생성할 것이다. 패키지의 이름은 learning_tf2_cpp이며 의존성 패키지들은 geometry_msgs, rclcpp, tf2, tf2_ros, turtlesim이 있다.

새로운 터미널을 실행하고 ROS2 환경을 source한 후, 워크스페이스의 src폴더로 이동한다. 그 다음 아래의 명령어를 실행해 새로운 패키지를 생성한다.
```bash
ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp
```

### 2. static broadcaster 노드 작성하기
먼저 소스파일을 작성해보자. 아래의 명령어로 src/learning_tf2_cpp/src 위치에 static broadcaster 코드 예제를 다운로드 해보자.

```bash
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/static_turtle_tf2_broadcaster.cpp
```

```cpp
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher(char * transformation[])
  : Node("static_turtle_tf2_broadcaster")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms(transformation);
  }

private:
  void make_transforms(char * transformation[])
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(
      atof(transformation[5]),
      atof(transformation[6]),
      atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc != 8) {
    RCLCPP_INFO(
      logger, "Invalid number of parameters\nusage: "
      "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
      "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // As the parent frame of the transform is `world`, it is
  // necessary to check that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
    return 1;
  }

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  rclcpp::shutdown();
  return 0;
}
```

### 2.1 코드 설명
첫번째 라인은 필요한 헤더파일들을 include하고 있다. 먼저 publish할 때 필요한 TransformStamped 메세지 타입에 접근을 위해, geometry_msgs/msg/transform_stampled.hpp를 include한다. 

```cpp
#include "geometry_msgs/msg/transform_stamped.hpp"
```

그 후 rclcpp::Node 클래스 사용을 위해 rclcpp를 include한다.

```cpp
#include "rclcpp/rclcpp.hpp"
```

오일러 각과 쿼터니언 사이의 변환에 유용한 tf2::Quaternion클래스와, static transform의 publish를 쉽게 해주는 StaticTransformBroadcaster클래스를 include해준다.

```cpp
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
```

StaticFramePublisher클래스는 생성자를 통해 static_turtle_tf2_broadcaster라는 이름으로 노드를 초기화 한다. 그 후, static transformation을 전송할 StaticTransformBroadcaster가 생성된다.

```cpp
explicit StaticFramePublisher(char * transformation[])
: Node("static_turtle_tf2_broadcaster")
{
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
```

make_transforms메소드에서는 우리가 전송할 TransformStamped 오브젝트가 생성된다. 실제 tarnsform을 전송하기 전에, 먼저 다음과 같은 메타데이터를 채워넣어야 한다.

1. 타임스탬프 : 현재 시간을 적용한다.
1. 부모 프레임의 이름 : world
1. 자식 프레임의 이름 : 노드 실행 시, 인자로 입력받을 link 이름

```cpp
geometry_msgs::msg::TransformStamped t;

t.header.stamp = this->get_clock()->now();
t.header.frame_id = "world";
t.child_frame_id = transformation[1];
```

그 다음, turtle의 6차원 자세를 채워넣는다. 입력 받을 x, y, z값은 그대로 채워 넣고, r, p, y값은 쿼터니언으로 변환하여 채워 넣는다.

```cpp
t.transform.translation.x = atof(transformation[2]);
t.transform.translation.y = atof(transformation[3]);
t.transform.translation.z = atof(transformation[4]);
tf2::Quaternion q;
q.setRPY(
  atof(transformation[5]),
  atof(transformation[6]),
  atof(transformation[7]));
t.transform.rotation.x = q.x();
t.transform.rotation.y = q.y();
t.transform.rotation.z = q.z();
t.transform.rotation.w = q.w();
```

마지막으로 sendTransform()함수를 사용해 static transform을 broadcast한다.

```cpp
tf_static_broadcaster_->sendTransform(t);
```

### 2.2 CMakeLists.txt
추후 ros2 run 명령을 통해 실행하기 위해, CmakeLists.txt에 static_turtle_tf2_broadcaster라는 이름의 executable을 추가해 준다.

```cmake
add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
   static_turtle_tf2_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
)
```

그리고 ros2 run 명령이 이 executable을 찾을 수 있도록 install(TARGETS...) 섹션을 추가한다.

```cmake
install(TARGETS
   static_turtle_tf2_broadcaster
   DESTINATION lib/${PROJECT_NAME})
```

### 3. Build
다음 명령어를 워크스페이스 최상단 위치에서 실행하여 의존성을 체크한다.

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

그 다음, 새로운 패키지를 빌드한다.

```bash
colcon build --packages-select learning_tf2_cpp
```

### 4. 실행
이제 static_turtle_tf2_broadcaster 노드를 실행해보자.
새로운 터미널을 열고, 워크스페이스 최상단에서 setup파일을 source한다.

```bash
. install/setup.bash
```

그 후 아래의 명령어를 통해 노드를 실행한다.

```bash
ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
```

실행된 노드는 mystaticturtle에 대한 turtlr 자세 broadcast를 지상 1m 위에서 떠있도록 설정한다.
이제 우리는 tf_static 토픽을 출력해서 static transform이 publish되었는지 확인할 수 있다.

```bash
ros2 topic echo /tf_static
```

정상적으로 실행되었을 경우, 아래과 같이 static transform을 볼 수 있다.

```
transforms:
- header:
   stamp:
      sec: 1622908754
      nanosec: 208515730
   frame_id: world
child_frame_id: mystaticturtle
transform:
   translation:
      x: 0.0
      y: 0.0
      z: 1.0
   rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

## statis transform을 publish하는 적절한 방법
이 튜토리얼은 StaticTransformBroadcaster가 static transform을 publish하기 위해 어떻게 사용되는지를 보여주기 위함이다. 실제 개발 프로세스에서는, 이 코드를 직접 작성하는 것이 아니라, 전용 툴인 tf2_ros를 사용하여 작성하여야 한다. tf2_ros는 launch파일에 노드로 추가하거나 command line tool로서 사용될 수 있는 static_transform_publisher이라는 executable을 제공한다.

아래와 같이 command line tool로서 static_transform_publihser를 사용하여, static transform을 publish 해보자.

```bash
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id
```

쿼터니언으로도 가능하다.

```bash
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id
```

또한 launch파일에서도 사용 가능하다.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'mystaticturtle']
        ),
    ])
```

--frame-id와 --child-frame-id를 제외한 모든 인수들은 선택사항이다. 

## 요약
이번 튜토리얼에서는 world프레임과 mystaticturtle프레임 관계에서 처럼, 프레임들 사이에 고정된 관계를 정의하는데 static transform이 얼마나 유용한지 학습해 보았다. 또한, static transform이 공통된 프레임을 적용함으로써 레이저 스캐너와 같은 센서 데이터를 이해하는 데 얼마나 유용한 지 학습하였다. 마지막으로, static transform을 publish하는 노드를 작성해 보고, static_transform_publisher를 이용하여 static transform을 publish하는 방법을 학습하였다.
