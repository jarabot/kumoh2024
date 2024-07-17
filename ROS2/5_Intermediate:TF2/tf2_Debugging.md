# [디버깅](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html)
1. 목표
1. 배경지식
1. 디거빙 예제
1. 요약

## 목표
* tf2관련 문제를 디버깅하기 위해 체계적인 접근 방식을 사용하는 방법 학습하기

## 배경지식
이 튜토리얼은 일반적인 tf2 문제를 디버깅하는 단계를 다룬다. 또한, tf2_echo, tf2_monitor, view_frames와 같은 tf2디버깅툴들을 사용할 것이다. 이 튜토리얼은 tf2에 관련된 이전 튜토리얼들을 모두 선행 학습했다고 가정한다.

## 디버깅 예제
### 에제 세팅 및 시작하기
이번 튜토리얼을 위해, 많은 문제를 갖고 있는 데모 앱을 셋팅할 것이다. 이번 튜토리얼은 이런 문제들을 찾아내고 해결하기 위해 체계적인 접근 방식을 적용하는 것이 목표이다. 먼저 소스 파일을 만들어 보자.

learning_tf_cpp 패키지의 소스 파일 중, turtlr_tf2_listener.cpp를 복사하여 turtle_tf2_listener_debug.cpp라는 파일을 만들자.

파일을 열고 67번째 줄을 아래와 같이 수정한다.

```cpp
std::string toFrameRel = "turtle3";
```

그리고 75~79번째 줄에 있는 lookupTransform() 호출을 아래와 같이 수정한다.

```cpp
try {
   t = tf_buffer_->lookupTransform(
     toFrameRel,
     fromFrameRel,
     this->now());
} catch (tf2::TransformException & ex) {
```

그리고 파일을 정한다. 이 데모를 실행하기 위해, launch 디렉토리에 start_tf2_debug_demo.launch.py 파일을 생성하고 아래 내용을 입력해 준다.

```cpp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'target_frame', default_value='turtle1',
         description='Target frame name.'
      ),
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         output='screen'
      ),
      Node(
         package='learning_tf2_cpp',
         executable='turtle_tf2_broadcaster',
         name='broadcaster1',
         parameters=[
               {'turtlename': 'turtle1'}
         ]
      ),
      Node(
         package='learning_tf2_cpp',
         executable='turtle_tf2_broadcaster',
         name='broadcaster2',
         parameters=[
               {'turtlename': 'turtle2'}
         ]
      ),
      Node(
         package='learning_tf2_cpp',
         executable='turtle_tf2_listener_debug',
         name='listener_debug',
         parameters=[
               {'target_frame': LaunchConfiguration('target_frame')}
         ]
      ),
   ])
```

turtle_tf2_listener_debug executable을 CMakeLists.txt에 추가하고 빌드하자.

```cmake
add_executable(turtle_tf2_listener_debug src/turtle_tf2_listener_debug.cpp)
ament_target_dependencies(
    turtle_tf2_listener_debug
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    turtlesim
)

install(TARGETS
    turtle_tf2_listener_debug
    DESTINATION lib/${PROJECT_NAME})
```

이제 실행하여 무슨 일이 일어나는지 살펴 보자.

```bash
ros2 launch learning_tf2_cpp start_tf2_debug_demo.launch.py
```

turtlesim이 실행되는 것을 볼 수 있을 것이다. 또한 새로운 터미널에서 turtle_teleop_key를 실행하면 화살표키를 이용해서 turtle1을 조종할 수 있다.

```bash
ros2 run turtlesim turtle_teleop_key
```

두 번째 turtle은 더 왼쪽 아래에 있는 것을 확인할 수 있다. 만약 데모가 정상적으로 작동한다면, 두 번째 turtle은 첫 번째 turtle이 움직이는데로 따라가야 한다. 그라나 몇가지 문제가 해결되지 않았기 때문에, 따라가지 않을 것이다. 아래와 같은 출력을 확인해 볼 수 있다.

```
[INFO] [1630223557.477636052] [tf2_echo]: Waiting for transform turtle3 ->  turtle1:
Invalid frame ID "turtle3" passed to canTransform argument target_frame - frame does
not exist
```

### 2. tf2 요청 찾기
먼저 우리는 tf2에 요청하고 있는 것을 정확히 알 필요가 있다. 그러므로, tf2를 사용하는 코드로 가보자. turtle_tf2_listener_debug.cpp를 열고 67번째 줄을 보자.

```cpp
std::string to_frame_rel = "turtle3";
```

그리고 73~83줄은 아래와 같다.

```cpp
try {
   transformStamped = tf_buffer_->lookupTransform(
     toFrameRel,
     fromFrameRel,
     this->now());
} catch (tf2::TransformException & ex) {
```

이것이 우리가 실제로 tf2에 요청하고 있는 것이다. 이 3개의 인자는 now 시점에 turtle3에 대한 turtle1의 transform을 요청을 의미한다.

이제 왜 이러한 요청이 fail되었는지 확인해 보자.

### 3. 프레임 확인하기
먼저, tf2가 turtle3와 turtle1간에 transform을 알고있는지 확인하기 위해, tf2_echo 툴을 사용할 것이다.

```bash
ros2 run tf2_ros tf2_echo turtle3 turtle1
```

실행해보면, 아래와 같이 turtle3프레임은 존재하지 않는다고 출력된다.
```
[INFO] [1630223557.477636052] [tf2_echo]: Waiting for transform turtle3 ->  turtle1:
Invalid frame ID "turtle3" passed to canTransform argument target_frame - frame does
not exist
```

어떤 프레임들이 실제로 존재하는지를 시작적으로 확인하기 위해 view_frames 툴을 사용해 보자. 

```bash
ros2 run tf2_tools view_frames
```

생성된 frames.pdf 파일을 다음과 같다.

![](https://docs.ros.org/en/humble/_images/turtlesim_frames.png)

문제는 존재하지 않는 turtle3에 대한 transform요청을 하고 있는 것이다. 이 버그를 수정하기 위해, 67번째 줄의 turtle2를 turtle3로 수정하자.

```cpp
std::string toFrameRel = "turtle2";
```

실행중인 데모를 중지하고, 빌드 후 재 실행하자.

```bash
ros2 launch learning_tf2_cpp start_tf2_debug_demo.launch.py
```

곧바로 다음 문제가 아래와 같이 출력된다.

```
[turtle_tf2_listener_debug-4] [INFO] [1630223704.617382464] [listener_debug]: Could not
transform turtle2 to turtle1: Lookup would require extrapolation into the future. Requested
time 1630223704.617054 but the latest data is at time 1630223704.616726, when looking up
transform from frame [turtle1] to frame [turtle2]
```

### 4. Timestamp 체크하기
이제 프레임 이름 문제를 해결 했으니, timestamp를 살펴볼 차례이다. 우리가 현재 시점(now)에 대한 turtle1와 turtle2간에 transform을 요청하고 있다는 것을 기억해보자. 타이밍 관련 정보를 얻기 위해 tf2_monitor를 해당 프레임으로 호출해보자. 

```bash
ros2 run tf2_ros tf2_monitor turtle2 turtle1
```

아래와 같이 출력을 볼 수 있을 것이다.

```
RESULTS: for turtle2 to turtle1
Chain is: turtle1
Net delay     avg = 0.00287347: max = 0.0167241

Frames:
Frame: turtle1, published by <no authority available>, Average Delay: 0.000295833, Max Delay: 0.000755072

All Broadcasters:
Node: <no authority available> 125.246 Hz, Average Delay: 0.000290237 Max Delay: 0.000786781
```

핵심 부분은 turtle2에서 turtle1까지 체인에서의 딜레이이다. 출력은 평균 약 3밀리초 정도의 딜레이가 있다는 것을 보여준다. 이것은 tf2가 3밀리초가 지난 후에야 transform할 수 있다는 것을 의미한다. 그래서 만약 now 시점이 아닌 3밀리초 전 시점에서의 turtle간에 transformation을 요청한다면, tf2는 응답을 줄 수 있을 것이다. 아래와 같이 75~79번째 줄을 수정해서 테스트해보자.

```cpp
try {
   t = tf_buffer_->lookupTransform(
     toFrameRel,
     fromFrameRel,
     this->now() - rclcpp::Duration::from_seconds(0.1));
} catch (tf2::TransformException & ex) {
```

새로운 코드에서, 우리는 100밀리초 전에서의 turtle간에 transform에 대한 요청을 하고 있다. transform이 도착했는지를 확실히 하기 위해서는 일반적으로 좀 더 긴 시간을 사용하는 편이다. 데모를 중지하고 빌드 후 재실행 해보자.

```bash
ros2 launch learning_tf2_cpp start_tf2_debug_demo.launch.py
```

이제 turtle이 정상적으로 움직일 것이다.

![](https://docs.ros.org/en/humble/_images/turtlesim_follow1.png)

마지막으로 했던 수정이 우리가 원하는 것과 완벽히 일치하지는 않다. 단지 문제의 원인을 파악하기 위함일 뿐이다. 실제로 해야할 수정은 아래와 같다.

```cpp
try {
   t = tf_buffer_->lookupTransform(
     toFrameRel,
     fromFrameRel,
     tf2::TimePointZero);
} catch (tf2::TransformException & ex) {
```

또는,

```cpp
try {
   t = tf_buffer_->lookupTransform(
     toFrameRel,
     fromFrameRel,
     tf2::TimePoint());
} catch (tf2::TransformException & ex) {
```

아래와 같이 사용할 수도 있다.

```cpp
try {
   t = tf_buffer_->lookupTransform(
     toFrameRel,
     fromFrameRel,
     this->now(),
     rclcpp::Duration::from_seconds(0.05));
} catch (tf2::TransformException & ex) {
```

## 요약
이번 튜토리얼에서는 tf2 관련 문제를 디버깅할때 체계적인 접근방식을 사용하는 법을 학습했다. 또한 이러한 tf2 문제를 디버깅할 때 도움을 주는 tf2_echo, tf2_monitor, view_frames과 같은 tf2 디버깅 툴을 사용하는 법을 학습했다.
