# [Time 사용하기(C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html)
1. 목표
1. 배경지식
1. 실행
1. 요약

## 목표
* 특정 시간의 transform을 얻는 법 학습하기.
* lookupTransform()함수를 이용하여 tf2 tree에서 transform이 사용 가능할 때까지 기다리는 법 학습하기.

## 배경지식
이전 튜토리얼에서는, tf2 broadcaster와 tf2 listener를 작성하여 turtle 데모를 재현해 보았다. 또한, transformation 트리에 새로운 프레임을 추가하는 법과, 어떻게 tf2가 좌표 프레임 트리를 추척하는지 배웠다. 이 트리는 시간이 따라 변화하며, tf2는 모든 transform에 대해 time snapshot을 저장한다(기본값 10초). 지금까지 우리는 tf2 tree에서 언제 transform이 기록되었는지 와는 무관하게 최신 transform을 얻기 위해 lookupTransform()함수를 사용했다. 이번 튜토리얼에서는 특정 시간의 transform을 얻는 방법을 다룰 것이다.

## 실행
### 1. tf2 and time
turtle2_tf2_listener.cpp 파일을 열고, lookupTransform() 호출을 보자.

```cpp
transformStamped = tf_buffer_->lookupTransform(
   toFrameRel,
   fromFrameRel,
   tf2::TimePointZero);
```

tf2::TimePointZero를 호출함으로써, time을 0으로 특정한다는 것을 알 수 있다.

> **<i class="fa fa-exclamation-triangle" aria-hidden="true"></i> 주의:** tf2 패키지는 rclcpp::Time과는 다른 자신만의 time 타입을 가진다. tf2_ros 패키지의 많은 API들은 자동으로 rclcpp::Time과 tf2::Timepoint를 변환한다.<br><br>여기서 rclcpp::Time(0, 0, this->get_clock()->get_clock_type())은 사용 될 수 있었지만, tf2::TimePointZero로 변환되었을 것이다.

tf2의 경우, time 0는 버퍼에 있는 "최신의, 이용 가능한" transform을 의미한다. 이제 이 라인을 현재 시간의 transform을 얻기 위해, this->get_clock()->now()으로 바궈보자. 

```cpp
rclcpp::Time when = this->get_clock()->now();
t = tf_buffer_->lookupTransform(
  toFrameRel, fromFrameRel,
  when);
```

이제 패키지를 다시 빌드한 후, 런치 파일을 실행해 보자.

```bash
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
````

아래 아래와 같은 출력과 함께 fail할 것이다. 

```
[INFO] [1629873136.345688064] [listener]: Could not transform turtle1 to turtle2: Lookup would
require extrapolation into the future.  Requested time 1629873136.345539 but the latest data
is at time 1629873136.338804, when looking up transform from frame [turtle1] to frame [turtle2]
```

이는 그 프레임이 존재하지 않거나, 미래에 있다는 것을 말한다.

왜 이런 일이 발생하는지에 대해 이해하기 위해서는, 버퍼가 어떻게 동작하는지 이해할 필요가 있다. 먼저, 각 listener는 다른 tf2 broadcaster들로부터 오는 모든 좌표프레임을 저장하는 버퍼를 가진다. broadcaster가 transform을 내보내면, 그것이 버퍼로 들어가기까지 약간의 시간이 필요하다(대략 수 밀리초). 결과적으로 우리가 "현재" 시간에 프레임 transform에 대해 요청하면, 그 정보다 도착하기 까지 몇 밀리초를 기다려야 한다.

### 2. transform 기다리기
tf2는 transform이 사용할 수 있을 때까지 기다리는 유용한 툴을 제공한다. 우리는 lookupTransform()에 timeout 파라미터를 추가함으로써 이것을 사용할 수 있다. 아래와 같이 코드를 수정해 보자(마지막에 timeout 파라미터 추가)

```cpp
rclcpp::Time when = this->get_clock()->now();
t = tf_buffer_->lookupTransform(
  toFrameRel, fromFrameRel,
  when, 50ms);
```

lookupTransform()은 4개의 인자를 받아들일 수 있으며, 마지막 timeout은 선택사항이다. lookupTransform()은 timeout될 때까지 이 시간동안 block 된다.

### 결과 확인하기
이제 패키지를 다시 빌드런치 파일을 실행해 보자.

```bash
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
```

lookupTransform()은 실제로 두 turtle이 이용 가능해질 때까지 block될 것이다(대략 수 밀리초). Timeout이 되고(50 [ms]), transform 여전히 이용 불가능하면 exception이 발생할 것이다.

## 요약
이번 튜토리얼에서는 특정 timestamp에서 transform을 얻는 방법과, lookupTransform() 함수를 사용할 때 tf2 tree에서 transform이 이용 가능해질 때까지 기다리는 법을 학습했다.
