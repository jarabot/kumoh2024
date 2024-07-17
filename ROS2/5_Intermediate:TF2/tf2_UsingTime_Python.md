# [Time 사용하기(Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Py.html)
> **<i class="fa fa-exclamation-triangle" aria-hidden="true"></i> 주의:** lookup_tranform()에 timtout 옵션 적용 안됨.
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
### 1. listener 노드 수정
turtle2_tf2_listener.py 파일을 열고, 76번째 줄에서 lookup_transform() 호출에 전달 될 timeout=Duration(seconds=1.0)파라미터를 제거하자. 수정 후는 다음과 같다.

```python
now = self.get_clock().now()
t = self.tf_buffer.lookup_transform(
   to_frame_rel,
   from_frame_rel,
   now)
```

또한, 파일 상단에다 우리가 처리하게 될 exception들을 추가로 import해주자.

```python
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
```

81번째 라인 except블럭에 새로 import될 exception을 추가하고, 디버깅을 위해 raise문을 추가하자.

```python
except (LookupException, ConnectivityException, ExtrapolationException):
   self.get_logger().info('transform not ready')
   raise
   return
```

이제 이제 다시 런치파일을 실행하면, fail된다는 것을 확인할 수 있다.

```bash
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py
```

### 2. listener 노드 고치기
fail되고 있는 것은 lookup_transform()인 것을 확인할 수 있다. 이는 frame이 없거나 데이터가 미래에 있다는 것을 의미하고 있다. 이를 고치기 위해, 아래와 같이 코드를 76번째 줄을 수정하자(timeout 파라미터를 반환).

```python
now = self.get_clock().now()
t = self.tf_buffer.lookup_transform(
   to_frame_rel,
   from_frame_rel,
   now,
   timeout=rclpy.duration.Duration(seconds=1.0))
```

lookupTransform()은 4개의 인자를 받아들일 수 있으며, 마지막 timeout은 선택사항이다. lookupTransform()은 timeout될 때까지 이 시간동안 block 된다.

> **<i class="fa fa-exclamation-triangle" aria-hidden="true"></i> 주의:** 수정 후, except() 블럭의 raise문을 삭제하지 않으면 지속적으로 에러가 발생할 것이다.

이제 패키지를 다시 런치 파일을 실행해 보자.

```bash
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py
```

lookupTransform()은 실제로 두 turtle이 이용 가능해질 때까지 block될 것이다(대략 수 밀리초). Timeout이 되고(50 [ms]), transform 여전히 이용 불가능하면 exception이 발생할 것이다.

## 요약
이번 튜토리얼에서는 특정 timestamp에서 transform을 얻는 방법과, lookupTransform() 함수를 사용할 때 tf2 tree에서 transform이 이용 가능해질 때까지 기다리는 법을 학습했다.
