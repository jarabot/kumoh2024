# [시간여행하기(Python)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Py.html)
> **<i class="fa fa-exclamation-triangle" aria-hidden="true"></i> 주의:** error 발생 후 종료되는 버그 있음
1. 목표
1. 배경지식
1. lookupTransform() 고급 API
1. 결과 확인하기
1. 요약

## 목표
* tf2의 고급 시간여행 기능 학습하기

## 배경지식
이전 튜토리얼에서는, tf2와 time에 대한 기초를 다뤘다. 이번 튜토리얼에서는 한걸음 더 나아가 강력한 tf2 트릭인 시간여행에 대해 알아볼 것이다. tf2 라이브러리의 핵심 기능중 하나는 공간 뿐만 아니라 시간에 대한 데이터도 transform할 수 있다는 것이다.

tf2 시간여행 기능은 오랜 기간동안 로봇의 자세를 모니터링 하거나, 사람의 걸음을 따라가는 follower 로봇을 만들때와 같이 다양한 작업에서 유용할 수 있다. 우리는 이 시간여행 기능을 이전 시간의 transform을 찾고, carrot1의 5초 전을 turtle2가 따라가도록 프로그래밍하기 위해서 사용할 것이다.

## 시간 여행
먼저 carrot의 현재 위치한 곳으로 가는 두 번째 turtle 대신, carrot의 5초 전 위치로 가는 두 번째 turtle을 만들 것이다. turtle_tf2_listener.py파일 안에 lookupTransform() 호출을 수정하자.

```python
when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)
t = self.tf_buffer.lookup_transform(
    to_frame_rel,
    from_frame_rel,
    when,
    timeout=rclpy.duration.Duration(seconds=0.05))
```

이제 이것을 실행해보면, 두 번째 turtle은 5초 동안 어디로 가야할지 모를 것이다. 왜냐하면, 우리는 carrot의 위치에 대한 5초간의 history를 가지고 있지 않기 때문이다. 하지만 5초 뒤에는 무슨 일이 발생할까? 실행해 보자.

```bash
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py
```

![](https://docs.ros.org/en/humble/_images/turtlesim_delay1.png)

turtle은 위의 그림과 같이 제어할 수 없이 돌아다닐 것이다. 그 이유에 대해 이해해보자.

1. 코드에서, 우리는 다음과 같은 요청을 했다: "turtle2의 5초 전 위치에 대한 carrot1의 5초 전 위치는 어디였는가?". 이는 우리가 carrot의 5초 전 위치 뿐만 아니라 두 번째 turtle의 5초 전 위치를 기반으로 두 번재 turtle을 조종하고 있다는 것을 의미한다.
2. 그러나 우리가 진짜 원하는 것은 다음과 같다: "turtle2의 현재 위치에 대한 carrot1의 5초 전 위치는 어디였는가?".

### 2. lookupTransform() 고급 API
tf2에 이런 요청을 하기 위해서는, 특정 transformation을 언제 가져와야 하는지는 정확히 말할 수 있게 해주는 고급 API를 사용해야 할 것이다. 이는 추가 파라미터로 lookup_transform_full() 메소드를 호출하면 가능하다. 아래와 같이 코드를 수정하자.

```python
when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)
t = self.tf_buffer.lookup_transform_full(
        target_frame=to_frame_rel,
        target_time=rclpy.time.Time(),
        source_frame=from_frame_rel,
        source_time=when,
        fixed_frame='world',
        timeout=rclpy.duration.Duration(seconds=0.05))
```

LookupTransform() 고급 API는 6개의 인자를 받아들인다:

1. Target 프레임
2. Transfrom할 시간
3. Source 프레임
4. Source 프레임이 계산될 시간
5. 시간에 따라 변하지 않는 프레임 : world 프레임
6. Target 프레임이 이용 가능해질 때까지 기다릴 시간

요약하면 tf2는 백그라운드에서 다음을 수행한다. 과거 시점에서, carrot1에 대한 world로의 transform을 계산한다. world 프레임에서, tf2 시점은 과거에서 현재로 이동한다. 그리고 현재 시점에 tf2는 world에 대한 turtle2로의 transform을 계산한다.

### 결과 확인하기
이번에는 고급 시간여행 API으로 시뮬레이션을 다시 실행해 보자.

```bash
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py
```

![](https://docs.ros.org/en/humble/_images/turtlesim_delay2.png)

이제, 두 번째 turtle은 5초 전에 첫 번째 carrot이 있었던 곳으로 향한다.

## 요약
이 튜토리얼에서는 tf2의 고급 기능 중 하나를 살펴보았다. tf2가 데이터를 시간에 관련하여 transform할 수 있다는 것을 배웠고, turtlesim 예제를 통해 이를 수행하는 방법을 배웠다. tf2는 고급 lookup_transform_full() API를 통해, 과거로 돌아가 turtle의 과거와 현재 자세 간에 프레임 transform을 수행할 수 있게 해준다.
