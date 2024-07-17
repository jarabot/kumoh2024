# [쿼터니언 기초](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
1. 목표
1. 배경지식
1. 사전준비
1. 쿼터니언 구성요소
1. 쿼터니언 연산
1. ROS2 쿼터니언 타입
1. 요약

## 목표
* 쿼터니언 사용법 기초 학습하기

## 배경지식
쿼터니언은 rotation matrix보다 간결한, 방향에 대한 4-튜플 표현이다. 쿼터니언은 3차원 상에서의 회전이 포함된 상황을 분석할 때 매우 효율적이다. 또한 로보틱스, 양자역학, 컴퓨터 비전, 3D 애니메이션 등에 널리 활용된다.

위키피디아를 참고하면 기본 수학적 개념에 대해 배울 수 있다. 또한 유튜브 채널 3blue1brown에서 제작한 Visualizing quaternions 영상을 참고하자. 

이번 튜토리얼에서는, 쿼터니언과 변환 방법들이 어떻게 작동하는지 학습할 것이다.

## 사전준비
하지만, 이것은 필수 사항이 아니며, 다른 적합한 기하학 변환 라이브러리를 사용해도 된다. 예를 들면, transform3d, scipy.spatial.transform., pytransform3d, numpy-quaternion, blender.mathutils 등이 있다.

### 쿼터니언 구성요소
ROS2는 회전을 추적하고 적용하기 위해 쿼터니언을 사용한다. 쿼터니언은 4개의 요소 (x, y, z, w)를 가진다. ROS2에서는 w가 마지막에 위치하지만, Eigen과 같은 다른 라이브러리에서는 첫 번째에 위치할 수도 있다. x/y/z 축에 대해 회전이 없다는 것을 의미하는 단위 쿼터니언은 (0, 0, 0, 1)이다. 다음 방식으로 생성할 수 있다.

```cpp
#include <tf2/LinearMath/Quaternion.h>
...

tf2::Quaternion q;
// Create a quaternion from roll/pitch/yaw in radians (0, 0, 0)
q.setRPY(0, 0, 0);
// Print the quaternion components (0, 0, 0, 1)
RCLCPP_INFO(this->get_logger(), "%f %f %f %f",
            q.x(), q.y(), q.z(), q.w());
```

쿼터니언의 크기는 항상 1이어야 한다. 만약 수치적 오차로 인해 쿼터니언의 크기가 1 이상이 된다면, ROS2는 warning을 출력할 것이다. 이러한 warning을 피하기 위해서는, 쿼터니언을 표준화 하자.

```cpp
q.normalize();
```

### ROS2 쿼터니언 타입
ROS2는 두 개의 쿼터니언 타입이 있다: tf2::Quaternion, geometry_msgs::msg::Quaternion. C++에서 둘 사이를 변환하려면, tf2_geometry_msgs의 메소드를 사용하면 된다.

C++
```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// ros2-foxy 에서는 위 헤더를 직접 다운로드 받아야 함
...

tf2::Quaternion tf2_quat, tf2_quat_from_msg;
tf2_quat.setRPY(roll, pitch, yaw);
// Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

// Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
tf2::convert(msg_quat, tf2_quat_from_msg);
// or
tf2::fromMsg(msg_quat, tf2_quat_from_msg);
```

Python
```python
from geometry_msgs.msg import Quaternion
...

# Create a list of floats, which is compatible with tf2
# Quaternion methods
quat_tf = [0.0, 1.0, 0.0, 0.0]

# Convert a list to geometry_msgs.msg.Quaternion
msg_quat = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
```

c++에서 사용하려면 CMakeLists.txt에 tf2_geometry_msgs에 대한 의존성을 추가해야 한다.
```cmake
find_package(tf2_geometry_msgs REQUIRED)
...

ament_target_dependencies(
    <target_name>
    tf2_geometry_msgs
    ...
)
```


## 쿼터니언 연산
### 1. RPY에서 생각하고 쿼터니언으로 변환하기
축의 회전에 관해 생각하는 것은 쉽지만, 쿼터니언의 관점에서 생각하기란 쉽지 않다. 그렇기 때문에, 롤(x축), 피치(y축), 요(z축)의 관점에서 먼저 대상의 회전을 계산한 다음 쿼터니언으로 변환하는 것이 더 낫다.

python
```python
# quaternion_from_euler method is available in turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py
q = quaternion_from_euler(1.5707, 0, -1.5707)
print(f'The quaternion representation is x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]}.')
```

### 2. 쿼터니언 회전 적용하기
쿼터니언 회전을 자세에 적용하기 위해서는 간단히 이전 자세의 쿼터니언에 원하는 회전을 표현하는 쿼터니언을 곱하기만 하면 된다. 단, 곱셈의 순서는 중요하다.

C++
```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
...

tf2::Quaternion q_orig, q_rot, q_new;

q_orig.setRPY(0.0, 0.0, 0.0);
// Rotate the previous pose by 180* about X
q_rot.setRPY(3.14159, 0.0, 0.0);
q_new = q_rot * q_orig;
q_new.normalize();
```

Python
```python
q_orig = quaternion_from_euler(0, 0, 0)
# Rotate the previous pose by 180* about X
q_rot = quaternion_from_euler(3.14159, 0, 0)
q_new = quaternion_multiply(q_rot, q_orig)
```

### 3. 쿼터니언 인버팅
쿼터니언을 인버팅 하는 쉬운 방법은 w에 음수로 바꿔수는 것이다.

```python
q[3] = -q[3]
```

### 4. Relative rotations
한 프레임에 q_1, q_2라는 두 쿼터니언이 있다. 이때 q_1을 q_2로 변환하는 relative rotation을 찾고 싶다면, 다음과 같은 방식으로 구할 수 있다.

```python
q_2 = q_r * q_1
```

이를 행렬방정식처럼 풀면, q_r을 구할 수 있다. q_1을 인버팅하고 양변에 오른쪽에 곱해준다. 여기서도 곱셈의 순서는 중요하다.

```python
q_r = q_2 * q_1_inverse
```

아래는 이전 로봇 자세에서 현재 로봇 자세로 가는 relative rotation을 구하는 파이썬 예제이다.

```python
def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion

q1_inv[0] = prev_pose.pose.orientation.x
q1_inv[1] = prev_pose.pose.orientation.y
q1_inv[2] = prev_pose.pose.orientation.z
q1_inv[3] = -prev_pose.pose.orientation.w # Negate for inverse

q2[0] = current_pose.pose.orientation.x
q2[1] = current_pose.pose.orientation.y
q2[2] = current_pose.pose.orientation.z
q2[3] = current_pose.pose.orientation.w

qr = quaternion_multiply(q2, q1_inv)
```

## 요약
이번 튜토리얼에서는 쿼터니언의 기본적인 개념과, inversion, rotation과 같은 수학적 연산에 관해 학습했다. 또한 ROS2에서의 사용예제와 두 쿼터니언 간에 변환 방법에 대해 학습했다.