# [Transformation](https://navigation.ros.org/setup_guides/transformation/setup_transforms.html)

## Transformation 변환하기
이 가이드에서는, Nav2에 필요한 transform을 살펴볼 것이다. 이 transform들은 센서나 odometry와 같은 여러 소스들로부터 오는 정보를 사용할 좌표 프레임으로 transform해서, 이 정보를 해석할 수 있도록 해준다. 아래 그림은 전체 transform 트리가 어떻게 보이는지를 보여준다. 하지만 이 가이드에서는 더 간단한 것으로 시작할 것이다.

![](https://navigation.ros.org/_images/tf_full_tree.png)

![frame](https://qph.cf2.quoracdn.net/main-qimg-004d4b657e9ba128c0a3eb4f708533e5)
이 튜토리얼을 위해, 먼저 ROS의 transform에 대한 간단한 설명을 제공할 예정이다. 그 다음으로, TF2 static publisher의 명령줄 데모를 통해 이것이 실제로 작동하는지를 볼 것이다. 마지막으로, Nav2가 제대로 동작하기 위해 필요한 transform의 윤곽을 그려볼 것이다.

## Transform 소개
많은 ROS 패키지들은 TF2 ROS 패키지로 publish될 로봇의 transform 트리를 필요로 한다. transformation 트리는 병진이동, 회전, 상대적인 움직임의 관점에서 서로다른 두 좌표 프레임의 관계를 정의한다. 이것을 더 정확하게 하기 위해 우리는 레이저 센서가 하나 달려있는 간단한 모바일 로봇을 예로 들 것이다.

이 로봇은 두개의 좌표 프레임을 가진다: 하나는 로봇의 차체 중심과 일치하는 좌표 프레, 그리고 하나는 차체 상단에 부착된 레이저의 중심에 대한 좌표 프레임이다. 우리는 차체에 있는 프레임을 base_link라 부르고, 레이저에 있는 프레임을 base_laser라 부를 것이다. 이 프레임들에 대한 naming과 convention들은 다음 섹션에서 이야기 할 것이다.

이제 레이저 중심에서부터의 거리 데이터를 가지고 있다고 가정해 보자. 즉, base_laser 좌표 프레임에서의 데이터를 가지고 있는 것이다.

차체가 이 데이터를 가지고 장애물을 회피해야 한다고 해보자. 이것이 성공하려면, 우리가 받은 base_laser로부터의 데이터는 base_link 좌료 프레임으로 transform 하는 방법이 필요하다. 따라서, 개념적으로 base_laser와 base_link 좌표 프레임 사이의 관계를 정의할 필요가 있다.

![](https://navigation.ros.org/_images/simple_robot.png)

이런 관계를 정의할 때에, 차체 중심에서 전방 10cm, 상단 20cm에 laser가 설치되어 있다고 가정해보자. 이는 base_link를 base_link에 관련짓는 오프셋으로 말할 수 있다. 특히, base_link 프레임에서 base_laser프레임으로 데이터를 가져오기 위해서는 (x: 0.1m, y:0.0m, z:0.2m)의 이동이 있어야 하고 반대로 base_laser에서 base_link 프레임으로 데이터를 가져오기 위해서는 (x:-0.1m, y:0.0m, z:-0.2m)인 반대의 이동이 적용되어야 한다는 것을 알 수 있다.

이러한 관계를 저장해놓고, 필요할때 마다 두 프레임 사이의 적절한 변환을 적용할 수도 있지만, 좌표 프레임의 수가 늘어난다면, 실로 고통스러운 작업이 될 것이다. 다행히도, 우리는 이런 작업을 할 필요가 없다. 대신, 우리는 TF2를 이용해서 base_link와 base_laser의 관계를 한번만 정의하고, TF2가 관리하도록 할 것이다. 이러한 방법은 특히, map에서 차체처럼 같이 지속적으로 움직이는 프레임들의 집합, 즉 비 고정 transformation을 이용할 때 유용하다. TF2를 이용해서 transformation을 정의하고 저장하기 위해선, 이를 transform 트리에 추가할 필요가 있다. 개념적으로는, 각 노드는 좌표 프레임에 해당하고, 각 엣지는, 현재 노드에서 자식 노드로 적용이 필요한 transform에 해당한다. TF2가 트리 구조를 사용하는 이유는, 어느 두 프레임이든 단 하나의 트리 순회가 존재한다는 것을 보장하고, 트리의 모든 엣지가 부모에서 자식 노드로 향한다는 것을 가정하기 위해서이다.

![](https://navigation.ros.org/_images/tf_robot.png)

위의 간단한 예제에 대한 transform트리를 만들기 위해서, 우리는 두개의 노드를 생성할 것이다: 하나는 base_link좌표 프레임이며, 하나는 base_laser좌표 프레임이다. 둘 사이의 엣지를 생성하기 위해, 먼저 어떤 노드가 부모가 되고, 또 자식이 될지 결정해야 한다. TF2는 모든 transform이 부모에서 자식으로만 이동한다고 가정하기 때문에 이러한 구분이 중요하다는 것을 잊지 말자.

base_link 좌표 프레임을 부모로 선택해 보자. 왜냐하면, 로봇에 다른 센서나 부품이 추가 될 때, base_link를 통해 순회함으로써, base_link와 관련지어는 것이 가장 합리적이기 때문이다. 이는 base_link와 base_laser를 연결하는 엣지와 관련된 transform이 (x: 0.1m, y:0.0m, z:0.2m)가 되어야 한다는 것을 의미한다.

transform 트리가 설정되었으면, base_laser 프레임에서 받은 레이저 스캔을 base_link 프레임으로 변환하는 것은 TF2 library만 호출하면 되므로 간단해진다. 우리 로봇은 이제 base_link에서 이 레이저 스캔을 추론하기 위해 transform 정보를 사용할 수 있고, 주변 환경에서 장애물을 회피하는 경로 계획을 안전하게 세울 수 있다.