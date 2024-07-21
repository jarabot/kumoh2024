# Navigation 2
* [Nav2 홈페이지](https://navigation.ros.org/index.html)

* 목표
  * 역할
  * 아키텍쳐
  * 튜닝

## 역할
* A지점에서 B지점으로 이동
* object following

## 아키텍쳐

![](https://navigation.ros.org/_images/nav2_architecture.png)

* 분류
    * map
        * Map Server
        * Nav2 Costmap
    * localize
        * AMCL
    * planner
        * Nav2 Planner
        * Nav2 Smoother
    * controller
        * Velocity Smoother
    * behavior
        * Nav2 Behavior
        * BT Navigator
        * Nav2 Recoveries
        * Collision Monitor
    * manage servers
        * Nav2 Lifecycle Manager
        * Nav2 Core
    * API
        * Simple Commander

## 튜닝
* [Nav2 튜닝 가이드](https://docs.nav2.org/tuning/index.html)


* 방법
   1. Inflation Potential Fields
      * 로봇 주변 공간에 가상의 fields를 만들어 로봇이 장애물을 피하도록 돕는 핵심 기능
   2. Robot Footprint vs Radius
      * footprint란 로봇의 형태에 따라 차지하는 공간
      * Radius는 로봇 주변 원형 반경 
   3. Rotate in Place Behavior
      * 현재 위치에서 회전하여 방향을 설정
      * 로봇이 새로운 경로를 시작하기 전에 현재 진행 방향을 목표 방향으로 조정하는 데 사용되는 기능입니다. 로봇이 급격한 방향 전환을 수행하기 어려운 경우, Rotate in Place Behavior를 사용하여 로봇을 제자리에서 회전시킨 후 부드럽게 경로를 따라 이동하도록 할 수 있습니다.
   1. Planner Plugin Selection
      * 외부에서 다양한 planning plugins 받아서 사용 가능
| Plugin 이름  |  지원하는 Robot 타입  |
|---|---|
| NavFn Planner  | Circular Differential, Circular Omnidirectional  |
| Smac Planner 2D  |  Circular Differential, Circular Omnidirectional |
| Theta Star Planner  |  Circular Differential, Circular Omnidirectional |
| Smac Hybrid-A* Planner  | Non-circular or Circular Ackermann, Non-circular or Circular Legged  |
| Smac Lattice Planner  |  Non-circular Differential, Non-circular Omnidirectional, Arbitrary |

   1. Controller Plugin Selection
      *  
   2. Caching Obstacle Heuristic in Smac Planners
      * Smac Planners에서 Caching Obstacle Heuristic는 로봇의 경로 계획 속도를 향상시키는 데 사용되는 기술입니다. 이 기술은 이전에 계산된 경로 정보를 캐시하여 동일한 목표 지점으로의 반복적인 경로 계획 작업을 효율화합니다.
      * 동작 방식
         1. 로봇이 새로운 목표 지점을 받습니다.
         2. Smac Planner는 목표 지점까지의 경로를 계산합니다.
         3. 계산된 경로 정보는 캐시에 저장됩니다.
         4. 로봇이 동일한 목표 지점으로 다시 이동해야 하는 경우 Smac Planner는 캐시에서 이전에 계산된 경로 정보를 재사용합니다.
         5. 캐시에 저장된 경로 정보가 최신 정보와 일치하는지 확인합니다. 일치하지 않으면 새로운 경로를 계산하고 캐시를 업데이트합니다.
      * 설정 파라미터
         * cache 크기
         * cache timeout 
   3.  Costmap2D Plugins
      * dd 
   4.  Nav2 Launch Options
```
slam: 로컬라이제이션 및 매핑에 AMCL 또는 SLAM 툴박스를 사용할지 여부를 설정합니다. 기본값은 AMCL이며, false로 설정하면 SLAM 툴박스를 사용합니다.
map: 탐색에 사용할 지도 파일의 경로입니다. 기본값은 패키지의 maps 디렉토리 내 map.yaml 파일입니다.
world: 시뮬레이션에서 사용할 world 파일의 경로입니다. 기본값은 패키지의 worlds 디렉토리입니다.
params_file: 주요 탐색 설정 파일입니다. 기본값은 패키지의 params 디렉토리 내 nav2_params.yaml 파일입니다.
autostart: 탐색 시스템의 라이프사이클 관리 시스템을 자동으로 시작할지 여부를 설정합니다. 기본값은 true이며, Nav2 스택을 생성 시 활성화 상태로 전환하여 사용 가능하도록 준비합니다.
use_composition: 각 Nav2 서버를 개별 프로세스 또는 단일 구성 노드로 실행할지 여부를 설정합니다. CPU 및 메모리 절약을 위해 단일 프로세스 Nav2를 사용하는 것이 기본값인 true입니다.
use_respawn: 충돌로 인해 종료된 서버를 자동으로 다시 시작할지 여부를 설정합니다. 라이프사이클 관리자와 함께 구성된 경우, 관리자는 이미 활성화되어 충돌로 인해 종료된 시스템을 다시 백업합니다. 모든 노드가 동일한 프로세스/컨테이너에 있기 때문에 단일 구성 방식의 bringup에서는 작동하지 않습니다.
use_sim_time: 시뮬레이션에서 필요한 모든 노드가 시뮬레이션 시간을 사용하도록 설정할지 여부를 설정합니다. 시뮬레이션의 경우 기본값은 true입니다.
rviz_config_file: 사용할 rviz 구성 파일의 경로입니다. 기본값은 rviz 디렉토리의 파일입니다.
use_simulator: Nav2 스택과 함께 Gazebo 시뮬레이터를 시작할지 여부를 설정합니다. 기본값은 true이며, Gazebo를 실행합니다.
use_robot_state_pub: 로봇의 URDF 변환을 TF2에 게시하는 로봇 상태 게시자를 시작할지 여부를 설정합니다. 기본값은 true이며, 로봇의 TF2 변환을 게시합니다.
use_rviz: 시각화를 위해 rviz를 실행할지 여부를 설정합니다. 기본값은 true이며, rviz를 표시합니다.
headless: 백그라운드 Gazebo 시뮬레이션과 함께 Gazebo 프런트엔드를 실행할지 여부를 설정합니다. 기본값은 true이며, Gazebo 창을 표시합니다.
namespace: 필요한 경우 로봇을 실행할 네임스페이스입니다.
use_namespace: 로봇을 이 네임스페이스로 실행할지 여부를 설정합니다. 기본값은 false이며, 단일 로봇에 대해 전역 네임스페이스를 사용합니다.
robot_name: 실행할 로봇의 이름입니다.
robot_sdf: Gazebo 플러그인과 로봇 시스템 시뮬레이션 설정을 포함하는 로봇의 Gazebo 구성 파일 경로입니다.
x_pose, y_pose, z_pose, roll, pitch, yaw: 시뮬레이션에서 로봇의 초기 위치를 설정하는 매개 변수입니다.
```