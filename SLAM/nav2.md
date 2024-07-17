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
* [Nav2 튜닝 가이드](https://navigation.ros.org/tuning/index.html#)
