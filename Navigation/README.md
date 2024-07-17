# [Nav2](https://navigation.ros.org/)
* 목표 : Nav2 아키텍쳐 이해

![](https://navigation.ros.org/_images/nav2_architecture.png)

* 그림 읽는 방법
  * 그림 해석해보기

[![Video](http://img.youtube.com/vi/QB7lOKp3ZDQ/0.jpg)](http://www.youtube.com/watch?v=QB7lOKp3ZDQ)


* [navigation2 git repo](https://github.com/ros-planning/navigation2)
  * 각 부분 살펴보기
* [개발문서](https://github.com/ros-planning/navigation2/tree/main/doc)
  * [개요 문서](https://github.com/ros-planning/navigation2/blob/main/doc/design/Navigation_2_Overview.pdf)


## 구성
1. BT Navigator Server
2. Servers : Controller, Planner, Behavior, Smoother, Route
3. Plugins
4. Lifecycle Manager

## 1. BT Navigator Server

## 2. Servers

## 3. Plugins
* [Nav2 Plugins 목록](https://navigation.ros.org/plugins/index.html)

## 4. Lifecycle Manager
| State | Description | Transitions |
| --- | --- | --- |
| Unconfigured | The node is not configured. | configure |
| Inactive | The node is configured but not active. | activate, cleanup |
| Active | The node is active. | deactivate, cleanup |
| Finalized | The node is inactive and cannot be reactivated. | cleanup |

