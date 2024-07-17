# Tuning
* 목차
  1. 개요
  2. Robot Footprint vs. Radius
  3. Plannner Plugin 선택
  4. Controller Plugin 선택
  5. Nav2 lauch 옵션


## 1. 개요
## 2. Robot Footprint vs. Radius
## 3. Plannner Plugin 선택
## 4. Controller Plugin 선택
* [Navigation plugins 설정 방법](https://navigation.ros.org/setup_guides/algorithm/select_algorithm.html#select-algorithm)

* plugin인별 특징 이해
| Plugin 이름 | 지원 robot 타입 | Task |
|:---:|:---:|:---:|
| DWB controller | differential, omnidirectional | 동적 장애물 회피 |
| MPPI Controller | differential, omnidirectional, Ackermann, Legged | 동적 장애물 회피 |
| RPP controller | differential, Ackermann, Legged | 정확한 path follow |
| Rotation Shim | differential, omnidirectional | 대략적인 heading 회전 |


## 5. Nav2 launch 옵션
* 