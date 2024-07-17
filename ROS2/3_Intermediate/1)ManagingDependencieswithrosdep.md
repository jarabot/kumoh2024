# [rosdep로 의존성 관리하기](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) (5min)

## 목표
* rosdep을 사용하여 외부 의존성을 관리하는 방법을 배워보자.

## 목차 
1. rosdep 이란
2. package.xml 파일 소개
3. rosdep 동작 방식
4. package.xml에 어떤 키를 넣어야 되는지 알 수 있는 방법
5. 내가 작성한 library가 rosdisto에 없는 경우
6. rosdep 도구 사용법


### 1. rosdep 이란?
* rosdep은 ros+dependncies의 줄임말
* ROS의 의존성 관리 도구
  * ROS package와 외부 library를 사용하여 동작할 수 있도록 돕는 도구
  * command line 도구
  * 내가 작성한 package를 빌드하고 install하기 위해서 관련 의존성(dependencies)를 찾아서 설치
* rosdep 사용하는 경우
  - workspace를 빌드할때와 우리가 작성한 packages를 빌드할때 필요로 하는 의존성
  - 실행에 필요한 의존성인지 확인하고 packages 설치 (예: sudo apt install ros-humble-demo-nodes-cpp )
  - 기타

* 하나의 package에 대해서도 사용가능
* packages가 많을 때는 packages를 포함하고 있는 workspace에 대해서 사용할 수 있다.

### 2. package.xml 파일 소개
* package.xml 파일은 package가 필요로 하는 의존성 정보를 정의하고 있다.
* package.xml 파일 내부에 있는 의존성을 "rosdep keys"라고 부른다.
* 이 keys들은 <depend>, <test_depend>, <exec_depend>, <build_depend>, <build_export_depend> 태그 내에 존재

* tag 종류와 의미
  - <test_depend> : code를 테스트 하는 경우에 사용
  - <build_depend> : code를 빌드 하는 경우에 사용
  - <build_export_depend> : code가 export하는 헤더가 필요로 하는 경우 
  - <exec_depend> : code 실행시 사용
  - <depend> : 빌드, export, 실행 시간 등 복합적인 목적을 위해 사용 

* 이러한 의존성은 package를 생성하는 사람이 package.xml 파일에 수동으로 입력한다.
* 우리 package가 빌드와 실행에 필요로 하는 모든 libraries와 packages에 대한 정보를 입력해야한다.

### 3. rosdep 동작 방식
* rosdep은 package.xml을 검사한다.
  * package.xml 파일내에 rosdep keys를 찾는다.
  * 이 keys는 여러 가지 package managers 내에서 필요한 ROS package나 SW library를 찾는 핵심 index가 된다.(즉 key를 가지고 ROS package나 library 찾기)
* 마지막으로 해당 package를 찾게 되면 설치하여 빌드나 실행을 할 수 있게 된다.

* 중신이되는 index를 rosdistro라고 한다. [링크](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) 에서 humble rosdistro에서 package를 찾으면 설치하기 위한 정보를 포함하고 있다.

### 4. package.xml에 어떤 키를 넣어야 되는지 알 수 있는 방법
* 좋은 질문!!
* ROS packages인 경우(예제로 nav2_bt_navigator)
  * 그냥 package 이름을 입력하면 된다. 배포된 ROS packages의 모든 목록은 [rosdistro](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml) 내부에서 확인할 수 있다.

* ROS packages가 아닌 경우 
  * 특정 library에 대한 keys를 찾아야 된다.
  * 일반적으로 2개 파일이 있다.
    * [rosdep/base.yaml](https://github.com/ros/rosdistro/blob/humble/2022-12-16/rosdep/base.yaml)
      * apt 시스템 의존성
    * [rosdep/python.yaml](https://github.com/ros/rosdistro/blob/humble/2022-12-16/rosdep/python.yaml)
      * pip Python 의존성

* key를 찾기 위해서 이 파일 내부에서 library를 검색하고 이 key를 포함하는 yaml 내에서 이름을 찾는다. 이 key를 package.xml 파일에 입력한다.

* 예제로 doxygen에 의존성을 갖는 내 package를 만들었다고 하자. doxygen은 개발문서를 만드는데 사용하는 SW이다. 이 경우에 base.yaml에서 doxygen을 찾게 된다.

  * rosdistro의 <distro>/distribution.yaml 파일에서 해당 패포판에서 제공하는 모든 ROS packages의 목록을 볼 수 있다.
* ROS package가 아닌 system 의존성
  * 특정 library에 대한 key를 찾아야 한다.
  * 일반적으로 2개 파일 : rosdep/base.yaml, rosdep/python.yaml
  * 일번적으로 base.yaml 내부에 apt 시스템 의존성을 포함하고 있다.
  * python.yaml은 pip python 의존성을 포함하고 있다.

* key를 찾는 방법
  * 이 파일 내부에서 lib를 찾고 yaml 내부에서 이름 찾기
  * package.xml 파일 내에 이 key를 넣기
* 예제로 doxygen에 의존성을 가지는 package가 있다고 가정
  * 문서와 관련된 sw일 것이기 때문에 
  * doxygen에 대해서 [base.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml#L829)를 찾으면
```yaml
doxygen:
  arch: [doxygen]
  debian: [doxygen]
  fedora: [doxygen]
  freebsd: [doxygen]
  gentoo: [app-doc/doxygen]
  macports: [doxygen]
  nixos: [doxygen]
  openembedded: [doxygen@meta-oe]
  opensuse: [doxygen]
  rhel: [doxygen]
  ubuntu: [doxygen]
```
* 즉 우리의 rosdep는 doxygen이 되는데 OS마다 다른 package managers 내에서 찾게 된다.
예를 들어 doxygen 의존성인 경우, 위 형태로 Linux 버전별 rosdep key가 정의되어 있고 우리는 ubuntu에 해당되는 것을 사용한다.
[링크](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml#L829)

### 5. 내 라이브러리가 rosdistro에 없는 경우
* 만약 내가 개발한 library가 rosdistro 내부에 없다면, 오픈소스 sw 개발의 위대함을 경험하게 될 것이다. (내가 개발한 library를 rosdistro에 넣는 방법)
  * 우리가 그냥 직접 추가하면 된다!
  * rosdistro에 대해서 git PR하면 일주일 내에 merge된다.
* rosdep keys 개발에 contributor 되는 방법 [링크](https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md#rosdep-rules-contributions)

* [rosdep/base.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml) or [rosdep/python.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml)파일에서 해당 rosdep key를 찾는다 

### 6. rosdep 도구 사용하는 방법
* 이제 rosdep, package.xml, rosdistro에 대해서 좀 알게 되었으니 직접 사용해 보자.
* 맨 먼저 rosdep을 사용하므로 아래와 같이 초기화를 해야만 한다. 
```bash
# rosdep 초기화 
sudo rosdep init
# 업데이트시 로컬 캐시의 rosdistro 인덱스 업데이트 됨 (자주자주 업데이트 해주자!)
rosdep update
# 설치 모든 코드가 포함된 디렉터리인 src에서 실행
rosdep install --from-paths src -y --ignore-src
```
* 마지막으로 rosdep install을 실행하여 의존성을 설치한다. 일반적으로 workspace 에서 한번 실행하면 workspace가 포함하고 있는 packages이 필요로하는 모든 의존성을 설치하게 된다.
* workspace 아래로 이동하여 아래와 같이 명령을 수행하면 : 
```
rosdep install --from-paths src -y --ignore-src
```
* 옵션 의미
  * --from-paths src
    * package.xml 파일을 검사하기 위한 path 지정. 즉 ./src 디렉토리를 살펴보라는 의미 
  * -y
    * 모든 prompts에 대해서 기본적으로 yes를 의미. 즉 package manager가 각 설치마다 prompts로 물어보는 것에 대해서 그냥 yes로 하고 다음 설치를 넘어가라는 의미
  * --ignore-src
    * 해당 package가 workspace 내부에 있는 경우 dependencies 설치 하지말라는 의미.  

* rosdep 관련 옵션이나 설명을 보려면 (help)
```
rosdep -h
```