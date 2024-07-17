# package 생성하기 (C++)
1. 개요
2. 실습
   1. package 생성하기
   2. package 빌드하기
   3. setup 파일 source하기
   4. package 사용하기
   5. package contents 살펴보기
   6. 커스텀 package.xml

## 1. 개요
* 새로운 package 생성하기
* package란?
  * ROS 2 코드를 위한 구성 단위
  * ROS 2에서 code를 설치하거나 공유하기 위해서는 하나의 package로 구성해야함
  * package로 구성하면 배포 가능하고 다른 사람들이 빌드해서 쉽게 사용이 가능
* ROS 2 package 구성
```
CMakeLists.txt : package를 빌드하는 방법 
include/<package_name> directory : package 공용 헤더 파일
package.xml : package에 대한 meta 정보
src 디렉토리 : 소스 코드
```

* 간단한 package 구성
```
my_package/
     CMakeLists.txt
     include/my_package/
     package.xml
     src/
```

## 2. 실습
### 2-1. package 생성하기
* 이전에 생성한 workspace인 ros2_ws에 새로운 package를 생성하자.
* src 폴더로 이동하기 명령
```bash
cd ~/ros2_ws/src
```

* 새로운 package 생성하는 명령 형식
```bash
ros2 pkg create --build-type ament_cmake <package_name>
```

* 아래 명령으로 package 생성 명령 실행 (my_package)
```bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

* 실행 결과
```
going to create a new package
package name: my_package
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source and include folder
creating folder ./my_package/src
creating folder ./my_package/include/my_package
creating ./my_package/CMakeLists.txt
creating ./my_package/src/my_node.cpp
```
  * src 디렉토리 내부에 my_package라는 폴더 생성

### 2-2. package 빌드하기
* workspace의 폴더로 이동 명령 실행
```bash
cd ~/ros2_ws
```

* package 빌드 명령 실행
```bash
colcon build
```

* 특정 package만 빌드 명령 실행 (my_package)
```bash
colcon build --packages-select my_package
```

### 2-3. setup 파일 source하기
* workspace를 path에 추가하는 명령(빌드한 package를 실행 가능)
```bash
source install/local_setup.bash
```

### 2-4. package 사용하기
* --node-name 인자로 생성한 실행자를 실행하는 명령
```bash
ros2 run my_package my_node
```

* 결과
```
hello world my_package package
```

### 2-5 package contents 살펴보기
* ros2_ws/src/my_package 내부 살펴보기

```bash
CMakeLists.txt  include  package.xml  src
```

   * ros2 pkg create 명령으로 자동 생성된 것들
   * my_node.cpp 파일은 src 디렉토리 내부에 있음

### 2-6 package.xml 수정하기
* package.xml (수정 : description, license, maintainer 필드)
```xml
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <buildtool_depend>ament_cmake</buildtool_depend>

 <test_depend>ament_lint_auto</test_depend>
 <test_depend>ament_lint_common</test_depend>

 <export>
   <build_type>ament_cmake</build_type>
 </export>
</package>
```
  * 수정 후 저장하기!
