# [plugins 생성 및 사용](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
1. 목표
2. 배경지식
3. 사전준비
4. 실습
   1. Base Class Package 생성하기
   2. Plugin Package 생성하기
       1. plugins을 위한 소스 코드
       2. Plugin Declaration XML
       3. CMake Plugin Declaration
   3. Plugins 사용하기
   4. 빌드 및 실행
5. 요약

## 목표
* pluginlib를 사용하여 간단한 plugin을 생성하고 load하는 방법 배우기
## 배경지식
* 이 튜터리얼은 아래 링크를 기반으로 작성
  * http://wiki.ros.org/pluginlib
  * http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin
* pluginlib는 
  * C++ library
  * ROS package 내에 있는 plugins를 load/unload 수행
* plugin이란
  * 동적으로 로딩 가능한 class
  * runtime library에서 로딩(형태 : shared object, dynamically linked library)
* pluginlib 사용시 장점
  * application에서 해당 코드를 포함하지 않아도 된다.
  * 해당 pluginlib에 대한 정보를 많이 알고 있지 않아도 가능 
  * application의 소스코드 변경 없이 기능 확장 가능

## 사전준비
* 이 튜터리얼은 기본 C++에 대한 배경 지식이 있다고 가정한다.
* pluginlib 가 설치되어 있어야 한다.  
```
sudo apt-get install ros-humble-pluginlib
```

## 실습
* 이 튜터리얼에서 2개의 새로운 packages를 생성한다. 
   1. base class를 정의하는 package
   2. plugins을 제공하는 package
* base class는 generic polygon class를 정의하고 plugins는 특정 shapes을 정의한다.
* base class를 가져와서 상속받아서 사용하는 예제이다.

### 1. Base Class Package 생성하기
* ros2_ws/src 디렉토리 내에 빈 package를 생성하자.
* 다음 명령 실행
```
ros2 pkg create --build-type ament_cmake polygon_base --dependencies pluginlib --node-name area_node
```
* 편집기로 ros2_ws/src/polygon_base/include/polygon_base/regular_polygon.hpp 파일을 열고 붙여넣기
```c++
#ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
#define POLYGON_BASE_REGULAR_POLYGON_HPP

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // POLYGON_BASE_REGULAR_POLYGON_HPP
```

* 다른 class 파일에서 이 헤더 파일을 사용할수 있게 만들어야 한다. 
* ros2_ws/src/polygon_base/CMakeLists.txt 파일을 열고 ament_target_dependencies 명령 뒤에 다음 내용을 추가하자.
```
install(
  DIRECTORY include/
  DESTINATION include
)
```

* ament_package 명령 전에 이 명령을 추가하자.
```
ament_export_include_directories(
  include
)
```

* 나중에 이 package로 돌아와서 test node를 작성할 예정이다.

### 2. Plugin Package 생성하기
* 이제 추상 class로부터 상속받아 구현을 해보자. 2번째 package를 생성해보자. 
* ros2_ws로 이동하여 아래 명령을 실행한다.
```
ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins
```

### 2.1. plugins을 위한 소스 코드
* ros2_ws/src/polygon_plugins/src/polygon_plugins.cpp 파일 열고 아래 내용을 붙여넣자.
```c++
#include <polygon_base/regular_polygon.hpp>
#include <cmath>

namespace polygon_plugins
{
  class Square : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        return side_length_ * side_length_;
      }

    protected:
      double side_length_;
  };

  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        return 0.5 * side_length_ * getHeight();
      }

      double getHeight()
      {
        return sqrt((side_length_ * side_length_) - ((side_le/Pythongth_ / 2) * (side_length_ / 2)));
      }

    protected:
      double side_length_;
  };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
```

* PLUGINLIB_EXPORT_CLASS macro 에 대한 인자 살펴보기
   1. 여기서는 plugin class의 자격을 갖는 type으로 : polygon_plugins::Square
   2. 여기서는 base class의 자격을 갖는 type으로 : polygon_base::RegularPolygon

### 2.2. Plugin Declaration XML
* plugin loader가 해당 library를 찾는 방법을 제공해야만 한다.
* XML로 작성해 두면 ROS 빌드 툴에서 작업을 자동으로 해준다.

* ros2_ws/src/polygon_plugins/plugins.xml 파일을 생성한다.
```xml
<library path="polygon_plugins">
  <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
    <description>This is a square plugin.</description>
  </class>
  <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
    <description>This is a triangle plugin.</description>
  </class>
</library>
```

* 주의해야할 2가지
   1. library tag
     * export시키고자 하는 plugin을 포함하고 있는 libary에 대한 상대 경로를 libary에제 준다. ROS2에서 그냥 library의 이름일 뿐이다. 
   2. class tag
     * 우리 library로부터 export하고자하는 plugin을 선언한다. 이 작업을 parameters를 통해 해보자.
       * type : plugin의 type으로 여기서는  polygon_plugins::Square
       * description : plugin 설명으로 무엇을 하는 것인지 
       * name : name 속성으로 필요없음.

### 2.3. CMake Plugin Declaration
* 마지막 단계로 plugin을 export 시켜야한다. CMakeLists.txt 를 사용한다.
* ros2_ws/src/polygon_plugins/CMakeLists.txt 파일 내에서 find_package(pluginlib REQUIRED) 바로 뒤에 아래 코드를 붙여넣자.
```cmake
add_library(polygon_plugins src/polygon_plugins.cpp)
target_include_directories(polygon_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  polygon_plugins
  polygon_base
  pluginlib
)

pluginlib_export_plugin_description_file(polygon_base plugins.xml)

install(
  TARGETS polygon_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

* ament_package 명령  전에 다음을 추가하자.
```
ament_export_libraries(
  polygon_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)
```

* CMake 명령에 대한 인자들은 :
   1. base class를 위한 package (ex) polygon_base)
   2. Plugin Declaration xml에 대한 상대 경로(ex) plugins.xml)

### 3. Plugins 사용하기
* 이제 plugin을 사용할 차례이다.
* 이 작업은 어느 package에서든 가능하지만 여기서는 base package에서 해보자. 
* ros2_ws/src/polygon_base/src/area_node.cpp 파일을 다음과 같이 추가한다.

```c++
#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

  try
  {
    std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
```

* ClassLoader가 핵심 class인데 이것은 class_loader.hpp 헤더에 정의되어 있다.
  * base class로 template이 된다. (polygon_base::RegularPolygon)
  * 첫번째 인자 : base class의 package 이름 (polygon_base)
  * 두번쨰 인자 : plugin을 위한 fully qualified base class type의 string (polygon_base::RegularPolygon)

* class의 instance를 인스턴스화 시키는 여러 가지 방법이 있다. 이 예제에서는 shared pointers를 사용한다. createSharedInstance를 호출할때 plugin class 의 type을 사용해서 호출하면 된다. 이 경우에는 polygon_plugins::Square 이 된다.

* 중요 !!!
  * 이 node가 정의되어 있는 polygon_base package는 polygon_plugins class에 의존성을 가지 않는다. plugins는 선언에 대한 의존성 없이 동적으로 load된다. 직접 plugin 이름으로 해당 class를 인스턴스화시켰지만 동적으로 parameters로도 가능하다. 
### 4. 빌드 및 실행
* ros2_ws로 이동하여 아래 명령 수행한다.
```
colcon build --packages-select polygon_base polygon_plugins
```

* ros2_ws에서 setup파일을 source한다.
```
source install/setup.bash
```

* node를 실행한다.
```
ros2 run polygon_base area_node
```

* 다음과 같이 출력되어야 한다.
```
Triangle area: 43.30
Square area: 100.00
```

## 요약
* 첫번째 plugins 작성 및 사용