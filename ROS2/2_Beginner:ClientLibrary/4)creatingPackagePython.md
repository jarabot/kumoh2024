# package 생성하기 (Python)
1. 개요
2. 실습
   1. package 생성하기
   2. package 빌드하기
   3. setup 파일 source하기
   4. package 사용하기
   5. package contents 살펴보기
   6. package.xml 수정하기
   
## 1. 개요
* 새로운 package 생성하기
* package란?
  * ROS 2 코드를 위한 구성 단위
  * ROS 2에서 code를 설치하거나 공유하기 위해서는 하나의 package로 구성해야함
  * package로 구성하면 배포 가능하고 다른 사람들이 빌드해서 쉽게 사용이 가능
* ROS 2 package 구성
```
package.xml : package에 대한 meta 정보
resource/<package_name> marker file
setup.cfg : 실행자로 사용될때 필요한 파일
setup.py : package를 설치하는 방법 지시
<package_name> 디렉토리 : ros2 도구로 package 찾을때 필요
```

* 간단한 package 구성
```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
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
ros2 pkg create --build-type ament_python <package_name>
```

* 아래 명령으로 package 생성
```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

* 실행 결과 (자동생성)
```
going to create a new package
package name: my_package
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['<name> <email>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: []
node_name: my_node
creating folder ./my_package
creating ./my_package/package.xml
creating source folder
creating folder ./my_package/my_package
creating ./my_package/setup.py
creating ./my_package/setup.cfg
creating folder ./my_package/resource
creating ./my_package/resource/my_package
creating ./my_package/my_package/__init__.py
creating folder ./my_package/test
creating ./my_package/test/test_copyright.py
creating ./my_package/test/test_flake8.py
creating ./my_package/test/test_pep257.py
creating ./my_package/my_package/my_node.py
```

### 2-2. package 빌드하기
* workspace의 폴더로 이동
```bash
cd ~/ros2_ws
```

* package 빌드하기
```bash
colcon build
```

* 특정 package만 빌드하기 (my_package)
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
Hi from my_package.
```

### 2-5 package contents 살펴보기
* ros2_ws/src/my_package 내부
```
my_package  package.xml  resource  setup.cfg  setup.py  test
```
  * my_node.py는 my_package 디렉토리 내부에 존재

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

 <test_depend>ament_copyright</test_depend>
 <test_depend>ament_flake8</test_depend>
 <test_depend>ament_pep257</test_depend>
 <test_depend>python3-pytest</test_depend>

 <export>
   <build_type>ament_python</build_type>
 </export>
</package>
```

* setup.py (package.xml의 maintainer, maintainer_email, description 필드 동일하게 수정)
```python
from setuptools import setup

package_name = 'my_py_pkg'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'my_node = my_py_pkg.my_node:main'
     ],
   },
)
```