// g++ bind.cpp

/*
std::bind는 C++ 표준 라이브러리(STL)의 일부로,
 함수나 멤버 함수를 특정 객체 또는 값에 바인딩하는 데 사용됩니다. 

 쉽게 말해, 함수의 일부 인수를 미리 설정하여 새로운 함수를 만드는 기능을 제공합니다. 
 이렇게 만들어진 새로운 함수는 일반 함수처럼 호출할 수 있으며, 미리 설정되지 않은 인수만 전달하면 됩니다.
*/

#include <iostream>
#include <functional>

int add(int a, int b, int c) {
  return a + b + c;
}

/*
int add345(){
    return add(3, 4, 5);
}

int add5(int a, int b){
    return add(5, a, b);
}

*/


int main() {
  // 3, 4, 5를 더하는 함수 객체를 만듭니다.
  // add 함수에 3, 4, 5를 바인딩시켜서 이것을 add345 변수에 할당
  auto add345 = std::bind(add, 3, 4, 5);

   // add345 함수 객체를 호출합니다.
   int result = add345();
   std::cout << result << std::endl; // 7 출력

   // add 함수의 첫 번째 인수를 5로 미리 설정하고, 2번째, 3번째 인수는 함수 객체가 전달하도록 하는 함수 객체를 만듭니다.
   auto add5 = std::bind(add, 5, std::placeholders::_1, std::placeholders::_2);
   // add5 함수 객체를 10, 11과 함께 호출합니다.
   result = add5(10, 11);
   std::cout << result << std::endl; // 15 출력

  return 0;
}
