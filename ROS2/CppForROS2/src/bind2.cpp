// g++ -std=c++17 bind2.cpp
// 함수의 인자를 고정하여 새로운 함수로 만들어서 변수처럼 사용

#include <iostream>
#include <functional>

int Calculate(int a, int b, int c)
{ 
    return (a + b) * c;
}

int main(int argc, char* argv[])
{ 
    // auto bindedCal = std::bind(Calculate, 2, 3, 4); 
    auto bindedCal = std::bind(Calculate, 2, std::placeholders::_1, std::placeholders::_2); 
    std::cout<<bindedCal(3, 4)<<std::endl; 
    return 0;
}