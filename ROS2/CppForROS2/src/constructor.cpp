// g++ -std=c++17 constructor.cpp

#include <iostream>
using namespace std;

class MyClass {
public:
    MyClass():
        _x(10) { //생성자

    }
    int _x;
};

int main(){

    auto a = MyClass();
    cout<<a._x<<endl;
    return 0;
}