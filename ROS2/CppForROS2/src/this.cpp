// g++ -std=c++17 this.cpp

#include <iostream>
using namespace std;

class MyClass {
public:
    MyClass():
        _x(13) { //생성자

    }
    int _x;
    void printValue(){
        cout<<this->_x<<endl;
    }
};

int main(){
    auto m = MyClass();

    m.printValue();
    return 0;
}

