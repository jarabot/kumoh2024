// g++ -std=c++17 constructor.cpp

#include <iostream>
using namespace std;

/*
int sum(int a, int b){
    return a+b;
}

double sum(double a, double b){
    return a + b;
}

float sum(float a, float b){
    return a+ b;
}
*/

template<typename T>
T sum(T a, T b) {
    return a + b;
}

int main(){
    auto s1 = sum<int>(2, 3);
    auto s2 = sum<double>(2.1, 3.1);

    cout<<s1<<endl;
    cout<<s2<<endl;

    return 0;
}