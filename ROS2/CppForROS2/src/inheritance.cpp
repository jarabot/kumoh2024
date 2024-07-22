// g++ -std=c++17 inheritance.cpp

#include <iostream>
using namespace std;

class Base {
    public: 
        int x;
};

class Derived: public Base {
    public:
         int y;
 };

int main() 
{
    Derived d;
    d.x = 11;
    d.y = 22;
    cout<<"d.x = "<<d.x<<" d.y = "<<d.y<<endl;
    return 0;
}


/*
struct Base {
        int x;
};

struct Derived: Base {
    int y;
};

int main() 
{
    Derived d;
    d.x = 11;
    d.y = 22;
    cout<<"d.x = "<<d.x<<" d.y = "<<d.y;
    return 0;
}

*/