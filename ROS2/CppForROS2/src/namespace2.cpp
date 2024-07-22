// g++ -std=c++17 namespace2.cpp

#include <iostream>
using namespace std;

// void foo() {
//     cout<<"foo1";
// }

// void foo() {
//     cout<<"foo2";
// }

// int main() 
// {
//     foo();
// }



namespace first
{
    void foo() {
        cout<<"first::foo\n";
    }
}

namespace second 
{
    void foo() {
        cout<<"second::foo\n";
    }
}

int main()
{
    first::foo();
    second::foo();
}
