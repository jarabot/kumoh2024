// g++ -std=c++17 sharedptr.cpp
#include <iostream>
#include <string>

int main(){
    int *int_ptr = new int(11);

    std::cout<<(*int_ptr) << std::endl;
    delete int_ptr; // new로 메모리 할당 후 delete 해야 메모리 반환

    return 0;
}
