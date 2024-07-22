// g++ -std=c++17 sharedptr.cpp
#include <iostream>
#include <memory>

int main(){
    std::shared_ptr<int> int_ptr = std::make_shared<int>(10);

    std::cout<<(*int_ptr) << std::endl;
    return 0;
}

