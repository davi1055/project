#include "./include/vector.hpp"
#include <iostream>

int main() {
    mystl::vector a = mystl::vector<int>(100);
    std::cout << sizeof(a) << "\n";
    return 0;
}
