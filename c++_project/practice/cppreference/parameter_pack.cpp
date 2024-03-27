// Copyright 2024 davi

#include <iostream>
#include <vector>

template <typename... Args>
void printNums(const Args... args) {
    int sum{0};
    std::cout << sizeof...(Args) << "\n";

    sum += (args + ...);
    // sum = (args + ... + sum);
    std::cout << sum << "\n";
    int dummy[] = {(std::cout << args << " ", 0)...};
    ((std::cout << args << " "), ...);
}

template <typename T, typename... Args>
void push_back_vec(std::vector<T> &v, Args &&...args) {
    static_assert((std::is_constructible_v<T, Args &&> && ...),
                  "type error");  //  std::is_constructible_v(T,Args...)
                                  //  能否用Args...类型的参数构造T
    ((v.push_back(args)), ...);
}

int main() {
    std::vector<int> vec;
    push_back_vec(vec, 1, 1, 2, 3, 4, 4);
    for (const auto &it : vec) {
        std::cout << it << "\n";
    }
    return 0;
}
