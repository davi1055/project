// Copyright 2024 davi

#include <iostream>

int main(int args, char* argv[]) {
    int a = 1;
    // int b = 2;
    // int sum = [](int x, int y) -> auto& { return x + y };
    int& (*fpi)(int*) = [](auto* a) -> auto& { return *a; };
    // int (*fpi)(int*) = [](auto* a) -> auto { return *a; };
    &(int*)x = fpi(&a);
}
