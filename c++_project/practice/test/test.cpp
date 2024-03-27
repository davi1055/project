// Copyright davi 2024
#include <iostream>

enum class Color { RED, GREEN, BLUE, YELLOW };

int main(int argc, char* argv[]) {
    Color opt = Color::RED;
    switch (opt) {
        case Color::RED:
            std::cout << "RED";
            break;
        case Color::GREEN:
            std::cout << "GREEN";
            break;
        case Color::BLUE:
            std::cout << "GREEN";
            break;
        case Color::YELLOW:
            std::cout << "GREEN";
            break;
    }
    std::cout << argc << "\n";
    return 0;
}
