#include <cstddef>
#include <iostream>
#include <string>

/*
 * date:11.1
 */
template <typename T>
struct Test {
    T a{};
    Test(const T &a) : a(a){};
};

Test(const char *) -> Test<std::string>;

template <typename T, size_t size>
struct array {
    T arr[size];
};

template <class Tu, class... Tp>
array(Tu, Tp...) -> array<std::enable_if_t<(std::is_same_v<Tu, Tp> && ...), Tu>,
                          sizeof...(Tp) + 1>;

template <typename Tu, typename... Tp>
class Test2 {};

template <typename... Ts, typename U, typename = void>
void valid(U, Ts...);

template <class... Us>
void f(Us... pargs) {}

template <class... Ts>
void g(Ts... args) {
    auto f_ = [](auto arg) -> auto { return arg; }(f_(&args), ...);
    // f(&args...); // ��&args...�� �ǰ�չ��
    //  ��&args�� ������ģʽ
}

/*
 * date:11.2
 */

////1. ����...����
template <size_t... args>
struct X {
    void f() {
        /*for (const auto& i : { args... })
        {
                std::cout << i << " ";
        }*/
        ((std::cout << args << ""), ...);
        endl(std::cout);  // ʵ����������
    }
};

// 2. typename|class ... ����
template <typename... Args>
void f(Args... args) {
    ((std::cout << args << " "), ...);  // �۵�����ʽC++17
    endl(std::cout);
}

// 3.����Լ��...���� (C++20��)
template <std::integral... Args>
void f2(Args... args) {
    ((std::cout << args << " "), ...);
    endl(std::cout);
}

/*
 * date:11.3
 */

// 1.
class String {};
template <typename T>
class Array {
    // doing something...
};

template <typename T>
void sort(Array<T> &v) {
    // doing something...
}
void f(Array<String> &v) {
    sort(v);  // ��sort(Array<T>& v)��ʽ�ػ�Ϊ sort(Array<String>& v)
}

// error sort()����֮ǰ�ѱ���ʽ�ػ�Ϊsort(Array<String>& v)
template <>
void sort(Array<String> &v) {
    // doing something...
}

int main() { return 0; }
