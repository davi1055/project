#include <cstring>
#include <iostream>
// class rule_of_three {
//     char *cstring; // 以裸指针为动态分配内存的句柄
//
//     rule_of_three(const char *s, std::size_t n) // 避免重复计数
//         : cstring(new char[n])                  // 分配
//     {
//         std::memcpy(cstring, s, n); // 填充
//     }
//
//   public:
//     char *get_address() {
//         std::cout << static_cast<void *>(cstring) << "\n";
//         return cstring;
//     }
//     void get_num(size_t n) { std::cout << *(cstring + n) << "\n"; }
//     rule_of_three(const char *s = "") : rule_of_three(s, std::strlen(s) + 1)
//     {}
//
//     ~rule_of_three() // I. 析构函数
//     {
//         delete[] cstring; // 解分配
//     }
//
//     rule_of_three(const rule_of_three &other) // II. 复制构造函数
//         : rule_of_three(other.cstring) {}
//
//     rule_of_three &operator=(const rule_of_three &other) // III. 复制赋值
//     {
//         if (this == &other)
//             return *this;
//
//         std::size_t n{std::strlen(other.cstring) + 1};
//         char *new_cstring = new char[n];            // 分配
//         std::memcpy(new_cstring, other.cstring, n); // 填充
//         delete[] cstring;                           // 解分配
//
//         cstring = new_cstring;
//         return *this;
//     }
//
//     operator const char *() const // 访问器
//     {
//         return cstring;
//     }
// };

// template <typename... Args> void assignment(Args... args) { f(&args...); }

// template <typename... T> void f(T *point, T... Ts) {
//     int temp = sizeof...(Ts);
//     for (int i = temp; i > 0; i--) {
//         point = new T[1];
//         *point = ;
//         point++;
//     }
// }
template <typename... Args> auto f = [](void *ptr, Args &&args) {};
int main() { return 0; }
