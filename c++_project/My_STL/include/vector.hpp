#ifndef MY_VECTOR
#define MY_VECTOR

#include <cstddef>
#include <iostream>
namespace mystl {

template <typename T> class vector {
  public:
    // vector(最大容量)构造函数
    vector(size_t n) {
        _capcity = n;
        point = new T[_capcity];
    }
    // vector(最大容量，初值)构造函数
    vector(size_t n, T value) {}
    ~vector() {}

  private:
    typedef T _value_type;
    size_t _capcity;
    _value_type *point;
};

} // namespace mystl

#endif // !MY_VECTOR
