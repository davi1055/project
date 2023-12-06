#ifndef MY_VECTOR
#define MY_VECTOR

#include <cstddef>
namespace mystl {

template <typename T> class vector {
  public:
    typedef T value_type;
    vector(T t) : _t(t) {}
    ~vector() {}

  private:
    size_t _capcity;
    T _t;
};

} // namespace mystl

#endif // !MY_VECTOR
