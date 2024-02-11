# 					学习日志

## 1.折叠表达式

折叠表达式的四种语法：

（1）	(形参包 运算符... )

（2）	(... 运算符 形参包)

（3）	(形参包 运算符 ... 运算符 初值)

（4）	(初值 运算符 ... 运算符 形参包)

注意：这里四种语法形式中的（3）和（4）的初值可以是左值或右值

```c++
template <typename... Args> void printNums(Args... args) {
    int sum{0};                                                     
    sum = (args + ... + sum);  //(形参包 运算符 ... 运算符 初值)
    std::cout << sum << "\n";
}                                                         
```

## 2.逗号表达式

逗号表达式（Comma Expression）是C++中的一种运算符，它允许在单个语句中使用多个表达式，并返回最后一个表达式的值。一般起到分隔各个表达式的作用，逗号表达式的一般形式如下：

```C++
//(expression1, expression2, ..., expressionN)
```

关于逗号表达式的使用举个例子：

```c++
int main()
{
	int x=1;
	int y=2;
	int result=(x++,y++,x+y);  //x+y 最后得到的结果为:2+3=6
}
```

## 3.逗号表达式与折叠表达式

在折叠表达式中使用逗号表达式可以对形参包中的元素实现许多遍历操作

先来个简单的：

```c++
template <typename... Args>
void PrintNums(const Args... args)
{
    ((std::cout << args << " "),...);  //符合折叠表达式的(形参包 运算符 ...)形式
    							 //其中 形参包为(std::cout << args << " ")，运算符为 ','
}
int main()
{
    PrintNums(1,2,3,4,5);
    return 0;
}
```

所以在折叠表达式中使用到逗号表达式时形参包也可以是某个包含形参包的表达式，表达式(std::cout << args << " ")中包含形参包args

接下来是使用表达式来对形参包元素进行一些操作

```c++
template <typename T, typename... Args>
void push_back_vec(std::vector<T> &v, Args &&...args) {
    //首先检查类型是否正确
    static_assert((std::is_constructible_v<T, Args &&> && ...),
                  "type error"); //  std::is_constructible_v(T,Args...)
                                 //  能否用Args...类型的参数构造T
    ((v.push_back(args)), ...);  //  依次对形参包args中的元素进行push_back操作
}

int main() {
    std::vector<int> vec;
    push_back_vec(vec, 1, 1, 2, 3, 4, 4);
    for (const auto &it : vec) {
        std::cout << it << "\n";
    }
    return 0;
}
```

