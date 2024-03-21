// Copyright 2024 davi

#include <chrono>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

void change_sstream(std::stringstream s, char a) {
    for (size_t i = 0; i < 9; i++) s.putback(a);
}

void out_func(char a, size_t loop_num) {
    for (size_t i = 0; i < loop_num; i++) {
        std::cout << a;
    }
}

void out_func_mutex(char a, size_t loop_num, std::mutex& mutex) {
    std::lock_guard<std::mutex> guard(mutex);
    for (size_t i = 0; i < loop_num; i++) {
        std::cout << a;
    }
}

int main() {
    // std::basic_ifstream<wchar_t> file_stream;
    // if (!file_stream.is_open()) file_stream.open("test_file");
    // if (file_stream.is_open()) {
    //     std::cout << "file open!" << "\n";
    //     std::thread([&file_stream]() {
    //         for (size_t i = 0; i < 10; i++) file_stream.putback(1);
    //     }).detach();

    //     std::thread([&file_stream]() {
    //         for (size_t i = 0; i < 10; i++) file_stream.putback(2);
    //     }).detach();
    // }
    // file_stream.close();

    // std::stringstream s1("000000000000000000000");
    // s1.get();
    // std::thread([&s1]() {
    //     for (size_t i = 0; i < 10; i++) {
    //         s1.putback('1');
    //     }
    // }).detach();
    // std::thread([&s1]() {
    //     for (size_t i = 0; i < 10; i++) {
    //         s1.putback('2');
    //     }
    // }).detach();

    // std::thread thread1(change_sstream, std::move(s1), '1');
    // std::thread thread2(change_sstream, std::move(s1), '2');
    // thread1.join();
    // thread2.join();
    // std::cout << s1.rdbuf() << "\n";

    // std::thread([]() {
    //     for (size_t i = 0; i < 1000; i++) {
    //         std::cout << "1";
    //     }
    // }).detach();

    // std::thread([]() {
    //     for (size_t i = 0; i < 1000; i++) {
    //         std::cout << "2";
    //     }
    // }).detach();

    /*
        std::thread supports only one parameter list
    */
    std::cout << "without mutex:\n";
    std::thread a(out_func, '1', 1000);
    std::thread b(out_func, '2', 1000);
    a.join();
    b.join();
    std::cout << "\nwith mutex: \n";

    std::mutex out_mutex;
    std::thread c(out_func_mutex, '1', 1000, std::ref(out_mutex));
    std::thread d(out_func_mutex, '2', 1000, std::ref(out_mutex));
    c.join();
    d.join();
    return 0;
}
