// Copyright [2024] davi
#include <future>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

void accumulate(std::vector<int>::iterator first,
                std::vector<int>::iterator last,
                std::promise<int> accumulate_promise) {
    int sum = std::accumulate(first, last, 0);
    accumulate_promise.set_value(sum);  // callback result to future
}

int main() {
    // an example of std::promise
    std::vector<int> numbers = {1, 2, 3, 4, 5, 6};
    std::promise<int> accumulate_promise;
    std::future<int> accumulate_future = accumulate_promise.get_future();
    std::thread work(accumulate, numbers.begin(), numbers.end(),
                     std::move(accumulate_promise));
    accumulate_future.wait();
    // std::cout << accumulate_future.get() << "\n";
    std::promise<int> p;
    std::future<int> future = p.get_future();
    std::thread([&p]() { p.set_value_at_thread_exit(9); }).detach();
    return 0;
}
