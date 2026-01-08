#ifndef PARALLEL_PROCESSOR_H
#define PARALLEL_PROCESSOR_H

#include "CircularBuffer.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <future>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

// 2. ParallelProcessor 클래스 구현
template <typename T>
class ParallelProcessor {
private:
    int threadCount;

public:
    explicit ParallelProcessor(int threads) : threadCount(threads) {}

    // 병렬 처리를 위한 parallel_map 구현 (InputIt 제거하여 에러 수정)
    template <typename Func>
    auto parallel_map(const std::vector<T>& data, Func f) -> std::vector<decltype(f(std::declval<T>()))> {
        using ReturnType = decltype(f(std::declval<T>()));
        size_t n = data.size();
        std::vector<ReturnType> results(n);
        
        if (n == 0) return results;

        size_t actualThreads = std::min((size_t)threadCount, n);
        size_t chunkSize = n / actualThreads;
        std::vector<std::future<void>> futures;

        for (size_t i = 0; i < actualThreads; ++i) {
            size_t start = i * chunkSize;
            size_t end = (i == actualThreads - 1) ? n : (i + 1) * chunkSize;

            futures.push_back(std::async(std::launch::async, [&data, &results, start, end, f]() {
                for (size_t j = start; j < end; ++j) {
                    results[j] = f(data[j]);
                }
            }));
        }

        for (auto& fut : futures) fut.wait();
        return results;
    }
};


#endif