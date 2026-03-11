#pragma once

#include <chrono>

class Timer {
public:
    Timer() : start_(std::chrono::steady_clock::now()) {}

    [[nodiscard]] double elapsed_seconds() const {
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - start_).count();
    }

private:
    std::chrono::steady_clock::time_point start_;
};