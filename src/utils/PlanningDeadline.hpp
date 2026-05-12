#pragma once

#include <chrono>

// Shared wall-clock deadline for one solver invocation.  Passing the same
// instance through analysis, task generation, blocker generation, scheduling,
// and low-level search keeps every nested stage accountable to the caller's
// planning budget instead of letting a deep helper run after the outer loop has
// no time left.
class PlanningDeadline {
public:
    using Clock = std::chrono::steady_clock;

    PlanningDeadline() = default;

    explicit PlanningDeadline(Clock::time_point deadline) : deadline_(deadline) {}

    static PlanningDeadline after(std::chrono::milliseconds budget) {
        return PlanningDeadline(Clock::now() + budget);
    }

    [[nodiscard]] bool expired() const noexcept {
        return Clock::now() >= deadline_;
    }

    [[nodiscard]] Clock::time_point time_point() const noexcept { return deadline_; }

private:
    Clock::time_point deadline_{Clock::time_point::max()};
};
