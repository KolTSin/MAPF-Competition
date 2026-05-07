#pragma once

#include "domain/Level.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

#include <vector>

class TaskPrioritizer {
public:
    void score(const Level& level, const State& state, std::vector<Task>& tasks) const;
};
