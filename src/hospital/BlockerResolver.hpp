#pragma once

#include "analysis/LevelAnalysis.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

#include <vector>

class BlockerResolver {
public:
    [[nodiscard]] std::vector<Task> generate_blocker_tasks(const Level& level,
                                                           const State& state,
                                                           const LevelAnalysis& analysis,
                                                           int& next_task_id) const;
};
