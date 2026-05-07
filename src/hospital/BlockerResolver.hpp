#pragma once

#include "analysis/LevelAnalysis.hpp"
#include "domain/Level.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

#include <vector>

// Converts the current level/state snapshot into explicit blocker-relocation
// tasks. The input is read-only planning context plus the next free task id; the
// output is a list of MoveBlockingBoxToParking tasks that tell downstream
// schedulers which box should be moved, which agent color can move it, where it
// should be parked, and which goal box it unblocks when that relationship is
// known.
class BlockerResolver {
public:
    [[nodiscard]] std::vector<Task> generate_blocker_tasks(const Level& level,
                                                           const State& state,
                                                           const LevelAnalysis& analysis,
                                                           int& next_task_id) const;
};
