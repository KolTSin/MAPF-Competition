#pragma once

#include "domain/Level.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"
#include "plan/AgentPlan.hpp"

#include <string>
#include <vector>

class TaskGenerator {
public:
    // Builds the high-level work list consumed by solvers.
    //
    // Input:
    // - `level` supplies static information such as goal cells, walls, and
    //   color ownership rules for agents and boxes.
    // - `state` supplies the current dynamic placement of agents and boxes.
    //
    // Output:
    // - one DeliverBoxToGoal task for each unsatisfied box goal that has a
    //   matching box and at least one color-compatible agent;
    // - additional blocker-clearing tasks that make those deliveries easier.
    //
    // Any goal that cannot produce a delivery task is recorded in
    // `skip_reasons()` so callers can explain why the task list is smaller
    // than the number of visible goals.
    [[nodiscard]] std::vector<Task> generate_delivery_tasks(const Level& level,
                                                            const State& state);
    [[nodiscard]] std::vector<Task> generate_delivery_tasks(const Level& level,
                                                            const State& state,
                                                            const std::vector<AgentPlan>& initial_agent_plans);

    // Human-readable explanations for goals that were intentionally ignored
    // during the last call to `generate_delivery_tasks`.
    [[nodiscard]] const std::vector<std::string>& skip_reasons() const noexcept { return skip_reasons_; }

private:
    // Goal cells can contain several kinds of symbols. Only uppercase letters
    // represent box goals; all other symbols are ignored by this generator.
    [[nodiscard]] static bool is_box_goal(char goal_symbol) noexcept;

    // Returns whether `agent_id` is allowed to move `box_id` according to the
    // level's color table. Invalid agent indices or non-box symbols are treated
    // as not movable instead of raising an error, which keeps generation robust
    // when parsing incomplete or inconsistent levels.
    [[nodiscard]] static bool can_agent_move_box(const Level& level, int agent_id, char box_id);

    // Cleared at the beginning of every generation run and then populated with
    // reasons for each skipped goal.
    std::vector<std::string> skip_reasons_{};
};
