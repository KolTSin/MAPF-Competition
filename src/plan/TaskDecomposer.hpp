#pragma once

#include "actions/Action.hpp"
#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "plan/Task.hpp"
#include "state/State.hpp"

#include <vector>

class Decomposer {
public:
    explicit Decomposer(const Level& level);

    std::vector<Action> decompose_plan(
        const std::vector<Task>& high_level_tasks,
        const State& initial_state
    );

    void print_tree(const std::vector<Task>& tasks) const;

private:
    const Level& level_;

    Task decompose_task(const Task& task, State& state);

    Task decompose_move_box_to_goal(const Task& task, State& state);
    Task decompose_move_box_to_cell(const Task& task, State& state);
    Task decompose_move_box_one_step(const Task& task, State& state);
    Task decompose_move_agent_to(const Task& task, State& state);
    Task decompose_clear_cell(const Task& task, State& state);

    std::vector<Action> flatten_primitives(const Task& task) const;
    void print_task_recursive(const Task& task, int depth) const;

    bool validate_primitives_only(const Task& task) const;

    std::vector<Position> plan_agent_path(
        int agent_id,
        Position target,
        const State& state
    ) const;

    std::vector<Position> plan_box_path(
        char box,
        Position target,
        const State& state
    ) const;

    Action move_action_from_step(Position from, Position to) const;
    Action push_action_from_box_step(Position box_from, Position box_to) const;
};