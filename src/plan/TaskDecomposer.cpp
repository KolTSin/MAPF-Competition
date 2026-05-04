#include "plan/TaskDecomposer.hpp"

#include "actions/ActionSemantics.hpp"
#include "search/SpaceTimeAStar.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace {

Direction direction_from_delta(Position from, Position to) {
    int dr = to.row - from.row;
    int dc = to.col - from.col;

    if (dr == -1 && dc == 0) return Direction::North;
    if (dr == 1 && dc == 0) return Direction::South;
    if (dr == 0 && dc == -1) return Direction::West;
    if (dr == 0 && dc == 1) return Direction::East;

    throw std::runtime_error("Invalid adjacent step.");
}

Position add_dir(Position p, Direction d) {
    switch (d) {
        case Direction::North: return {p.row - 1, p.col};
        case Direction::South: return {p.row + 1, p.col};
        case Direction::West: return {p.row, p.col - 1};
        case Direction::East: return {p.row, p.col + 1};
    }
    return p;
}

Position sub_dir(Position p, Direction d) {
    switch (d) {
        case Direction::North: return {p.row + 1, p.col};
        case Direction::South: return {p.row - 1, p.col};
        case Direction::West: return {p.row, p.col + 1};
        case Direction::East: return {p.row, p.col - 1};
    }
    return p;
}

} // namespace

Decomposer::Decomposer(const Level& level)
    : level_(level) {}

std::vector<Action> Decomposer::decompose_plan(
    const std::vector<Task>& high_level_tasks,
    const State& initial_state
) {
    State working_state = initial_state;

    std::vector<Task> decomposed_roots;
    std::vector<Action> result;

    std::cerr << "\n[HTN] Starting decomposition\n";

    for (const auto& task : high_level_tasks) {
        Task root = decompose_task(task, working_state);

        if (!validate_primitives_only(root)) {
            std::cerr << "[HTN][FAIL] Non-primitive leaf remains in task tree:\n";
            print_task_recursive(root, 0);
            throw std::runtime_error("HTN decomposition incomplete.");
        }

        auto actions = flatten_primitives(root);

        for (const auto& action : actions) {
            // Optional: simulate state here if you already have a single-agent applicator.
            // working_state = ActionApplicator::apply_single_agent(...);
        }

        result.insert(result.end(), actions.begin(), actions.end());
        decomposed_roots.push_back(root);
    }

    std::cerr << "\n[HTN] Decomposition tree\n";
    print_tree(decomposed_roots);

    std::cerr << "\n[HTN] Primitive plan\n";
    for (std::size_t t = 0; t < result.size(); ++t) {
        std::cerr << "  t=" << t << ": " << result[t].to_string() << "\n";
    }

    return result;
}

Task Decomposer::decompose_task(const Task& task, State& state) {
    switch (task.type) {
        case TaskType::MoveBoxToGoal:
            return decompose_move_box_to_goal(task, state);

        case TaskType::MoveBoxToCell:
            return decompose_move_box_to_cell(task, state);

        case TaskType::MoveBoxOneStep:
            return decompose_move_box_one_step(task, state);

        case TaskType::MoveAgentTo:
            return decompose_move_agent_to(task, state);

        case TaskType::ClearCell:
            return decompose_clear_cell(task, state);

        case TaskType::Primitive:
            return task;

        default:
            throw std::runtime_error(
                "No decomposition method for task: " + task_to_string(task)
            );
    }
}

Task Decomposer::decompose_move_box_to_goal(
    const Task& task,
    State& state
) {
    Task root = task;

    Task move_to_cell;
    move_to_cell.type = TaskType::MoveBoxToCell;
    move_to_cell.agent_id = task.agent_id;
    move_to_cell.box = task.box;
    move_to_cell.to = task.to;
    move_to_cell.priority = task.priority;

    root.children.push_back(decompose_task(move_to_cell, state));

    return root;
}


Task Decomposer::decompose_move_box_to_cell(
    const Task& task,
    State& state
) {
    Task root = task;

    std::vector<Position> box_path = plan_box_path(task.box, task.to, state);

    if (box_path.size() < 2) {
        throw std::runtime_error(
            "Could not plan box path for " + task_to_string(task)
        );
    }

    for (std::size_t i = 0; i + 1 < box_path.size(); ++i) {
        Task step;
        step.type = TaskType::MoveBoxOneStep;
        step.agent_id = task.agent_id;
        step.box = task.box;
        step.from = box_path[i];
        step.to = box_path[i + 1];

        root.children.push_back(decompose_task(step, state));
    }

    return root;
}

Task Decomposer::decompose_move_box_one_step(
    const Task& task,
    State& state
) {
    Task root = task;

    Direction box_dir = direction_from_delta(task.from, task.to);

    Position setup_cell = sub_dir(task.from, box_dir);

    Task position_agent;
    position_agent.type = TaskType::MoveAgentTo;
    position_agent.agent_id = task.agent_id;
    position_agent.to = setup_cell;

    root.children.push_back(decompose_task(position_agent, state));

    Task push;
    push.type = TaskType::Primitive;
    push.agent_id = task.agent_id;
    push.primitive = push_action_from_box_step(task.from, task.to);

    root.children.push_back(push);

    return root;
}

Task Decomposer::decompose_move_agent_to(
    const Task& task,
    State& state
) {
    Task root = task;

    std::vector<Position> path = plan_agent_path(task.agent_id, task.to, state);

    if (path.size() < 2) {
        return root;
    }

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        Task move;
        move.type = TaskType::Primitive;
        move.agent_id = task.agent_id;
        move.from = path[i];
        move.to = path[i + 1];
        move.primitive = move_action_from_step(path[i], path[i + 1]);

        root.children.push_back(move);
    }

    return root;
}

Task Decomposer::decompose_clear_cell(
    const Task& task,
    State& state
) {
    Task root = task;

    // TODO:
    // 1. Check whether task.to contains an agent.
    // 2. If yes, create MoveAgentTo(agent, parking_cell).
    // 3. Check whether task.to contains a box.
    // 4. If yes, create MoveBoxToCell(box, parking_cell).

    std::cerr << "[HTN][WARN] ClearCell not fully implemented yet: "
              << task_to_string(task) << "\n";

    return root;
}

std::vector<Position> plan_agent_path(
    int agent_id,
    Position target,
    const State& state
){

}