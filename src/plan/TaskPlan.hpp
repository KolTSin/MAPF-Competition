#pragma once

#include "actions/Action.hpp"
#include "domain/Position.hpp"
#include "tasks/Task.hpp"

#include <string>
#include <vector>

// Low-level plan produced for a single high-level task. Trajectories are kept
// alongside primitive actions so reservation tables can reserve future cells.
struct TaskPlan {
    int task_id{-1};
    TaskType task_type{TaskType::DeliverBoxToGoal};
    int agent_id{-1};

    std::vector<Action> primitive_actions;
    std::vector<Position> agent_trajectory;
    std::vector<Position> box_trajectory;

    bool success{false};
    std::string failure_reason;
};
