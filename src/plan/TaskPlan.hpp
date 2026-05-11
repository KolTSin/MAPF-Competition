#pragma once

#include "domain/Position.hpp"
#include "plan/AgentPlan.hpp"
#include "tasks/Task.hpp"

#include <string>
#include <vector>

// Machine-readable low-level planner failure categories.  The legacy
// failure_reason string is still populated for logs and older callers, while
// this enum lets repair code distinguish reservation, static-map, and search
// budget failures without parsing text.
enum class TaskFailureCause {
    None,
    InvalidAgent,
    InvalidBox,
    ReservedStartCell,
    BoxDestinationBlockedByWall,
    BoxDestinationBlockedByBox,
    BoxDestinationReserved,
    StaticComponentUnreachable,
    AgentCannotReachRequiredPushSide,
    NoLegalBoxTransportSuccessors,
    TimeHorizonReached,
    ExpansionLimitReached,
    NoPathForSingleBox
};

struct TaskFailureInfo {
    TaskFailureCause cause{TaskFailureCause::None};
    Position nearest_box_to_goal{-1, -1};
    Position nearest_reachable_push_side{-1, -1};
    Position blocking_position{-1, -1};
    char blocking_object{'\0'};
    int expansions{0};
    int max_expansions{0};
    int max_time{0};
};

// Low-level plan produced for a single high-level task. Trajectories are kept
// alongside primitive actions so reservation tables can reserve future cells.
struct TaskPlan {
    int task_id{-1};
    TaskType task_type{TaskType::DeliverBoxToGoal};
    int agent_id{-1};

    AgentPlan agent_plan;
    std::vector<Position> box_trajectory;

    bool success{false};
    std::string failure_reason;
    TaskFailureInfo failure_info;
};
