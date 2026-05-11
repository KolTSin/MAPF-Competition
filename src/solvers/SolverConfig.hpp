#pragma once

struct SolverConfig {
    bool debug_htn_trace{true};
    bool debug_analysis{false};
    int max_replans{256};
    int max_batch_tasks{4};
    int max_box_planner_expansions{50000};
    int max_agent_planner_expansions{20000};
    int reservation_persistence_horizon{20};
    int safe_prefix_length{6};
    int max_local_repair_attempts{2};
    int max_parking_candidates{8};
    int planning_time_budget_ms{175000};
};
