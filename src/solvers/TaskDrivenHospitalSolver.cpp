#include "solvers/TaskDrivenHospitalSolver.hpp"

#include "actions/Action.hpp"
#include "actions/ActionApplicator.hpp"
#include "actions/ActionSemantics.hpp"
#include "hospital/BoxTransportPlanner.hpp"
#include "hospital/HospitalTaskDecomposer.hpp"
#include "hospital/MapAnalysis.hpp"
#include "plan/PlanMerger.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace {
struct TimedCell {
    int row;
    int col;
    int time;

    bool operator==(const TimedCell& other) const noexcept {
        return row == other.row && col == other.col && time == other.time;
    }
};

struct TimedEdge {
    Position from;
    Position to;
    int time;

    bool operator==(const TimedEdge& other) const noexcept {
        return from == other.from && to == other.to && time == other.time;
    }
};

struct TimedCellHasher {
    std::size_t operator()(const TimedCell& v) const noexcept {
        std::size_t seed = std::hash<int>{}(v.row);
        seed ^= std::hash<int>{}(v.col) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(v.time) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct TimedEdgeHasher {
    std::size_t operator()(const TimedEdge& v) const noexcept {
        std::size_t seed = std::hash<int>{}(v.from.row);
        seed ^= std::hash<int>{}(v.from.col) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(v.to.row) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(v.to.col) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int>{}(v.time) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct ConflictTable {
    std::unordered_set<TimedCell, TimedCellHasher> cells;
    std::unordered_set<TimedEdge, TimedEdgeHasher> edges;
};

struct RecoveryPlan {
    int agent_id{-1};
    char box_symbol{'\0'};
    Position from{};
    Position destination{};
    std::vector<Action> actions;
};

std::optional<RecoveryPlan> try_recovery_relocation(
    const Level& level,
    State& state,
    const MapAnalysis& analysis,
    const std::vector<HospitalTask>& tasks,
    const BoxTransportPlanner& planner) {
    std::unordered_set<char> candidate_boxes;
    for (const HospitalTask& task : tasks) {
        if (task.type == HospitalTaskType::AssignBox) {
            continue;
        }
        candidate_boxes.insert(task.box_symbol);
    }

    auto pick_agent_for_box = [&](const char box, const Position& box_pos) {
        const Color box_color = level.box_colors[box - 'A'];
        int best_agent = -1;
        int best_distance = std::numeric_limits<int>::max();
        for (int agent = 0; agent < state.num_agents(); ++agent) {
            if (level.agent_colors[agent] != box_color) {
                continue;
            }

            const int dist = std::abs(state.agent_positions[agent].row - box_pos.row)
                             + std::abs(state.agent_positions[agent].col - box_pos.col);
            if (dist < best_distance) {
                best_distance = dist;
                best_agent = agent;
            }
        }
        return best_agent;
    };

    std::optional<RecoveryPlan> best_plan;

    for (int row = 0; row < state.rows; ++row) {
        for (int col = 0; col < state.cols; ++col) {
            const char box = state.box_at(row, col);
            if (box == '\0' || !candidate_boxes.contains(box)) {
                continue;
            }

            const Position from{row, col};
            const int agent_id = pick_agent_for_box(box, from);
            if (agent_id < 0) {
                continue;
            }

            std::vector<Position> candidates = analysis.find_relocation_candidates(state, from);
            constexpr int MAX_CANDIDATES_TO_TRY = 16;
            const int max_candidates = std::min(static_cast<int>(candidates.size()), MAX_CANDIDATES_TO_TRY);

            for (int i = 0; i < max_candidates; ++i) {
                const Position& destination = candidates[static_cast<std::size_t>(i)];
                if (analysis.is_transit_cell(destination.row, destination.col)) {
                    continue;
                }

                State simulated = state;
                const HospitalTask recovery_task{
                    HospitalTaskType::RelocateBox,
                    agent_id,
                    box,
                    from,
                    destination,
                    0,
                    "automatic recovery relocation"
                };

                std::vector<Action> actions = planner.plan_for_task(level, simulated, recovery_task);
                if (actions.empty()) {
                    continue;
                }

                if (!best_plan.has_value() || actions.size() < best_plan->actions.size()) {
                    best_plan = RecoveryPlan{
                        agent_id,
                        box,
                        from,
                        destination,
                        std::move(actions)
                    };
                }
            }
        }
    }

    return best_plan;
}

bool has_conflict(
    const std::vector<Action>& plan,
    const Position& start,
    int delay,
    const ConflictTable& conflicts) {
    Position pos = start;

    for (int t = 0; t <= delay; ++t) {
        if (conflicts.cells.contains(TimedCell{pos.row, pos.col, t})) {
            return true;
        }
    }

    int time = delay;
    for (const Action& action : plan) {
        const Position next = ActionSemantics::compute_effect(pos, action).agent_to;

        if (conflicts.cells.contains(TimedCell{next.row, next.col, time + 1})) {
            return true;
        }

        if (conflicts.edges.contains(TimedEdge{next, pos, time})) {
            return true;
        }

        pos = next;
        ++time;
    }

    return false;
}

void reserve_plan(
    const std::vector<Action>& scheduled_plan,
    const Position& start,
    ConflictTable& conflicts,
    Position& final_position) {
    Position pos = start;
    conflicts.cells.insert(TimedCell{pos.row, pos.col, 0});

    for (int t = 0; t < static_cast<int>(scheduled_plan.size()); ++t) {
        const Position next = ActionSemantics::compute_effect(pos, scheduled_plan[t]).agent_to;
        // Forbid follow conflicts: no later agent may enter a cell immediately
        // after another agent occupied it, even if it has just been vacated.
        conflicts.cells.insert(TimedCell{pos.row, pos.col, t + 1});
        conflicts.edges.insert(TimedEdge{pos, next, t});
        conflicts.cells.insert(TimedCell{next.row, next.col, t + 1});
        pos = next;
    }

    final_position = pos;
}

std::vector<Action> with_delay(const std::vector<Action>& plan, int delay) {
    std::vector<Action> out(static_cast<std::size_t>(delay), Action::noop());
    out.insert(out.end(), plan.begin(), plan.end());
    return out;
}

void reserve_goal_holding_cells(
    ConflictTable& conflicts,
    const std::unordered_map<int, Position>& final_positions,
    int max_horizon) {
    for (const auto& [agent, pos] : final_positions) {
        (void) agent;
        for (int t = 0; t <= max_horizon + 20; ++t) {
            conflicts.cells.insert(TimedCell{pos.row, pos.col, t});
        }
    }
}
}

Plan TaskDrivenHospitalSolver::solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) {
    (void) heuristic;

    State current = initial_state;
    const int num_agents = current.num_agents();

    MapAnalysis analysis(level);
    HospitalTaskDecomposer decomposer;
    BoxTransportPlanner planner(analysis);

    std::vector<std::vector<Action>> per_agent_plans(static_cast<std::size_t>(num_agents));
    constexpr int MAX_TASK_ITERATIONS = 512;
    for (int iter = 0; iter < MAX_TASK_ITERATIONS; ++iter) {
        const std::vector<HospitalTask> tasks = decomposer.decompose(level, current, analysis);
        if (tasks.empty()) {
            break;
        }

        bool progressed = false;
        for (const HospitalTask& task : tasks) {
            if (task.type == HospitalTaskType::AssignBox) {
                std::cerr << "[assign] agent=" << task.agent_id
                          << " box=" << task.box_symbol
                          << " -> goal=(" << task.destination.row << "," << task.destination.col << ")"
                          << " reason=" << task.reason
                          << '\n';
                continue;
            }

            const std::vector<Action> segment = planner.plan_for_task(level, current, task);
            if (segment.empty()) {
                continue;
            }

            auto& target = per_agent_plans[task.agent_id];
            target.insert(target.end(), segment.begin(), segment.end());

            std::cerr << "[execute] iter=" << iter
                      << " type=" << (task.type == HospitalTaskType::RelocateBox ? "relocate" : "transport")
                      << " agent=" << task.agent_id
                      << " box=" << task.box_symbol
                      << " actions=" << segment.size()
                      << " reason=" << task.reason
                      << '\n';

            progressed = true;
            break;
        }

        if (!progressed) {
            const std::optional<RecoveryPlan> recovery =
                try_recovery_relocation(level, current, analysis, tasks, planner);
            if (!recovery.has_value()) {
                std::cerr << "[stop] no executable hospital tasks at iter=" << iter << '\n';
                break;
            }

            auto& target = per_agent_plans[recovery->agent_id];
            target.insert(target.end(), recovery->actions.begin(), recovery->actions.end());
            for (const Action& action : recovery->actions) {
                current = ActionApplicator::apply(level, current, recovery->agent_id, action);
            }
            std::cerr << "[recovery] iter=" << iter
                      << " agent=" << recovery->agent_id
                      << " box=" << recovery->box_symbol
                      << " from=(" << recovery->from.row << "," << recovery->from.col << ")"
                      << " to=(" << recovery->destination.row << "," << recovery->destination.col << ")"
                      << " actions=" << recovery->actions.size()
                      << '\n';
        }
    }

    ConflictTable conflicts;
    std::unordered_map<int, Position> final_positions;
    std::vector<std::vector<Action>> conflict_free_plans(static_cast<std::size_t>(num_agents));
    int max_horizon = 0;

    for (int agent = 0; agent < num_agents; ++agent) {
        const auto& plan = per_agent_plans[agent];

        int delay = 0;
        constexpr int MAX_DELAY = 256;
        while (delay < MAX_DELAY && has_conflict(plan, initial_state.agent_positions[agent], delay, conflicts)) {
            ++delay;
        }

        std::vector<Action> scheduled = with_delay(plan, delay);
        Position final_pos{};
        reserve_plan(scheduled, initial_state.agent_positions[agent], conflicts, final_pos);
        final_positions[agent] = final_pos;
        max_horizon = std::max(max_horizon, static_cast<int>(scheduled.size()));
        conflict_free_plans[agent] = std::move(scheduled);

        std::cerr << "[schedule] agent=" << agent
                  << " base_actions=" << plan.size()
                  << " delay=" << delay
                  << " scheduled_actions=" << conflict_free_plans[agent].size()
                  << '\n';
    }

    reserve_goal_holding_cells(conflicts, final_positions, max_horizon);
    return PlanMerger::merge_agent_plans(conflict_free_plans, num_agents);
}
