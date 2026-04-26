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
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace {
struct RecoveryPlan {
    int agent_id{-1};
    char box_symbol{'\0'};
    Position from{};
    Position destination{};
    std::vector<Action> actions;
};

const char* task_type_name(const HospitalTaskType type) {
    switch (type) {
    case HospitalTaskType::AssignBox:
        return "assign";
    case HospitalTaskType::RelocateBox:
        return "relocate";
    case HospitalTaskType::TransportBox:
        return "transport";
    }
    return "unknown";
}

std::optional<RecoveryPlan> try_recovery_relocation(
    const Level& level,
    State& state,
    const MapAnalysis& analysis,
    const std::vector<HospitalTask>& tasks,
    const BoxTransportPlanner& planner) {
    std::unordered_set<char> candidate_boxes;
    for (const HospitalTask& task : tasks) {
        if (task.box_symbol == '\0') {
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
            std::cerr << "[recovery-candidates] box=" << box
                      << " from=(" << from.row << "," << from.col << ")"
                      << " agent=" << agent_id
                      << " total=" << candidates.size()
                      << " trying=" << max_candidates
                      << '\n';

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

std::vector<Action> with_delay(const std::vector<Action>& plan, int delay) {
    std::vector<Action> out(static_cast<std::size_t>(delay), Action::noop());
    out.insert(out.end(), plan.begin(), plan.end());
    return out;
}

struct ConflictEvent {
    bool has_conflict{false};
    int first_agent{-1};
    int second_agent{-1};
    int time{-1};
};

Position position_at_time(const std::vector<Action>& plan, const Position& start, const int delay, const int time) {
    Position pos = start;
    if (time <= delay) {
        return pos;
    }

    int executed = 0;
    for (const Action& action : plan) {
        if (executed >= time - delay) {
            break;
        }
        pos = ActionSemantics::compute_effect(pos, action).agent_to;
        ++executed;
    }
    return pos;
}

ConflictEvent find_first_conflict(
    const std::vector<std::vector<Action>>& per_agent_plans,
    const State& initial_state,
    const std::vector<int>& delays) {
    const int num_agents = initial_state.num_agents();
    int horizon = 0;
    for (int agent = 0; agent < num_agents; ++agent) {
        horizon = std::max(horizon, delays[agent] + static_cast<int>(per_agent_plans[agent].size()) + 1);
    }

    for (int t = 0; t <= horizon; ++t) {
        for (int a = 0; a < num_agents; ++a) {
            const Position a_now = position_at_time(per_agent_plans[a], initial_state.agent_positions[a], delays[a], t);
            const Position a_prev = position_at_time(per_agent_plans[a], initial_state.agent_positions[a], delays[a], std::max(0, t - 1));
            for (int b = a + 1; b < num_agents; ++b) {
                const Position b_now = position_at_time(per_agent_plans[b], initial_state.agent_positions[b], delays[b], t);
                const Position b_prev = position_at_time(per_agent_plans[b], initial_state.agent_positions[b], delays[b], std::max(0, t - 1));

                if (a_now == b_now) {
                    return ConflictEvent{true, a, b, t};
                }

                if (a_prev == b_now && b_prev == a_now) {
                    return ConflictEvent{true, a, b, t};
                }
            }
        }
    }

    return ConflictEvent{};
}

std::optional<std::vector<int>> cbs_style_delay_resolution(
    const std::vector<std::vector<Action>>& per_agent_plans,
    const State& initial_state) {
    struct DelayNode {
        std::vector<int> delays;
        int cost{0};
    };

    struct NodeCompare {
        bool operator()(const DelayNode& lhs, const DelayNode& rhs) const noexcept {
            return lhs.cost > rhs.cost;
        }
    };

    const int num_agents = initial_state.num_agents();
    std::priority_queue<DelayNode, std::vector<DelayNode>, NodeCompare> open;
    std::unordered_set<std::string> closed;

    auto encode = [&](const std::vector<int>& delays) {
        std::string key;
        key.reserve(static_cast<std::size_t>(num_agents * 3));
        for (int d : delays) {
            key += std::to_string(d);
            key += '|';
        }
        return key;
    };

    open.push(DelayNode{std::vector<int>(static_cast<std::size_t>(num_agents), 0), 0});
    constexpr int MAX_DELAY_PER_AGENT = 256;
    constexpr int MAX_CBS_EXPANSIONS = 4096;
    int expansions = 0;

    while (!open.empty() && expansions < MAX_CBS_EXPANSIONS) {
        DelayNode node = open.top();
        open.pop();

        const std::string key = encode(node.delays);
        if (closed.contains(key)) {
            continue;
        }
        closed.insert(key);
        ++expansions;

        const ConflictEvent conflict = find_first_conflict(per_agent_plans, initial_state, node.delays);
        if (!conflict.has_conflict) {
            std::cerr << "[cbs] solved expansions=" << expansions
                      << " total_delay=" << node.cost
                      << '\n';
            return node.delays;
        }

        std::cerr << "[cbs] conflict t=" << conflict.time
                  << " agents=" << conflict.first_agent << "," << conflict.second_agent
                  << " delays=(" << node.delays[conflict.first_agent] << "," << node.delays[conflict.second_agent] << ")"
                  << '\n';

        for (const int agent : {conflict.first_agent, conflict.second_agent}) {
            if (node.delays[agent] >= MAX_DELAY_PER_AGENT) {
                continue;
            }

            DelayNode child = node;
            child.delays[agent] += 1;
            child.cost += 1;
            open.push(std::move(child));
        }
    }

    std::cerr << "[cbs] failed expansions=" << expansions << '\n';
    return std::nullopt;
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
        std::cerr << "[task-batch] iter=" << iter
                  << " tasks=" << tasks.size()
                  << '\n';
        if (tasks.empty()) {
            break;
        }

        bool progressed = false;
        for (std::size_t task_index = 0; task_index < tasks.size(); ++task_index) {
            const HospitalTask& task = tasks[task_index];
            std::cerr << "[task] iter=" << iter
                      << " idx=" << task_index
                      << " type=" << task_type_name(task.type)
                      << " agent=" << task.agent_id
                      << " box=" << task.box_symbol
                      << " from=(" << task.source.row << "," << task.source.col << ")"
                      << " to=(" << task.destination.row << "," << task.destination.col << ")"
                      << " reason=" << task.reason
                      << '\n';

            if (task.type == HospitalTaskType::AssignBox) {
                continue;
            }

            const std::vector<Action> segment = planner.plan_for_task(level, current, task);
            if (segment.empty()) {
                std::cerr << "[task-failed] iter=" << iter
                          << " idx=" << task_index
                          << " type=" << task_type_name(task.type)
                          << " agent=" << task.agent_id
                          << " box=" << task.box_symbol
                          << " reason=planner-returned-empty"
                          << '\n';
                continue;
            }

            auto& target = per_agent_plans[task.agent_id];
            target.insert(target.end(), segment.begin(), segment.end());

            std::cerr << "[execute] iter=" << iter
                      << " idx=" << task_index
                      << " type=" << task_type_name(task.type)
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

    std::vector<std::vector<Action>> conflict_free_plans(static_cast<std::size_t>(num_agents));
    std::vector<int> delays(static_cast<std::size_t>(num_agents), 0);
    if (const std::optional<std::vector<int>> resolved = cbs_style_delay_resolution(per_agent_plans, initial_state); resolved.has_value()) {
        delays = *resolved;
    } else {
        std::cerr << "[cbs] fallback to zero-delay merge\n";
    }

    for (int agent = 0; agent < num_agents; ++agent) {
        conflict_free_plans[agent] = with_delay(per_agent_plans[agent], delays[agent]);
        std::cerr << "[schedule] agent=" << agent
                  << " base_actions=" << per_agent_plans[agent].size()
                  << " delay=" << delays[agent]
                  << " scheduled_actions=" << conflict_free_plans[agent].size()
                  << '\n';
    }

    return PlanMerger::merge_agent_plans(conflict_free_plans, num_agents);
}
