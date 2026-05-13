#include "plan/ConflictDetector.hpp"
#include "plan/AgentPlan.hpp"
#include "plan/Conflicts.hpp"
#include "domain/Position.hpp"
#include "actions/ActionSemantics.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct BoxInstance {
    int id{-1};
    char letter{'\0'};
    Position pos{-1, -1};
};

struct AgentAttempt {
    int agent{-1};

    Position agent_from{-1, -1};
    Position agent_to{-1, -1};

    Action action{};

    bool moves_box{false};
    int box_id{-1};
    char box_letter{'\0'};
    Position box_from{-1, -1};
    Position box_to{-1, -1};
};

bool is_box_action(const Action& action) {
    return action.type == ActionType::Push || action.type == ActionType::Pull;
}

int find_box_index_at(const std::vector<BoxInstance>& boxes, const Position& p) {
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        if (boxes[i].pos == p) {
            return i;
        }
    }
    return -1;
}

std::vector<BoxInstance> extract_boxes(const State& state) {
    std::vector<BoxInstance> boxes;
    int next_id = 0;

    // Important: do NOT use unordered_map<char, Position>.
    // The domain allows multiple boxes with the same letter.
    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const char b = state.box_at(r, c);
            if (b != '\0') {
                boxes.push_back(BoxInstance{
                    next_id++,
                    b,
                    Position{r, c}
                });
            }
        }
    }

    return boxes;
}

Conflict make_conflict(ConflictType type, int a0, int a1, int time) {
    Conflict c;
    c.type = type;
    c.agents[0] = a0;
    c.agents[1] = a1;
    c.time = time;
    return c;
}

void add_agent_agent_conflicts(
    const std::vector<AgentAttempt>& attempts,
    int t,
    std::vector<Conflict>& out
) {
    for (int i = 0; i < static_cast<int>(attempts.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(attempts.size()); ++j) {
            const AgentAttempt& a = attempts[i];
            const AgentAttempt& b = attempts[j];

            // 1. Agent vertex conflict: both agents end in same cell.
            if (a.agent_to == b.agent_to) {
                Conflict c = make_conflict(
                    ConflictType::AgentVertex,
                    a.agent,
                    b.agent,
                    t + 1
                );
                c.cell = a.agent_to;
                out.push_back(c);
            }

            // 2. Agent edge swap conflict.
            if (a.agent_from == b.agent_to &&
                b.agent_from == a.agent_to &&
                a.agent_from != b.agent_from) {
                Conflict c = make_conflict(
                    ConflictType::AgentEdgeSwap,
                    a.agent,
                    b.agent,
                    t
                );
                c.from[0] = a.agent_from;
                c.to[0] = a.agent_to;
                c.from[1] = b.agent_from;
                c.to[1] = b.agent_to;
                out.push_back(c);
            }

            // 3. Follow conflict: a enters b's start cell while b leaves.
            if (a.agent_to == b.agent_from &&
                b.agent_from != b.agent_to &&
                !(a.agent_from == b.agent_to && a.agent_to == b.agent_from)) {
                Conflict c = make_conflict(
                    ConflictType::AgentFollow,
                    a.agent,
                    b.agent,
                    t
                );
                c.cell = b.agent_from;
                c.from[0] = a.agent_from;
                c.to[0] = a.agent_to;
                c.from[1] = b.agent_from;
                c.to[1] = b.agent_to;
                out.push_back(c);
            }

            // Symmetric follow conflict.
            if (b.agent_to == a.agent_from &&
                a.agent_from != a.agent_to &&
                !(b.agent_from == a.agent_to && b.agent_to == a.agent_from)) {
                Conflict c = make_conflict(
                    ConflictType::AgentFollow,
                    b.agent,
                    a.agent,
                    t
                );
                c.cell = a.agent_from;
                c.from[0] = b.agent_from;
                c.to[0] = b.agent_to;
                c.from[1] = a.agent_from;
                c.to[1] = a.agent_to;
                out.push_back(c);
            }
        }
    }
}

void add_same_box_conflicts(
    const std::vector<AgentAttempt>& attempts,
    int t,
    std::vector<Conflict>& out
) {
    for (int i = 0; i < static_cast<int>(attempts.size()); ++i) {
        if (!attempts[i].moves_box) {
            continue;
        }

        for (int j = i + 1; j < static_cast<int>(attempts.size()); ++j) {
            if (!attempts[j].moves_box) {
                continue;
            }

            if (attempts[i].box_id == attempts[j].box_id) {
                Conflict c = make_conflict(
                    ConflictType::SameBoxMovedByTwoAgents,
                    attempts[i].agent,
                    attempts[j].agent,
                    t
                );
                c.boxes[0] = attempts[i].box_id;
                c.boxes[1] = attempts[j].box_id;
                c.box_letters[0] = attempts[i].box_letter;
                c.box_letters[1] = attempts[j].box_letter;
                c.cell = attempts[i].box_from;
                out.push_back(c);
            }
        }
    }
}

void add_agent_box_conflicts(
    const std::vector<AgentAttempt>& attempts,
    const std::vector<BoxInstance>& boxes,
    int t,
    std::vector<Conflict>& out
) {
    for (const AgentAttempt& a : attempts) {
        // A. Agent tries to enter a box's start cell.
        //
        // This is illegal for all boxes except the box this same agent is legally pushing.
        // In a push, the agent moves into the pushed box's old cell.
        for (const BoxInstance& b : boxes) {
            const bool own_pushed_box =
                a.moves_box &&
                a.action.type == ActionType::Push &&
                a.box_id == b.id &&
                a.agent_to == b.pos;

            if (a.agent_to == b.pos && !own_pushed_box) {
                Conflict c = make_conflict(
                    ConflictType::AgentIntoBoxStartCell,
                    a.agent,
                    -1,
                    t
                );
                c.cell = b.pos;
                c.boxes[0] = b.id;
                c.box_letters[0] = b.letter;
                out.push_back(c);
            }
        }

        // B. Agent and moving box try to end in the same destination cell.
        for (const AgentAttempt& mover : attempts) {
            if (!mover.moves_box) {
                continue;
            }

            if (a.agent_to == mover.box_to) {
                Conflict c = make_conflict(
                    ConflictType::AgentBoxSameDestination,
                    a.agent,
                    mover.agent,
                    t + 1
                );
                c.cell = a.agent_to;
                c.boxes[0] = mover.box_id;
                c.box_letters[0] = mover.box_letter;
                out.push_back(c);
            }
        }
    }

    // C. Box tries to enter an agent's start cell.
    //
    // This is illegal except for the same agent's own Pull, where the box moves
    // into the pulling agent's old position.
    for (const AgentAttempt& mover : attempts) {
        if (!mover.moves_box) {
            continue;
        }

        for (const AgentAttempt& a : attempts) {
            const bool legal_own_pull =
                mover.agent == a.agent &&
                mover.action.type == ActionType::Pull &&
                mover.box_to == a.agent_from;

            if (mover.box_to == a.agent_from && !legal_own_pull) {
                Conflict c = make_conflict(
                    ConflictType::BoxIntoAgentStartCell,
                    mover.agent,
                    a.agent,
                    t
                );
                c.cell = mover.box_to;
                c.boxes[0] = mover.box_id;
                c.box_letters[0] = mover.box_letter;
                out.push_back(c);
            }
        }
    }
}

void add_box_box_conflicts(
    const std::vector<AgentAttempt>& attempts,
    const std::vector<BoxInstance>& boxes,
    int t,
    std::vector<Conflict>& out
) {
    // A. Two moving boxes end in the same cell.
    for (int i = 0; i < static_cast<int>(attempts.size()); ++i) {
        if (!attempts[i].moves_box) {
            continue;
        }

        for (int j = i + 1; j < static_cast<int>(attempts.size()); ++j) {
            if (!attempts[j].moves_box) {
                continue;
            }

            if (attempts[i].box_to == attempts[j].box_to) {
                Conflict c = make_conflict(
                    ConflictType::BoxVertex,
                    attempts[i].agent,
                    attempts[j].agent,
                    t + 1
                );
                c.cell = attempts[i].box_to;
                c.boxes[0] = attempts[i].box_id;
                c.boxes[1] = attempts[j].box_id;
                c.box_letters[0] = attempts[i].box_letter;
                c.box_letters[1] = attempts[j].box_letter;
                out.push_back(c);
            }
        }
    }

    // B. A moving box tries to enter another box's start cell.
    //
    // Since occupancy is checked at the beginning of the joint action, this is
    // unsafe even if the other box is also moving away.
    for (const AgentAttempt& mover : attempts) {
        if (!mover.moves_box) {
            continue;
        }

        for (const BoxInstance& b : boxes) {
            if (b.id == mover.box_id) {
                continue;
            }

            if (mover.box_to == b.pos) {
                Conflict c = make_conflict(
                    ConflictType::BoxIntoBoxStartCell,
                    mover.agent,
                    -1,
                    t
                );
                c.cell = b.pos;
                c.boxes[0] = mover.box_id;
                c.boxes[1] = b.id;
                c.box_letters[0] = mover.box_letter;
                c.box_letters[1] = b.letter;
                out.push_back(c);
            }
        }
    }
}

std::vector<AgentAttempt> compute_attempts(
    const Plan& plan,
    const State& initial_state,
    const std::vector<Position>& agent_positions,
    const std::vector<BoxInstance>& boxes,
    std::size_t t
) {
    std::vector<AgentAttempt> attempts;
    attempts.reserve(agent_positions.size());

    const auto& joint = plan.steps[t];

    for (std::size_t a = 0; a < agent_positions.size(); ++a) {
        AgentAttempt attempt;
        attempt.agent = static_cast<int>(a);
        attempt.agent_from = agent_positions[a];

        // Assumption: joint.actions is indexed by agent id.
        // If your PlanMerger always fills all agents, this should hold.
        assert(a < joint.actions.size());

        attempt.action = joint.actions[a];

        const ActionEffect effect =
            ActionSemantics::compute_effect(agent_positions[a], attempt.action);

        attempt.agent_to = effect.agent_to;

        if (is_box_action(attempt.action)) {
            const int box_index = find_box_index_at(boxes, effect.box_from);

            // If no box exists at box_from, the action is inapplicable.
            // This detector focuses on conflicts, so we do not classify it here.
            // You can add InvalidAction later if you want.
            if (box_index != -1) {
                attempt.moves_box = true;
                attempt.box_id = boxes[box_index].id;
                attempt.box_letter = boxes[box_index].letter;
                attempt.box_from = effect.box_from;
                attempt.box_to = effect.box_to;
            }
        }

        attempts.push_back(attempt);
    }

    return attempts;
}

void apply_attempts(
    const std::vector<AgentAttempt>& attempts,
    std::vector<Position>& agent_positions,
    std::vector<BoxInstance>& boxes
) {
    for (const AgentAttempt& a : attempts) {
        assert(a.agent >= 0);
        assert(a.agent < static_cast<int>(agent_positions.size()));
        agent_positions[a.agent] = a.agent_to;
    }

    for (const AgentAttempt& a : attempts) {
        if (!a.moves_box) {
            continue;
        }

        for (BoxInstance& b : boxes) {
            if (b.id == a.box_id) {
                b.pos = a.box_to;
                break;
            }
        }
    }
}

} // namespace

Position ConflictDetector::getPositionAt(const AgentPlan& plan, int t) {
    assert(!plan.positions.empty());

    if (t <= 0) {
        return plan.positions.front();
    }

    const int last = static_cast<int>(plan.positions.size()) - 1;
    if (t >= last) {
        return plan.positions.back();
    }

    return plan.positions[t];
}

std::vector<Conflict>
ConflictDetector::findAllAgentConflicts(const std::vector<AgentPlan>& plans) {
    std::vector<Conflict> conflicts;

    if (plans.empty()) {
        return conflicts;
    }

    int makespan = 0;
    for (const auto& p : plans) {
        makespan = std::max(makespan, static_cast<int>(p.actions.size()));
    }

    for (int t = 0; t < makespan; ++t) {
        std::vector<AgentAttempt> attempts;
        attempts.reserve(plans.size());

        for (int i = 0; i < static_cast<int>(plans.size()); ++i) {
            AgentAttempt a;
            a.agent = plans[i].agent;
            a.agent_from = getPositionAt(plans[i], t);
            a.agent_to = getPositionAt(plans[i], t + 1);
            attempts.push_back(a);
        }

        const std::size_t before = conflicts.size();
        add_agent_agent_conflicts(attempts, t, conflicts);

        // Return all conflicts from the earliest conflicting timestep.
        if (conflicts.size() > before) {
            return conflicts;
        }
    }

    return conflicts;
}

Conflict ConflictDetector::findFirstConflict(const std::vector<AgentPlan>& plans) {
    const std::vector<Conflict> conflicts = findAllAgentConflicts(plans);
    if (conflicts.empty()) {
        return Conflict{};
    }
    return conflicts.front();
}

std::vector<Conflict>
ConflictDetector::findAllConflicts(
    const Plan& plan,
    const State& initial_state,
    bool stop_at_first_conflicting_timestep
) {
    std::vector<Conflict> conflicts;

    std::vector<Position> agent_positions = initial_state.agent_positions;
    std::vector<BoxInstance> boxes = extract_boxes(initial_state);

    for (std::size_t t = 0; t < plan.steps.size(); ++t) {
        const std::vector<AgentAttempt> attempts =
            compute_attempts(plan, initial_state, agent_positions, boxes, t);

        std::vector<Conflict> step_conflicts;

        add_agent_agent_conflicts(attempts, static_cast<int>(t), step_conflicts);
        add_same_box_conflicts(attempts, static_cast<int>(t), step_conflicts);
        add_agent_box_conflicts(attempts, boxes, static_cast<int>(t), step_conflicts);
        add_box_box_conflicts(attempts, boxes, static_cast<int>(t), step_conflicts);

        if (!step_conflicts.empty()) {
            std::stable_sort(step_conflicts.begin(), step_conflicts.end(), [](const Conflict& a, const Conflict& b) {
                return a.time < b.time;
            });
            conflicts.insert(
                conflicts.end(),
                step_conflicts.begin(),
                step_conflicts.end()
            );

            // Important: after a conflict, the server state may diverge because
            // conflicting agents perform NoOp. So later simulated conflicts may be fake.
            if (stop_at_first_conflicting_timestep) {
                return conflicts;
            }
        }

        // Only apply if this timestep is conflict-free, or if the caller explicitly
        // wants best-effort diagnostics through the whole plan.
        apply_attempts(attempts, agent_positions, boxes);
    }

    return conflicts;
}

std::vector<Conflict>
ConflictDetector::findAllConflicts(
    const std::vector<AgentPlan>& plans,
    const State& initial_state,
    bool stop_at_first_conflicting_timestep
) {
    std::size_t horizon = 0;
    for (const AgentPlan& plan : plans) {
        horizon = std::max(horizon, plan.actions.size());
    }

    Plan joint_plan;
    joint_plan.steps.resize(horizon);
    for (std::size_t t = 0; t < horizon; ++t) {
        JointAction& step = joint_plan.steps[t];
        step.actions.resize(plans.size(), Action::noop());
        for (std::size_t agent = 0; agent < plans.size(); ++agent) {
            if (t < plans[agent].actions.size()) {
                step.actions[agent] = plans[agent].actions[t];
            }
        }
    }

    return findAllConflicts(joint_plan, initial_state, stop_at_first_conflicting_timestep);
}

Conflict ConflictDetector::findFirstConflict(
    const Plan& plan,
    const State& initial_state
) {
    const std::vector<Conflict> conflicts =
        findAllConflicts(plan, initial_state, true);

    if (conflicts.empty()) {
        return Conflict{};
    }

    return conflicts.front();
}

Conflict ConflictDetector::findFirstConflict(
    const std::vector<AgentPlan>& plans,
    const State& initial_state
) {
    const std::vector<Conflict> conflicts =
        findAllConflicts(plans, initial_state, true);

    if (conflicts.empty()) {
        return Conflict{};
    }

    return conflicts.front();
}

bool ConflictDetector::has_conflict(
    const Plan& plan,
    const State& initial_state,
    std::string* reason
) {
    const std::vector<Conflict> conflicts =
        findAllConflicts(plan, initial_state, true);

    if (conflicts.empty()) {
        return false;
    }

    if (reason) {
        *reason = to_string(conflicts.front().type);
    }

    return true;
}

std::string to_string(ConflictType type) {
    switch (type) {
        case ConflictType::None:
            return "none";

        case ConflictType::AgentVertex:
            return "agent_vertex_conflict";

        case ConflictType::AgentEdgeSwap:
            return "agent_edge_swap_conflict";

        case ConflictType::AgentFollow:
            return "agent_follow_conflict";

        case ConflictType::AgentIntoBoxStartCell:
            return "agent_into_box_start_cell_conflict";

        case ConflictType::AgentBoxSameDestination:
            return "agent_box_same_destination_conflict";

        case ConflictType::BoxIntoAgentStartCell:
            return "box_into_agent_start_cell_conflict";

        case ConflictType::BoxVertex:
            return "box_vertex_conflict";

        case ConflictType::BoxIntoBoxStartCell:
            return "box_into_box_start_cell_conflict";

        case ConflictType::SameBoxMovedByTwoAgents:
            return "same_box_moved_by_two_agents_conflict";
    }

    return "unknown_conflict";
}

std::string to_string(const Conflict& c) {
    std::ostringstream out;

    out << to_string(c.type)
        << " agents=(" << c.agents[0] << "," << c.agents[1] << ")"
        << " boxes=(" << c.boxes[0] << "," << c.boxes[1] << ")"
        << " time=" << c.time
        << " cell=(" << c.cell.row << "," << c.cell.col << ")";

    return out.str();
}
