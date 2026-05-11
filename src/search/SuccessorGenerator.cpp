#include "search/SuccessorGenerator.hpp"

#include "actions/ActionApplicator.hpp"
#include "actions/ActionLibrary.hpp"
#include "actions/ActionSemantics.hpp"
#include "domain/Color.hpp"

#include <algorithm>
#include <utility>

namespace {

bool cell_has_other_agent(const State& state, int acting_agent, const Position& p) {
    for (int i = 0; i < state.num_agents(); ++i) {
        if (i == acting_agent) continue;
        if (state.agent_positions[i] == p) return true;
    }
    return false;
}

bool is_static_free_cell(const Level& level,
                         const State& state,
                         int acting_agent,
                         const Position& p,
                         Position ignored_box) {
    if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) return false;
    const char box = state.box_at(p.row, p.col);
    if (box != '\0' && p != ignored_box) return false;
    if (cell_has_other_agent(state, acting_agent, p)) return false;
    return true;
}

bool compatible_box(const Level& level, int agent_id, char box_id) {
    if (box_id < 'A' || box_id > 'Z') return false;
    return level.agent_colors[agent_id] == level.box_colors[box_id - 'A'];
}

bool can_move_agent_for_active_box_push(const ReservationTable& reservations,
                                        int agent_id,
                                        char box_id,
                                        Position from,
                                        Position to,
                                        int absolute_time) {
    const int active_box_owner = -1000 - static_cast<int>(box_id);
    const bool destination_is_free_for_agent =
        !reservations.is_cell_reserved(to.row, to.col, absolute_time + 1, agent_id) ||
        !reservations.is_cell_reserved(to.row, to.col, absolute_time + 1, active_box_owner);

    return destination_is_free_for_agent &&
           !reservations.is_edge_reserved(to, from, absolute_time, agent_id) &&
           !reservations.is_incoming_reserved(to, absolute_time, agent_id) &&
           !reservations.is_outgoing_reserved(to, absolute_time, agent_id);
}

bool reserved_transition_is_valid(const ReservationTable& reservations,
                                  int agent_id,
                                  Position agent_from,
                                  Position agent_to,
                                  int absolute_time) {
    return reservations.can_move_agent(agent_id, agent_from, agent_to, absolute_time, absolute_time + 1);
}

} // namespace

void SuccessorGenerator::expand_agent(
    const Level& level,
    const State& state,
    int agent_id,
    int time,
    std::vector<Successor>& out)
{
    (void) time;
    out.clear();

    for (const Action& action : ActionLibrary::ALL_ACTIONS)
    {
        if (ActionApplicator::is_applicable(level, state, agent_id, action))
        {
            State next = ActionApplicator::apply(level, state, agent_id, action);

            out.push_back(Successor{
                action,
                std::move(next)
            });
        }
    }
}

void SuccessorGenerator::expand_agent_task(
    const Level& level,
    const State& state,
    int agent_id,
    Position current,
    int relative_time,
    int start_time,
    const ReservationTable& reservations,
    std::vector<AgentTaskSuccessor>& out)
{
    out.clear();

    for (const Action& action : ActionLibrary::ALL_ACTIONS) {
        if (action.type != ActionType::Move && action.type != ActionType::NoOp) continue;

        const ActionEffect effect = ActionSemantics::compute_effect(current, action);
        const Position next_agent = effect.agent_to;
        if (action.type == ActionType::Move &&
            !is_static_free_cell(level, state, agent_id, next_agent, Position{-1, -1})) {
            continue;
        }

        const int absolute_time = start_time + relative_time;
        if (!reserved_transition_is_valid(reservations, agent_id, current, next_agent, absolute_time)) {
            continue;
        }

        out.push_back(AgentTaskSuccessor{action, next_agent, relative_time + 1});
    }
}

void SuccessorGenerator::expand_box_transport(
    const Level& level,
    const State& state,
    int agent_id,
    char box_id,
    Position original_active_box,
    const BoxTransportSearchState& current,
    int start_time,
    const ReservationTable& reservations,
    std::vector<BoxTransportSuccessor>& out)
{
    out.clear();
    if (!compatible_box(level, agent_id, box_id)) return;

    for (const Action& action : ActionLibrary::ALL_ACTIONS) {
        const ActionEffect effect = ActionSemantics::compute_effect(current.agent, action);
        Position next_agent = effect.agent_to;
        Position next_box = current.box;

        const int absolute_time = start_time + current.time;
        bool valid = false;

        switch (action.type) {
            case ActionType::NoOp:
                valid = reservations.can_move_agent(agent_id, current.agent, next_agent, absolute_time, absolute_time + 1) &&
                        reservations.can_occupy_box(box_id, current.box, absolute_time + 1);
                break;

            case ActionType::Move:
                valid = is_static_free_cell(level, state, agent_id, next_agent, original_active_box) &&
                        next_agent != current.box &&
                        reservations.can_move_agent(agent_id, current.agent, next_agent, absolute_time, absolute_time + 1) &&
                        reservations.can_occupy_box(box_id, current.box, absolute_time + 1);
                break;

            case ActionType::Push:
                if (effect.box_from == current.box) {
                    next_box = effect.box_to;
                    valid = is_static_free_cell(level, state, agent_id, next_box, original_active_box) &&
                            can_move_agent_for_active_box_push(reservations, agent_id, box_id, current.agent, next_agent, absolute_time) &&
                            reservations.can_move_box(box_id, current.box, next_box, absolute_time, absolute_time + 1);
                }
                break;

            case ActionType::Pull:
                if (effect.box_from == current.box) {
                    next_box = effect.box_to;
                    valid = is_static_free_cell(level, state, agent_id, next_agent, original_active_box) &&
                            next_agent != current.box &&
                            reservations.can_apply_transition(agent_id,
                                                               current.agent,
                                                               next_agent,
                                                               box_id,
                                                               current.box,
                                                               next_box,
                                                               absolute_time);
                }
                break;
        }

        if (!valid) continue;
        out.push_back(BoxTransportSuccessor{action, BoxTransportSearchState{next_agent, next_box, current.time + 1}});
    }
}
