#include "actions/ActionApplicator.hpp"

#include "actions/ActionSemantics.hpp"
#include "domain/Color.hpp"

bool ActionApplicator::cell_has_other_agent(const State& state,
                                            int acting_agent,
                                            int row,
                                            int col) {
    for (int i = 0; i < state.num_agents(); ++i) {
        if (i == acting_agent) {
            continue;
        }
        if (state.agent_positions[i].row == row && state.agent_positions[i].col == col) {
            return true;
        }
    }
    return false;
}

bool ActionApplicator::cell_is_free(const Level& level,
                                    const State& state,
                                    int acting_agent,
                                    int row,
                                    int col) {
    if (!level.in_bounds(row, col)) {
        return false;
    }
    if (level.is_wall(row, col)) {
        return false;
    }
    if (state.has_box(row, col)) {
        return false;
    }
    if (cell_has_other_agent(state, acting_agent, row, col)) {
        return false;
    }
    return true;
}

bool ActionApplicator::is_applicable(const Level& level,
                                     const State& state,
                                     int agent_id,
                                     const Action& action) {
    const Position agent_pos = state.agent_positions[agent_id];
    const ActionEffect effect = ActionSemantics::compute_effect(agent_pos, action);

    switch (action.type) {
        case ActionType::NoOp:
            return true;

        case ActionType::Move:
            return cell_is_free(level, state, agent_id, effect.agent_to.row, effect.agent_to.col);

        case ActionType::Push: {
            if (!level.in_bounds(effect.box_from.row, effect.box_from.col)) {
                return false;
            }

            const char box = state.box_at(effect.box_from.row, effect.box_from.col);
            if (box == '\0') {
                return false;
            }

            const Color agent_color = level.agent_colors[agent_id];
            const Color box_color = level.box_colors[box - 'A'];
            if (agent_color != box_color) {
                return false;
            }

            return cell_is_free(level, state, agent_id, effect.box_to.row, effect.box_to.col);
        }

        case ActionType::Pull: {
            if (!cell_is_free(level, state, agent_id, effect.agent_to.row, effect.agent_to.col)) {
                return false;
            }

            if (!level.in_bounds(effect.box_from.row, effect.box_from.col)) {
                return false;
            }

            const char box = state.box_at(effect.box_from.row, effect.box_from.col);
            if (box == '\0') {
                return false;
            }

            const Color agent_color = level.agent_colors[agent_id];
            const Color box_color = level.box_colors[box - 'A'];
            if (agent_color != box_color) {
                return false;
            }

            return true;
        }
    }

    return false;
}

State ActionApplicator::apply(const Level& level,
                              const State& state,
                              int agent_id,
                              const Action& action) {
    (void) level;

    State next = state;

    if (!is_applicable(level, state, agent_id, action)) {
        return next;
    }

    const Position agent_pos = state.agent_positions[agent_id];
    const ActionEffect effect = ActionSemantics::compute_effect(agent_pos, action);

    switch (action.type) {
        case ActionType::NoOp:
            break;

        case ActionType::Move:
            next.agent_positions[agent_id] = effect.agent_to;
            break;

        case ActionType::Push: {
            const char box = state.box_at(effect.box_from.row, effect.box_from.col);
            next.set_box(effect.box_from.row, effect.box_from.col, '\0');
            next.set_box(effect.box_to.row, effect.box_to.col, box);
            next.agent_positions[agent_id] = effect.agent_to;
            break;
        }

        case ActionType::Pull: {
            const char box = state.box_at(effect.box_from.row, effect.box_from.col);
            next.set_box(effect.box_from.row, effect.box_from.col, '\0');
            next.set_box(effect.box_to.row, effect.box_to.col, box);
            next.agent_positions[agent_id] = effect.agent_to;
            break;
        }
    }

    return next;
}
