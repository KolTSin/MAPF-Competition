#include "actions/ActionSemantics.hpp"

ActionEffect ActionSemantics::compute_effect(const Position& agent_pos, const Action& action) {
    ActionEffect effect;
    effect.agent_from = agent_pos;
    effect.agent_to = agent_pos;

    switch (action.type) {
        case ActionType::NoOp:
            break;

        case ActionType::Move:
            effect.agent_to.row += drow(action.move_dir);
            effect.agent_to.col += dcol(action.move_dir);
            break;

        case ActionType::Push:
            effect.agent_to.row += drow(action.move_dir);
            effect.agent_to.col += dcol(action.move_dir);

            effect.moves_box = true;
            effect.box_from = effect.agent_to;
            effect.box_to = effect.box_from;
            effect.box_to.row += drow(action.box_dir);
            effect.box_to.col += dcol(action.box_dir);
            break;

        case ActionType::Pull:
            effect.agent_to.row += drow(action.move_dir);
            effect.agent_to.col += dcol(action.move_dir);

            effect.moves_box = true;
            effect.box_to = agent_pos;
            effect.box_from = agent_pos;
            effect.box_from.row -= drow(action.box_dir);
            effect.box_from.col -= dcol(action.box_dir);
            break;
    }

    return effect;
}