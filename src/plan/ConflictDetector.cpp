#include "plan/ConflictDetector.hpp"

#include "actions/ActionSemantics.hpp"

#include <unordered_map>
#include <vector>

bool ConflictDetector::has_conflict(const Plan& plan, const State& initial_state, std::string* reason) {
    std::vector<Position> positions = initial_state.agent_positions;
    std::unordered_map<char, Position> boxes;
    for (int r = 0; r < initial_state.rows; ++r) {
        for (int c = 0; c < initial_state.cols; ++c) {
            const char b = initial_state.box_at(r, c);
            if (b != '\0') {
                boxes[b] = Position{r, c};
            }
        }
    }
    for (std::size_t t = 0; t < plan.steps.size(); ++t) {
        std::vector<Position> next = positions;
        std::unordered_map<char, Position> next_boxes = boxes;
        for (std::size_t a = 0; a < positions.size(); ++a) {
            const ActionEffect effect = ActionSemantics::compute_effect(positions[a], plan.steps[t].actions[a]);
            next[a] = effect.agent_to;
            if (plan.steps[t].actions[a].type == ActionType::Push || plan.steps[t].actions[a].type == ActionType::Pull) {
                for (auto& [box_id, box_pos] : next_boxes) {
                    if (box_pos == effect.box_from) {
                        box_pos = effect.box_to;
                        break;
                    }
                }
            }
        }
        for (std::size_t i = 0; i < next.size(); ++i) {
            for (std::size_t j = i + 1; j < next.size(); ++j) {
                if (next[i] == next[j]) { if (reason) *reason = "vertex_conflict"; return true; }
                if (positions[i] == next[j] && positions[j] == next[i]) { if (reason) *reason = "edge_swap_conflict"; return true; }
                if (next[i] == positions[j] && positions[j] != next[j]) { if (reason) *reason = "follow_conflict"; return true; }
                if (next[j] == positions[i] && positions[i] != next[i]) { if (reason) *reason = "follow_conflict"; return true; }
            }
        }
        std::vector<Position> box_positions;
        box_positions.reserve(next_boxes.size());
        for (const auto& [_, p] : next_boxes) {
            box_positions.push_back(p);
        }
        for (std::size_t i = 0; i < box_positions.size(); ++i) {
            for (std::size_t j = i + 1; j < box_positions.size(); ++j) {
                if (box_positions[i] == box_positions[j]) { if (reason) *reason = "box_conflict"; return true; }
            }
        }
        boxes = std::move(next_boxes);
        positions = std::move(next);
    }
    return false;
}
