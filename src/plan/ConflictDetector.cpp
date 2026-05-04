#include "plan/ConflictDetector.hpp"

#include "actions/ActionSemantics.hpp"

#include <vector>

bool ConflictDetector::has_conflict(const Plan& plan, const State& initial_state, std::string* reason) {
    std::vector<Position> positions = initial_state.agent_positions;
    for (std::size_t t = 0; t < plan.steps.size(); ++t) {
        std::vector<Position> next = positions;
        for (std::size_t a = 0; a < positions.size(); ++a) {
            next[a] = ActionSemantics::compute_effect(positions[a], plan.steps[t].actions[a]).agent_to;
        }
        for (std::size_t i = 0; i < next.size(); ++i) {
            for (std::size_t j = i + 1; j < next.size(); ++j) {
                if (next[i] == next[j]) { if (reason) *reason = "vertex_conflict"; return true; }
                if (positions[i] == next[j] && positions[j] == next[i]) { if (reason) *reason = "edge_swap_conflict"; return true; }
            }
        }
        positions = std::move(next);
    }
    return false;
}
