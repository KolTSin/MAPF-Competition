#include "search/CBS.hpp"

#include "search/SpaceTimeAStar.hpp"

AgentPlan CBS::search(
    const Level& level,
    const State& initial_state,
    const int agent,
    const int max_time,
    const ReservationTable& reservations
) {
    SpaceTimeAStar low_level(heuristic_);
    return low_level.search(level, initial_state, agent, max_time, reservations);
}
