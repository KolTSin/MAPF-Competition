#pragma once

#include "solvers/Solver.hpp"

#include "plan/Conflicts.hpp"
#include "plan/ReservationTable.hpp"

#include <vector>

namespace cbs_solver_detail {

Constraint make_constraint_for_branch(const Conflict& conflict, int branch);
void add_constraint_reservation(const Constraint& constraint, ReservationTable& reservations);
ReservationTable build_reservations_for_agent(const std::vector<Constraint>& constraints, int agent);

} // namespace cbs_solver_detail

class CBSSolver final : public Solver {
public:
    Plan solve(const Level& level, const State& initial_state, const IHeuristic& heuristic) override;
};