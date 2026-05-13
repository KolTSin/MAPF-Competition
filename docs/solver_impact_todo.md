# Consolidated Solver TODO by Impact

This is the canonical open TODO list consolidated from:

- `docs/benchmark_failure_todo.md`
- `docs/masimple_remaining_work_todo.md`
- `docs/cbs_solver_review.md` for CBS prerequisites that block agent-only MAPF work

Completed checklist items from those files were treated as already-picked-off prerequisites rather than duplicated as new work here.  The ordering below is by expected solver impact, not by implementation size.  Each item lists the upstream work that must exist first so dependent tasks can be scheduled safely.

## 1. Make the whole planning pipeline deadline-aware and prefix-first

**Impact:** Highest. This directly targets large-map timeouts and zero-action failures where the client spends the server budget before emitting a validated prefix.

**Open work:**

- Thread one shared `PlanningDeadline` through level analysis, task generation, prioritization, blocker generation, local repair, and every low-level planner.
- Done: Add a cheap first-prefix mode that tries a small set of direct deliveries before expensive blocker enumeration.
- Cache static analysis, static distances, component reachability, parking candidates, and task skeletons across waves.
- Done: Stop expanding low-priority coarse-route blocker tasks when the remaining budget is too small to validate and return a safe prefix.
- Done: `BoxTransportPlanner` expansion limits are configurable through `SolverConfig` for competitive scheduling and local repair, with map-size fallback for default callers.

**Prerequisites:**

- Done: top-level competitive-solver wall-clock budget and safe-prefix return path.
- Done: reusable benchmark triage script for before/after measurement.

## 2. Convert scheduler stalls into explicit dependency or clearing work

**Impact:** Very high. This turns repeated `scheduler_empty` / `all_repair_stages_failed` loops into new prerequisite tasks or concrete impossible-state explanations.

**Open work:**

- When delivery planning fails, identify the blocking cell, blocking box, blocking agent, occupied goal, reservation conflict, or static impossibility from the failed search frontier.
- Convert structured failure causes into new task-graph work:
  - `MoveBlockingBoxToParking` for blocking boxes on agent or box routes,
  - `ParkAgentSafely` for agents occupying required routes or goals,
  - ordering edges when reservations or already-scheduled work block a task,
  - alternate physical box-goal assignments when same-letter matching caused the failure.
- Store generated dependencies in the task graph instead of retrying the same ready tasks.
- Emit a diagnostic reason that names the blocker/reservation and the dependency or clearing task generated.

**Prerequisites:**

- Done: machine-readable `TaskFailureCause` / `TaskFailureInfo` on low-level box planner failures.
- Done: local repair can already try delays, alternate agents, alternate parking, relaxed-reservation replans, and safe-prefix fallback.
- Done: structural blocker relationships and hard dependencies for access blockers replace debug-label string matching.
- Should follow item 1 so failure conversion respects the global deadline.

## 3. Feed cycle and stagnation detection back into planning as tabu memory

**Impact:** Very high. Current cycle detection rejects bad waves, but the next wave can still regenerate the same local trajectory, parking choice, or assignment.

**Open work:**

- Store per-wave and cross-wave tabu signatures for rejected `(task kind, box id/position, target position, first push/pull signature, final box position)` combinations.
- Feed rejected signatures into task prioritization, local repair, parking selection, and same-letter box assignment.
- When a rejected blocker move repeats because its target corridor is needed later, escalate to a generated dependency or reserve that corridor as non-parking.
- Add regression checks that a wave rejected for a repeated local trajectory is not regenerated unchanged.

**Prerequisites:**

- Done: cyclic/no-progress waves are rejected before appending to the output prefix.
- Should follow item 2 if rejected cycles need to synthesize dependencies rather than only lower priorities.

## 4. Rework parking into global, purpose-aware assignment

**Impact:** Very high. Bad parking can unblock one delivery while poisoning later routes, especially in blocker-heavy and corridor-heavy maps.

**Open work:**

- Make parking selection globally aware across waves and repair attempts, including rejected/cyclic parking choices.
- Keep parking cells outside exact planned future box routes once multi-task route planning exposes those routes.
- Replace cheap parking connectivity checks with `BoxTransportPlanner` push-feasibility checks for candidates that pass static filters.
- If a selected parking cell still blocks the dependent delivery, reject it and try another candidate.
- Solve multi-blocker parking as a small assignment/matching problem instead of greedily selecting the best cell per blocker.

**Prerequisites:**

- Done: candidate parking simulates the blocker in the candidate cell.
- Done: parking rejects unsatisfied goals, chokepoints, component-disconnecting cells, cells blocking the active delivery, and cells blocking agent access to assigned boxes.
- Done: blocker generation tracks reserved parking targets and prefers route/reachability-safe candidates, including coarse future box routes.
- Should follow item 3 so cyclic or rejected parking choices become tabu penalties.

## 5. Add small interacting-task ordering search and post-relocation regeneration

**Impact:** High. Some failures need alternate ordering across two to four coupled deliveries/relocations rather than a fixed priority order.

**Open work:**

- Detect small coupled task sets of two to four unsatisfied box goals.
- Try multiple delivery/relocation orderings.
- Simulate each ordering with the updated state after every task.
- Regenerate delivery tasks from the updated simulated state after a blocker relocation.
- Reject orderings that make any remaining goal unreachable.
- Prefer the shortest valid ordering or the ordering with the fewest blocker relocations.
- Add regression tests for packed multi-box ordering in `MAsimple4` and cross-color ordering in `MAsimple5`.

**Prerequisites:**

- Done: reservation-aware task planning and state replay for accepted tasks.
- Done: structural blocker dependencies for simple access-blocker cases.
- Should follow items 2 and 4 so failed orderings produce dependencies and parking choices are not locally poisonous.

## 6. Support goal-relevant boxes as temporary blockers

**Impact:** High. A box that eventually has its own goal may still need to move temporarily to unblock another required delivery.

**Open work:**

- Allow a goal-relevant box to become a temporary blocker relocation task.
- Record that the relocated goal box still needs final delivery after parking.
- Ensure relocation does not mark the box's original delivery as complete.
- Build dependencies: temporary relocation -> unblocked delivery -> relocated box final delivery, when needed.
- Add tests where a goal box must move out of the way and then be delivered afterward.

**Prerequisites:**

- Done: blocker tasks can be represented structurally and made hard dependencies of affected deliveries.
- Should follow item 5 because temporary goal-box relocation depends on reliable multi-step ordering and state regeneration.

## 7. Add deadlock-aware same-letter box matching and global box-order reasoning

**Impact:** High. Long Sokoban/tower-style partial plans often indicate that the solver can move boxes locally but commits to a bad physical box or goal order.

**Open work:**

- Replace greedy physical box selection with retryable matching between same-letter boxes and goals.
- Score assignments with static distance, push feasibility, deadlock risk, corridor dependencies, and goal-room order.
- Track failed `(box position, goal position)` attempts and retry alternate pairings.
- Detect stack/tower/goal-room situations where boxes must be placed in reverse dependency order.
- Allow moving a solved same-letter box only when it is necessary to unblock remaining work and can be restored.
- Add family-specific regression sets for towers, `SAsoko3_*`, `SACrunch`, `SAWatsOn`, and selected competition partial-progress failures.

**Prerequisites:**

- Should follow item 3 so failed pairings become tabu/failure memory.
- Should follow item 8 because goal-room and corridor deadlock annotations should inform matching scores.

## 8. Extend goal-room and corridor deadlock analysis

**Impact:** High. This prevents legal-looking parking or early deliveries from making future goals unreachable.

**Open work:**

- Compute articulation cells, one-cell-wide corridors, single-entrance rooms, goal-room entrance ordering, and cells behind irreversible push directions.
- Mark forbidden parking cells and low-priority temporary cells in `LevelAnalysis`.
- Add final-placement safety checks before accepting deliveries into goal rooms.
- Feed these annotations into task prioritization, parking selection, same-letter matching, and local repair.

**Prerequisites:**

- Done: static analysis already exposes parking candidates, chokepoints, components, and reachability enough for current parking filters.
- Should be developed before or alongside items 4 and 7 because those systems need the annotations.

## 9. Fix CBS correctness and scalability before relying on it for MAPF subproblems

**Impact:** Medium-high. Agent-only levels already route to CBS, so CBS must be correct and bounded before it can serve as the foundation for permutation/vacate-goal mode.

**Open work:**

- Compute and maintain `sum_of_costs` and `makespan` for root and child CBS nodes.
- Split vertex, edge, and follow constraints so vertex conflicts do not create accidental edge reservations.
- In each CBS branch, reserve only the constrained branch agent's forbidden transition.
- Validate all root low-level plans before inserting the root node.
- Gate CBS and low-level debug logs behind a runtime or compile-time verbose flag.
- Partially done: avoid extra full-node copies when branching CBS nodes. Remaining: remove redundant reservation checks, clarify max-time bounds, and handle agent goal encoding beyond single-digit ids.

**Prerequisites:**

- Done: no-box-goal levels route through the CBS solver instead of the HTN box scheduler.
- Should precede item 10 because MAPF permutation mode depends on a reliable joint-search backend.

## 10. Add dedicated agent-only MAPF permutation and vacate-goal handling

**Impact:** Medium-high. This targets zero-action agent-only/permutation failures that the box HTN path should not handle independently.

**Open work:**

- Add explicit vacate-goal, temporary parking, and permutation-ordering states for tightly coupled agent-only rooms/corridors.
- Detect all-agent/no-box subproblems inside mixed levels and solve them as coordinated joint segments.
- Search over goal/permutation ordering for tightly coupled rooms and corridors.
- Return the synchronized joint segment and reserve it as one coordinated wave.

**Prerequisites:**

- Done: no-box-goal levels route through the CBS solver instead of the HTN box scheduler.
- Requires item 9 so CBS constraints, node ordering, root validation, and logging behavior are dependable.

## 11. Add protocol-level and cause-oriented regression gates

**Impact:** Medium. This does not directly solve levels, but it prevents regressions and makes future solver changes measurable.

**Open work:**

- Re-run each `MAsimple` level through the MAvis server with `build/searchclient --solver comp` and record protocol-level results.
- Add a server-run smoke script or CTest wrapper for all `MAsimple` levels.
- Define a small benchmark regression matrix for MAPF permutation, blocker/parking, cycle repair, large timeout, tower ordering, and competition partial-progress failures.
- Add optional CI comparison around the triage script's JSON output.

**Prerequisites:**

- Done: in-process hard-validation tests solve and validate `MAsimple1` through `MAsimple5`.
- Done: plan simulator validates hospital-domain applicability, final box goals, final agent goals, and color compatibility.
- Done: triage script summarizes benchmark status, runtime, action counts, log markers, repeated parking targets, and JSON output.

## 12. Improve verbose HTN and low-level diagnostics

**Impact:** Medium-low. Better logs speed diagnosis, but they should stay gated because benchmark stderr can affect runtime.

**Open work:**

- Log planned task order, dependencies, selected parking cells, and rejected parking cells when verbose HTN tracing is enabled.
- Log why `BoxTransportPlanner` failed: expansion limit, unreachable push side, blocked destination, reservation conflict, static obstacle, or deadline expiry.
- Log blocker-detection output: blocker box, affected goal box, candidate parking cells, and chosen parking cell.
- Log whether replanning made progress in boxes, agents, or goals.

**Prerequisites:**

- Done: machine-readable failure info exists for important low-level box-planner failures.
- Should follow item 2 so diagnostics can report generated dependencies, not just raw failure causes.
