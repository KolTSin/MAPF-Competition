# MAsimple Remaining Engineering Work TODO

This checklist tracks the additional coordination, blocker, parking, and validation work needed to solve the remaining `MAsimple` levels reliably. `MAsimple1` and `MAsimple2` are expected to solve with the current solver; `MAsimple3` through `MAsimple5` still expose gaps in task sequencing, blocker relocation, reservations, and local repair.

## Goal

- [ ] Solve `levels/MAsimple3.lvl` in the MAvis server with `build/searchclient --solver comp`.
- [ ] Solve `levels/MAsimple4.lvl` in the MAvis server with `build/searchclient --solver comp`.
- [ ] Solve `levels/MAsimple5.lvl` in the MAvis server with `build/searchclient --solver comp`.
- [ ] Add tests that fail if any `MAsimple` level produces an empty, invalid, conflicting, or non-solving plan.

## 1. Reserve boxes over time

The scheduler currently reserves agent paths, but transported boxes also need time-dependent reservations.

- [x] Extend `ReservationTable` with box/object cell reservations.
- [x] Reserve each transported box cell in every `TaskPlan::box_trajectory` timestep.
- [x] Reserve moved-box final cells for a persistence horizon.
- [x] Make later planning reject cells occupied or reserved by previously moved boxes.
- [x] Add tests for two tasks where the second task attempts to walk through, push through, or park on a reserved box cell.

## 2. Make `BoxTransportPlanner` reservation-aware

`BoxTransportPlanner` is still a single-box planner over the static/simulated state. It should account for scheduled agents and boxes.

- [x] Add a `ReservationTable` parameter or overload for `BoxTransportPlanner::plan`.
- [x] Reject agent destinations reserved at `t + 1`.
- [x] Reject box destinations reserved at `t + 1`.
- [x] Reject follow conflicts, incoming conflicts, and edge swaps using the reservation table.
- [x] Keep the single-box no-time closed set only for reservation-free planning; use time-aware keys when reservations are present.
- [ ] Make the expansion limit configurable through `SolverConfig` rather than hard-coded from map size.

## 3. Validate parking cells by future reachability

Parking cells must be more than static high-score cells. A candidate is useful only if it preserves future routes.

- [ ] For each parking candidate, simulate the blocker in that cell.
- [ ] Reject parking cells that are goals for unsatisfied boxes or agents.
- [ ] Reject parking cells that are chokepoints or disconnect required components.
- [ ] Reject parking cells that block the active delivery box from reaching its goal.
- [ ] Reject parking cells that block any compatible agent from reaching its assigned box.
- [ ] Prefer parking cells outside all coarse and planned future box routes.
- [ ] Add a regression test where the highest static parking score is unsolvable but a lower-score parking cell solves the level.

## 4. Complete blocker relocation before dependent delivery

`MAsimple3` needs relocation of an access-blocking box, followed by the originally blocked delivery. This ordering must be explicit and robust.

- [ ] Represent blocker relationships structurally instead of relying on `Task::debug_label` string matching.
- [ ] Ensure access-blocker relocation tasks are hard dependencies of the affected delivery tasks.
- [ ] After a blocker relocation, regenerate delivery tasks from the updated simulated state.
- [ ] If the selected parking cell still blocks the dependent delivery, reject it and try another candidate.
- [ ] Add a test based on `MAsimple3` where `B` blocks agent `0` from reaching `A`, requiring `B` to move first.

## 5. Add small interacting-task ordering search

`MAsimple4` and `MAsimple5` require trying alternative task orders instead of relying only on static priorities.

- [ ] Detect small coupled task sets of 2--4 unsatisfied box goals.
- [ ] Try multiple orderings of delivery and relocation tasks.
- [ ] Simulate each ordering with the updated state after every task.
- [ ] Reject orderings that make any remaining goal unreachable.
- [ ] Prefer the shortest valid ordering or the ordering with the fewest blocker relocations.
- [ ] Add regression tests for packed multi-box ordering in `MAsimple4` and cross-color ordering in `MAsimple5`.

## 6. Support goal boxes as temporary blockers

The remaining simple levels can require moving a box that itself has a goal later. The blocker resolver should handle this explicitly.

- [ ] Allow a goal-relevant box to become a temporary blocker relocation task when it blocks another required delivery.
- [ ] Record that the relocated goal box still needs its final delivery after parking.
- [ ] Ensure relocation does not mark the box's original delivery as complete.
- [ ] Build dependencies: temporary relocation -> unblocked delivery -> relocated box final delivery when needed.
- [ ] Add tests where a goal box must move out of the way and then be delivered afterward.

## 7. Replace Manhattan-only agent repositioning

`AgentPathPlanner` currently moves row-first then column-first. Tight maps need actual pathfinding.

- [ ] Replace row-first/column-first movement with BFS or A* over free cells.
- [ ] Respect walls, boxes, other agents, and reservations.
- [ ] Allow `NoOp` wait actions when a reservation blocks the shortest path temporarily.
- [ ] Add tests where the direct Manhattan route is blocked but another route exists.

## 8. Improve local repair

Local repair should generate meaningful alternatives when scheduling fails.

- [ ] When delivery planning fails, identify the blocking cell or blocking box from the failed search frontier.
- [ ] Generate relocation tasks for discovered blockers.
- [ ] Try alternate parking cells before giving up.
- [ ] Try alternate task orderings before giving up.
- [ ] Try delaying a conflicting task before giving up.
- [ ] Emit a diagnostic reason that explains which blocker or reservation caused failure.

## 9. Strengthen plan validation and tests

Existing tests should assert solved states, not just non-empty or conflict-free plans.

- [ ] Add a plan simulator that applies joint actions with hospital-domain applicability checks.
- [ ] Assert that all box goals are satisfied after applying the plan.
- [ ] Assert that all agent goals are satisfied when present.
- [ ] Assert no action attempts to move a box with an incompatible-color agent.
- [ ] Convert the current `MAsimple` loop into hard assertions for `MAsimple1` through `MAsimple5`.
- [ ] Add a server-run smoke script or CTest wrapper for all `MAsimple` levels.

## 10. Diagnostics for future debugging

The next failures should be easier to diagnose than `scheduler_empty`.

- [ ] Log planned task order, dependencies, selected parking cells, and rejected parking cells when verbose HTN tracing is enabled.
- [ ] Log why `BoxTransportPlanner` failed: expansion limit, unreachable agent side, blocked box destination, reservation conflict, or static obstacle.
- [ ] Log blocker-detection output: blocker box, affected goal box, candidate parking cells, and chosen parking cell.
- [ ] Log whether replanning made progress in boxes, agents, or goals.

## Suggested implementation order

1. [ ] Add solved-state validation tests for `MAsimple1`--`MAsimple5` so failures are visible.
2. [x] Add box reservations and make `BoxTransportPlanner` reservation-aware.
3. [ ] Make parking validation route/reachability-aware.
4. [ ] Replace debug-label blocker dependencies with structured dependencies.
5. [ ] Add small ordering search for coupled box tasks.
6. [ ] Support goal boxes as temporary blockers.
7. [ ] Replace `AgentPathPlanner` with BFS/A*.
8. [ ] Expand local repair to generate alternate parking and ordering attempts.
9. [ ] Re-run MAvis server smoke tests for every `MAsimple` level and check off the goals above.
