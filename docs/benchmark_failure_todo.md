# Benchmark Failure TODO

This checklist turns the latest benchmark runs into pick-off engineering tasks. It uses both regular `levels/` benchmarks from `benchmark_results_20260511_172717` and competition `complevels/` benchmarks from `benchmark_results_20260511_181033`. Re-evaluation on 2026-05-12 separates already-landed mitigations from the still-open benchmark blockers; the benchmark counts below are still the latest recorded run data in this repository, not a fresh rerun after the mitigations.

## Current benchmark snapshot

| Suite | Run directory | Tested | Solved | Failed | Success rate | Avg solved time |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `levels/` | `benchmark_results_20260511_172717` | 107 | 68 | 39 | 63.55% | 5.077s |
| `complevels/` | `benchmark_results_20260511_181033` | 47 | 10 | 37 | 21.28% | 8.825s |

### Main failure points

1. **Large-map timeouts before emitting useful actions.** The regular benchmark still has 9 timeout/crash-class failures, and the competition benchmark has 22. Most of those report `server_time_s=0.000` and `solution_length=0`, which means the client spent the whole timeout before the server accepted any useful plan.
2. **Scheduler stalls after partial progress.** Many failures end in `scheduler_empty` followed by local repair attempts. In the latest logs this appears in 17 failed regular levels and 17 failed competition levels.
3. **Local repair is too shallow and often repeats the same unproductive choices.** `all_repair_stages_failed` appears in 16 failed regular levels and 16 failed competition levels. Successful repairs are often only a one-step delay, which helps local timing conflicts but does not create new domain work.
4. **Cycle detection is catching repeated local trajectories, but the solver does not learn enough from them.** `cycle_detected` appears in 24 failed regular levels and 15 failed competition levels, with repeated box trajectories such as moving the same box between the same two cells.
5. **Parking-cell selection over-concentrates blockers.** Logs show many generated `MoveBlockingBoxToParking` tasks targeting the same parking cell for several different boxes, which creates impossible or cyclic subproblems.
6. **Agent-only MAPF coordination is still missing.** `MAPFreorder*`, `MAPFslidingpuzzle`, and the larger `MAPF02C`/`MAPF03C` variants fail with zero solution actions despite the smaller MAPF cases solving quickly.
7. **Long single-agent puzzle families make progress but do not finish.** Tower levels, Sokoban/box-ordering levels, and several competition levels produce tens or hundreds of actions but are still unsolved, indicating that the high-level order is locally plausible but globally wrong.
8. **Benchmark reporting is good enough for pass/fail, but not yet good enough for cause-directed development.** The summary CSVs expose status, time, and action count, but recurring log reasons still require one-off scripts.

## Re-evaluated status

- **Still highest priority:** add reusable benchmark triage/reporting before deeper solver work so every future change can be measured against the same failure taxonomy.
- **Partially mitigated:** the competitive solver now uses a top-level planning-time budget and returns the accumulated safe prefix instead of falling into an unbounded CBS fallback, but the expensive analysis/generation/planning phases still do not share a deadline object.
- **Partially mitigated:** local repair now tries delayed starts, alternate agents, alternate parking, relaxed-reservation replans, and one-step safe-prefix fallback, but it still does not synthesize new dependencies from structured failure causes.
- **Partially mitigated:** no-box-goal levels now route through the CBS solver, covering the simplest agent-only MAPF cases; dedicated permutation/parking/vacate-goal handling remains open for tightly coupled MAPF families.
- **Still open:** same-letter box matching, goal-room/corridor deadlock analysis, and cause-oriented benchmark gates remain the main partial-progress puzzle work.

## TODO list

### 1. Add budget-aware early-exit and first-prefix planning for large maps

- [ ] Make expensive pre-planning and task-generation phases wall-clock aware.
  - [x] Add a top-level competitive-solver wall-clock budget and return the accumulated HTN/reservation prefix instead of running an unbounded fallback after useful progress exists.
  - [ ] Thread a shared deadline through level analysis, task generation, prioritization, blocker generation, local repair, and low-level planners.

**Failure signal**

- Timeout/crash failures are the largest competition failure class: 22 of 47 `complevels/` and 9 of 107 regular levels.
- Most timeout rows have `server_time_s=0.000` and `solution_length=0`, e.g. `AIMAS7`, `AMC`, `ISO`, `MAbispebjerg`, `MAmultiagentSort`, and full `MAthomasAppartment`.

**Implementation idea**

- Thread a shared deadline/budget object through level analysis, task generation, prioritization, blocker generation, and low-level planning.
- Add a cheap first wave that plans only the top few reachable, low-risk deliveries before running deep blocker enumeration.
- Cache static distances, level analysis, component reachability, parking candidates, and generated task skeletons across waves.
- If no accepted prefix exists by a small budget threshold, switch to a conservative fallback: choose the shortest safe delivery or a safe non-goal parking move and emit it.
- Stop exploring lower-priority blocker tasks once the remaining budget falls below the threshold needed to validate and print a safe prefix.

**Expected result**

- Timeout failures should no longer be dominated by `solution_length=0`.
- Large maps may still be unsolved initially, but they should emit validated partial progress and become visible as `UNSOLVED` with actions instead of timeout/no-action failures.
- The competition success rate should improve indirectly because smaller subproblems inside large maps will start completing before global reasoning times out.

### 2. Turn `scheduler_empty` into structured dependency generation

- [ ] When the scheduler cannot add any ready task, convert failed low-level plans into new clearing or ordering tasks.
  - [x] Add machine-readable `TaskFailureCause`/`TaskFailureInfo` for low-level box planner failures.
  - [x] Try delay, alternate-agent, alternate-parking, relaxed-reservation, and safe-prefix repair before giving up.
  - [ ] Convert those failure causes into new blocker, parking, agent-vacate, or ordering dependencies in the task graph.

**Failure signal**

- `scheduler_empty` appears in 17 failed regular levels and 17 failed competition levels.
- Current local repair can retry or delay a task, but the logs show many repeated repair failures without adding the missing prerequisite work.

**Implementation idea**

- Extend `TaskPlan` failures with structured causes: blocked route cell, blocking box id/letter, blocking agent id, reserved cell/time, goal occupied, parking occupied, dead corridor, or no color-compatible agent route.
- On `scheduler_empty`, inspect failed plans and synthesize one of:
  - `MoveBlockingBoxToParking` for a blocking box on the agent route or box route,
  - `ParkAgentSafely` for an agent occupying a needed route/goal,
  - an ordering edge when a reservation or already-planned task blocks the current task,
  - an alternate physical box-goal assignment when same-letter matching caused the failure.
- Store the generated dependency in the task graph instead of only retrying the same ready tasks.
- Add verbose HTN output that states exactly why the scheduler was empty and which dependency was generated.

**Expected result**

- `scheduler_empty` should become rare and actionable.
- Failed levels should transition from repeated “all repair stages failed” loops into either new clearing tasks or a concrete impossible-state reason.
- Blocker-heavy competition levels such as `DECrunchy`, `PFarthing`, `QRscammer`, and `jAIl` should solve more subgoals before stopping.

### 3. Make local repair stateful and tabu-aware

- [ ] Teach repair and wave planning to remember failed local trajectories and avoid immediately regenerating them.
  - [x] Reject cyclic/no-progress waves before appending them to the output prefix.
  - [ ] Feed rejected trajectory signatures back into local repair and parking selection as tabu constraints.

**Failure signal**

- `cycle_detected` appears in 24 failed regular levels and 15 failed competition levels.
- `repeated_task_local_trajectory` appears in both suites; examples include logs where the same box is moved back and forth between two cells, then the wave is rejected.

**Implementation idea**

- Store per-wave and cross-wave tabu entries for `(task kind, box id/position, target position, first push/pull signature, final box position)`.
- When a wave is rejected for a repeated trajectory, ban that exact task/parking/assignment choice for the next few waves.
- Feed cycle information back into task prioritization so the next wave selects a different parking cell, different same-letter box, different agent, or different subgoal order.
- Escalate repeated cycles into dependency generation: if a blocker move cycles because the target corridor is needed later, generate a different blocker-clearing task or reserve that corridor as non-parking.
- Add regression checks that assert a rejected wave is not regenerated unchanged.

**Expected result**

- Repeated `cycle_detected` messages should drop substantially.
- Runtime on failing levels should decrease because the solver will stop spending full waves on known-bad local trajectories.
- More partial-progress failures should convert into successful alternate plans rather than ending after several identical waves.

### 4. Rework parking-cell selection and reservation

- [ ] Make parking candidates conflict-aware, purpose-aware, and diversified across blockers.
  - [x] Track reserved parking targets within blocker generation and prefer route/reachability-safe parking candidates.
  - [ ] Make parking selection globally aware across waves and repair attempts, including rejected/cyclic parking choices.

**Failure signal**

- Logs show many blockers assigned to the same parking cell in one wave, such as repeated `MoveBlockingBoxToParking` entries targeting one cell for many boxes.
- This is especially visible in large single-agent/competition maps and contributes to local repair failure and repeated cycles.

**Implementation idea**

- Score parking by reachability, distance, future-goal safety, corridor criticality, component connectivity, and whether the cell is already targeted by another task.
- Reserve selected parking cells at the task-generation level, not only during low-level action planning.
- Keep parking cells away from articulation corridors, goal rooms, doorways, and known transit cells unless there is no alternative.
- For multi-box blocker sets, solve parking assignment as a small matching problem instead of greedily selecting the nearest available cell per blocker.
- If repair fails because parking is occupied or cyclic, blacklist that `(box, parking)` pair and retry a distinct parking cell.

**Expected result**

- The number of identical-parking blocker tasks in HTN output should drop.
- `all_repair_stages_failed` should decrease on blocker-heavy levels.
- Large maps should avoid self-inflicted congestion and preserve transit space for later deliveries.

### 5. Add a dedicated agent-only MAPF permutation mode

- [ ] Detect agent-goal permutation levels/subproblems and solve them as a coordinated multi-agent problem.
  - [x] Route levels with no box goals through the CBS solver instead of the HTN box scheduler.
  - [ ] Add explicit vacate-goal, permutation-ordering, and parking states for tightly coupled agent-only rooms/corridors.

**Failure signal**

- `MAPFreorder.lvl`, `MAPFreorder2.lvl`, `MAPFreorder3.lvl`, `MAPFreorderC.lvl`, and `MAPFslidingpuzzle.lvl` fail with zero actions.
- `MAPF02C.lvl` and `MAPF03C.lvl` also fail with zero actions while smaller MAPF variants solve.

**Implementation idea**

- Detect when all remaining tasks are `MoveAgentToGoal` tasks and no boxes need to move.
- Switch from independent task scheduling to a CBS/space-time A* joint subsolver for the involved agents.
- Add temporary “vacate goal” and “parking” states so an agent can leave its own goal if that cell is needed as a transit buffer.
- Search over goal/permutation ordering for tightly coupled rooms and corridors instead of greedily committing the first agent to its final goal.
- Return a synchronized joint plan segment and reserve it as one coordinated wave.

**Expected result**

- The `MAPFreorder*` family and `MAPFslidingpuzzle` should produce coordinated moves instead of immediate zero-action failures.
- `MAPF02C`/`MAPF03C` should either solve or fail with a much clearer joint-search limit.
- Future competition maps with agent-only subproblems should not be handled as independent single-agent tasks.

### 6. Improve same-letter box assignment and global box-order reasoning

- [ ] Replace greedy physical box selection with retryable matching and deadlock-aware ordering.

**Failure signal**

- Long single-agent families such as towers, `SAsokobanLevel96`, `SACrunch`, `SAWatsOn`, and multiple competition maps emit many actions but remain unsolved.
- This suggests the solver can move boxes locally, but it commits to an order or physical box choice that blocks the remaining puzzle.

**Implementation idea**

- For each letter, compute a small matching between physical boxes and compatible goals using static distance, push feasibility, deadlock risk, and corridor/goal-room dependencies.
- Track failed `(box position, goal position)` attempts and retry alternate boxes instead of regenerating the same task.
- Detect goal-room stacks and tower-like structures where boxes must be placed in reverse dependency order.
- Protect solved boxes only when they are truly final; allow moving a solved same-letter box if it is the only way to unblock the remaining goals and can be restored.
- Add family-specific regression sets: towers, `SAsoko3_*`, `SACrunch`, `SAWatsOn`, and a few competition partial-progress failures.

**Expected result**

- Partial-progress unsolved levels should either solve or stop earlier with a clear dependency/deadlock explanation.
- Tower and Sokoban-family action counts should become shorter and more goal-directed.
- Repeated attempts on the same bad same-letter pairing should disappear from verbose logs.

### 7. Add goal-room and corridor deadlock analysis

- [ ] Extend static analysis to mark cells where parking or early delivery can make future goals unreachable.

**Failure signal**

- Many levels fail after nonzero progress, which implies an earlier legal action can leave the remaining problem unsolvable.
- The current parking and blocker generation can place boxes in cells that look locally free but are globally critical transit or goal-room cells.

**Implementation idea**

- Compute articulation cells, one-cell-wide corridors, rooms with single entrances, goal-room entrance ordering, and cells behind irreversible push directions.
- Mark forbidden parking cells and low-priority temporary cells in `LevelAnalysis`.
- Add a “final placement safety” check before accepting a delivery into a goal room: verify that remaining boxes/goals in the same room still have an access order.
- Feed these annotations into task prioritization, parking selection, and local repair.

**Expected result**

- The solver should stop creating moves that make later goals unreachable.
- Long partial solutions should become more reliable, especially for tower/Sokoban-style levels and competition warehouse maps.
- Failure messages should distinguish true no-solution/deadlock from ordinary pathfinding failure.

### 8. Add cause-oriented benchmark triage and regression gates

- [ ] Create a reusable script that summarizes benchmark results by family, status, action count, runtime, and log reason.

**Failure signal**

- The CSV summaries are useful, but identifying recurring root causes currently requires ad hoc Python/log inspection.
- New benchmark runs added both `levels/` and `complevels/`, so manual comparison is now too slow and error-prone.

**Implementation idea**

- Add `scripts/triage_benchmarks.py` that reads one or more benchmark result directories and reports:
  - solved/unsolved/timeout counts,
  - zero-action failures,
  - partial-progress failures,
  - slow solved levels,
  - counts of log markers such as `scheduler_empty`, `repair outcome`, `all_repair_stages_failed`, `cycle_detected`, and `repeated_task_local_trajectory`,
  - grouped families such as `MAPF*`, `MAthomasAppartment*`, `SAtowers*`, and competition maps.
- Add optional JSON output so before/after runs can be compared in CI.
- Define a small regression matrix for every major failure mode: MAPF permutation, blocker/parking, cycle repair, large timeout, tower ordering, and competition partial-progress.

**Expected result**

- Every future solver change can be evaluated against the same failure taxonomy.
- PRs can report whether they improved success count, reduced zero-action failures, reduced cycles, or only shifted failures between categories.
- The team can pick work from this TODO without re-reading dozens of logs.

## Suggested implementation order

1. Add the triage script first, because it makes every subsequent task measurable and the latest benchmark counts have not yet been regenerated after the solver mitigations.
2. Finish shared-deadline/budget plumbing below the competitive-solver loop so zero-action timeouts cannot hide inside analysis, task generation, blocker generation, or low-level planning.
3. Turn scheduler stalls into dependency generation using the now-available structured failure diagnostics.
4. Make local repair tabu-aware and feed cycle/no-progress decisions back into repair attempts.
5. Rework parking-cell selection across waves/repair attempts so repeated parking targets stop poisoning blocker-heavy plans.
6. Add dedicated MAPF permutation/vacate-goal mode for the zero-action `MAPFreorder*` and `MAPFslidingpuzzle` failures that the generic CBS fallback still cannot cover reliably.
7. Improve same-letter matching and deadlock analysis for the remaining partial-progress puzzle families.
