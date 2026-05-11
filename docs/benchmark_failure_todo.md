# Benchmark Failure TODO

This checklist turns the current benchmark results in `benchmark_results_20260511_115056/summary.csv` into pick-off engineering tasks.  Each item describes the failure signal, the implementation idea, and the expected result once the item is complete.

## Benchmark snapshot

- Current run: 107 levels total, 57 solved, 32 unsolved, 18 timeout/crash-class failures.
- Strong areas: `BFSfriendly*`, simple `MAPF00`--`MAPF03C`, `MAsimple*`, `SAsoko1_*`, and `SAsoko2_*`.
- Weak areas: `MAPFreorder*`, `MAPFslidingpuzzle`, large `MA*` hospital/sort/challenge levels, `MAthomasAppartment` multi-color combinations, `SAsoko3_*`, `SAtowers*`, and several single-agent puzzle maps with blockers or deadlocks.

## 1. Implement real local repair

- [x] Replace the placeholder `LocalRepair::repair` stages with actual planning attempts.

**Current signal**

- Logs repeatedly reach `scheduler_empty; attempting local repair on first task`, then every repair attempt fails.
- Representative failures: `MAPFreorder.log`, `SACrunch.log`, and `MAthomasAppartment_redcyan.log`.
- The current repair implementation only succeeds when the failure string already contains synthetic `stage_ok_*` markers, so it does not currently repair real benchmark failures.

**Implementation idea**

- Implement each advertised repair stage as a concrete strategy:
  - Delay the failed task start time and retry with the same agent/box/parking target.
  - Try alternate compatible agents for box tasks.
  - Try alternate parking cells for blocker relocation tasks.
  - Temporarily remove or relax reservations in a small neighborhood around the failed task and replan the local subset.
  - Extract and return a safe prefix if the full wave cannot be repaired.
- Extend `TaskPlan::failure_reason` or add a richer failure object so repair can tell whether the failure was a reservation conflict, static unreachable path, blocked parking target, expansion limit, or dependency/order issue.
- Add focused tests that inject a failed delivery/blocker plan and assert that repair tries at least one real alternative before returning unresolved.

**Expected result**

- `SACrunch.lvl` and `MAthomasAppartment_redcyan.lvl` should no longer terminate immediately after the first failed blocker parking task.
- Logs should show concrete repair choices such as selected alternate parking cells or delayed start times instead of five identical failed repair attempts.
- The number of `UNSOLVED` levels with `solution_length=0` should drop.

## 2. Add cycle and stagnation detection for accepted waves

- [x] Detect and reject local push/pull loops and repeated task-local trajectories.

**Current signal**

- `SAsoko3_04.log` times out after thousands of repeated `Push(E,S)` / `Pull(W,N)` style actions.
- The top-level solver currently accepts waves if boxes move, even when the movement is cyclic and does not make useful goal progress.

**Implementation idea**

- Track a signature per active task: task type, agent, box letter, box start, box end, goal, and box trajectory hash.
- Reject a wave if the same task-local signature repeats without reducing distance-to-goal or satisfying a new goal.
- Add inverse-action detection for immediate oscillations, especially push/pull pairs that return the box to an earlier location.
- Maintain per-box progress counters based on static wall-only distance to the target goal; require accepted waves to improve or unlock another task after a small no-progress allowance.
- When a cycle is detected, force a different task order, alternate box choice, alternate parking target, or early safe-prefix return.

**Expected result**

- `SAsoko3_*` levels should stop producing very long repetitive action streams.
- Timeouts caused by cyclic output should convert into either solved runs or fast, diagnosable `UNSOLVED` runs with a clear `cycle_detected` reason.
- Benchmark wall time for failing `SAsoko3_*` cases should fall far below the 180-second server timeout.

## 3. Make blocker relocation and parking multi-candidate

- [x] Treat parking assignment as a search/ranking problem instead of a single selected target.

**Current signal**

- Blocker tasks fail with `no_path_for_single_box`, and the solver often gives up immediately.
- `MAthomasAppartment_redcyan.log` creates multiple blocker tasks that target the same parking coordinate, indicating insufficient global parking coordination.

**Implementation idea**

- Have `BlockerResolver` emit several ranked parking candidates per blocker, or attach a candidate list to each blocker task.
- Reserve chosen parking cells globally before later blocker tasks are generated or scheduled.
- Before scheduling a blocker task, cheaply validate whether the box can plausibly be moved to the parking cell using static reachability and push-side feasibility.
- If `BoxTransportPlanner` fails for one parking candidate, retry the next candidate before declaring the blocker unschedulable.
- Penalize parking cells on future box routes, agent routes, goal corridors, and chokepoints.

**Expected result**

- `SACrunch.lvl` should try alternate parking for boxes `B` and `D` instead of failing after the first `MoveBlockingBoxToParking` attempt.
- `MAthomasAppartment_*` multi-color cases should avoid assigning multiple blockers to the same parking cell unless the sequence is explicitly feasible.
- More failures should include actionable rejected-parking diagnostics.

## 4. Improve low-level box-planner failure reporting

- [x] Split `no_path_for_single_box` into specific failure causes.

**Current signal**

- Many distinct failures collapse into `no_path_for_single_box`, which gives the scheduler and repair layer no useful next step.

**Implementation idea**

- Track why the box transport search ended:
  - expansion limit reached,
  - time horizon reached,
  - agent cannot reach required push side,
  - box destination blocked by wall/box/reservation,
  - start cell reserved,
  - no legal push/pull successors,
  - static component unreachable.
- Preserve a best frontier sample: nearest box state to the goal, nearest reachable push side, and the first blocking object or reservation when known.
- Return the structured cause in `TaskPlan` and print it in verbose HTN logs.
- Use these causes to trigger the right repair path: delay for reservations, blocker relocation for blocking boxes, alternate parking for blocked parking cells, or expansion-limit escalation for search caps.

**Expected result**

- Logs should replace vague `no_path_for_single_box` messages with specific causes.
- Repair should stop trying irrelevant alternatives; for example, an expansion-limit failure should not be treated the same as a reserved-start-cell failure.
- Debugging new failures should require less manual log inspection.

## 5. Add dedicated MAPF agent-reordering mode

- [ ] Detect tightly coupled agent-only goal permutations and solve them jointly.

**Current signal**

- `MAPFreorder.lvl`, `MAPFreorder2.lvl`, `MAPFreorder3.lvl`, `MAPFreorderC.lvl`, and `MAPFslidingpuzzle.lvl` are unsolved.
- Logs show the solver treating these as independent `MoveAgentToGoal` tasks; after one agent moves, the remaining agents become unplannable.

**Implementation idea**

- Detect levels or subproblems where all remaining tasks are agent-only goals and agents share narrow corridors/rooms.
- Instead of independent planning, invoke CBS or space-time A* over the involved agents as a joint subproblem.
- Generate temporary parking/vacate-goal tasks for agents occupying a goal another agent must pass through.
- Add a small permutation search for assigning goal order in tight rooms.
- Avoid greedily committing one agent to its final goal if that cell is needed as transit space.

**Expected result**

- The `MAPFreorder*` family should solve or at least make multi-agent coordinated progress instead of failing after one agent moves.
- The timeout-adjacent `MAPF02B` and `MAPF03B` solved cases should become much faster because the planner will avoid pathological independent-agent ordering.

## 6. Add anytime fallback for large maps

- [ ] Ensure large hospital/sort/challenge levels emit useful safe prefixes before the server timeout.

**Current signal**

- `MAbispebjerg.lvl`, `MAbispebjergHospital.lvl`, `MAchallenge.lvl`, and `MAmultiagentSort.lvl` time out with no actions and `server_time_s=0.000`.

**Implementation idea**

- Add wall-clock checks inside expensive analysis, task generation, prioritization, and low-level planning loops, not only around the outer competitive wave loop.
- Start with a quick first wave: generate a small top-N set of reachable delivery tasks, plan the easiest one, and emit it before attempting expensive global blocker analysis.
- Cache level analysis, static distances, parking candidates, and component reachability across waves.
- Add a budget-aware mode to skip deep blocker generation when the first safe prefix has not yet been produced.
- Return the best safe prefix immediately if remaining budget falls below a configured threshold.

**Expected result**

- Large benchmarks should stop timing out with zero actions.
- Even if full solve remains hard, logs should show partial progress and a returned safe prefix.
- The `TIMEOUT_OR_CRASH` count should decrease, and failures should become shorter and more diagnosable.

## 7. Rework same-letter box selection and goal assignment

- [ ] Retry alternate physical boxes for same-letter goals when the selected box leads to a dead end or cycle.

**Current signal**

- `SAsoko3_*` levels have multiple same-letter boxes/goals and repeatedly regenerate delivery tasks for the same remaining goal.
- The generator currently chooses an unassigned matching box using a static heuristic before the low-level planner validates the downstream consequences.

**Implementation idea**

- Represent same-letter box assignment as a small matching problem using static distance, push feasibility, and deadlock risk.
- If the selected box fails or cycles, mark that `(box position, goal position)` pair as temporarily banned and retry another same-letter box.
- Prefer boxes that are not already satisfying another goal, but allow moving a satisfied same-letter box only when it is required to unblock the remaining puzzle and can be restored.
- Add regression tests with multiple identical boxes where the nearest box is not the solvable choice.

**Expected result**

- `SAsoko3_*` levels should avoid repeatedly trying the same unproductive box-goal pairing.
- Single-agent Sokoban failures should shift from timeout loops to deliberate alternate assignment attempts.

## 8. Improve dependency generation from failed planning

- [ ] Convert low-level failures into new ordering constraints or new clearing tasks.

**Current signal**

- The scheduler skips tasks whose candidate plans fail and stops when no task can be added in a pass.
- This loses the reason the pass failed and does not generate new work to clear the blocker.

**Implementation idea**

- When a task fails, inspect the structured low-level failure cause and frontier sample.
- If a box blocks a needed route, create a `MoveBlockingBoxToParking` dependency before the failed delivery.
- If an agent blocks a needed route or goal, create a `ParkAgentSafely` dependency.
- If a reservation blocks the task, add an ordering edge or delayed start rather than skipping the task.
- Store failed-task diagnostics per wave and include them in verbose HTN output.

**Expected result**

- `scheduler_empty` should become rare; the scheduler should usually produce either new dependencies, blocker tasks, or a specific impossibility reason.
- Reorder and blocker-heavy maps should make progress after initially failed planning attempts.

## 9. Add a benchmark triage script and regression gates

- [ ] Add a script that summarizes benchmark results by family, status, runtime, and recurring failure reason.

**Current signal**

- The benchmark summary is useful, but identifying families such as `SAsoko3_*` and `MAPFreorder*` currently requires manual grouping and log inspection.

**Implementation idea**

- Create a script under `scripts/` that reads a benchmark result directory and prints:
  - solved/unsolved/timeout counts,
  - counts by level family prefix,
  - slow solved cases,
  - zero-action failures,
  - top failure reasons from logs,
  - suspected loops based on repeated action lines.
- Add optional JSON output so future CI can compare before/after benchmark runs.
- Use the script output to define small regression sets: `MAPFreorder*`, `SAsoko3_04`, `SACrunch`, selected `MAthomasAppartment_*`, and one large zero-action hospital map.

**Expected result**

- After each solver change, developers can quickly see whether the change improved a failure family or merely moved failures around.
- Future TODO items can be driven by objective deltas in solve rate, timeouts, and failure reasons.

## Suggested implementation order

1. [ ] Add benchmark triage script so every subsequent change has clear before/after metrics.
2. [ ] Implement structured `BoxTransportPlanner` failure reasons.
3. [ ] Implement real local repair for delay, alternate parking, and alternate agent.
4. [ ] Add multi-candidate blocker parking.
5. [ ] Add cycle/stagnation detection for accepted waves.
6. [ ] Add dedicated MAPF agent-reordering mode.
7. [ ] Add anytime fallback for large maps.
8. [ ] Rework same-letter box assignment for Sokoban-style levels.
9. [ ] Convert low-level failures into dependencies and generated clearing tasks.
10. [ ] Re-run the full benchmark suite and update this document with solved/unsolved deltas.
