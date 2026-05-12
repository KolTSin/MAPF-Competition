# Benchmark failure analysis (2026-05-12 runs)

Scope inspected:

- `benchmark_results_20260512_113533/summary.csv` plus its `logs/` directory.
- `benchmark_results_20260512_133326/summary.csv` plus its `logs/` directory.

## High-level counts

| Run | Unsolved with returned plan | No returned plan |
| --- | ---: | ---: |
| `benchmark_results_20260512_113533` | 19 | 20 |
| `benchmark_results_20260512_133326` | 19 | 14 |

## Unsolved levels that returned a plan

These failures are not all identical, but they have a shared shape: the solver emits a partial prefix, then later HTN waves cannot make safe progress. The repeated diagnostics are:

- `scheduler_empty; attempting local repair on ready tasks`
- `repair ... all_repair_stages_failed`, often mixed with small local successes such as `delay_start_to_t1`, `alternate_agent=...`, or `alternate_parking=...`
- `cycle_detected`, especially `repeated_world` and `repeated_task_local_trajectory ...`

The single-agent benchmark run (`20260512_113533`) is dominated by repeated-world or repeated-local-trajectory cycle rejections after a partial prefix. The Tower-style levels repeatedly move a blocking box into a short local trajectory and reject the next wave, leaving the initial tower goals unsatisfied.

The competition-level run (`20260512_133326`) shows the same cycle/stagnation pattern, but with more multi-agent partial prefixes. Levels such as `AIMAS7`, `AMC`, `BigSplit`, `GroupEZ`, `Spiraling`, `jAIl`, and `mosAIc` emit non-empty plans, then exhaust time or no-progress budget while ready tasks keep failing repair or repeat a previous world/trajectory.

### Ordering/causality issue found

The logs made the suspected jumbling plausible, and the scheduler confirmed the root cause. `schedule_once` replays every accepted task into a single `simulated_state` immediately, so later tasks are planned against a future world snapshot. Before this fix, a later task on a different agent could still be emitted at time 0 or another time before that future snapshot existed, because its start time only considered that agent's availability and explicit dependencies.

That means the emitted joint plan could contain actions whose prerequisites were produced by another task that was only simulated earlier, not actually scheduled earlier. This is the task-order jumbling suspected from the logs.

The fix adds a `global_state_frontier` that advances whenever the scheduler replays a task into `simulated_state`. All subsequently planned task starts, including dynamic agent-parking tasks, are now at least that frontier. This preserves the invariant that a task is never emitted before the state used to plan it exists.

## Levels marked unsolved with no returned plan

There are two different no-plan groups.

### Immediate empty output / no useful plan

In `benchmark_results_20260512_113533`, several `UNSOLVED` rows have `solution_length=0` and no useful returned plan:

- Agent-only MAPF/reorder levels (`MAPF02C`, `MAPF03C`, `MAPFreorder`, `MAPFreorder2`, `MAPFreorder3`, `MAPFreorderC`, `MAPFslidingpuzzle`) have no HTN trace because the competitive solver falls back to the agent-only CBS path when no box goals exist.
- `SAD2`, `SAD3`, `SATheRedDot`, and `SAbotbot` do enter HTN, but every wave is empty or rejected; local repair cannot find a safe task prefix, so no actions are emitted.

### Timeout before client emits a plan

The remaining no-plan failures are timeouts/crashes where the server killed the client before it wrote an action plan. In `benchmark_results_20260512_113533`, this includes `MAbispebjerg`, `MAbispebjergHospital`, `MAmultiagentSort`, `MAthomasAppartment`, `SAOptimal`, `SAbispebjergHospital`, `SAlabyrinth`, `SAlabyrinthOfStBertin`, and `SAsoko3_128`.

In `benchmark_results_20260512_133326`, every no-plan entry is a timeout before an emitted plan: `CphAirprt`, `ISO`, `Lily`, `LiteralAI`, `MAGiC`, `Medibots`, `Minchia`, `NameHere`, `PokeNOM`, `TheGate`, `doggy`, `donkeyK`, `help`, and `merRAM`.

## Follow-up recommendations

1. Re-run both benchmark sets after the scheduler-frontier fix. It may not solve every partial-plan level because many still need better blocker dependencies and cycle escape, but it removes one real source of out-of-order execution.
2. Add a benchmark log mode that prints accepted scheduled tasks with `(task_id, agent, start, end, type, box, goal)` so future investigations can distinguish real dependency order from priority order in `task_batch` traces.
3. For the zero-action HTN failures, capture the best low-level failure reason from scheduler candidates; currently the logs mostly collapse to `scheduler_empty` plus local-repair summaries.
4. For timeout-before-plan levels, periodically flush a validated safe prefix instead of waiting until the full solve returns, or enforce a stricter internal deadline margin so the client can emit before the server timeout.
