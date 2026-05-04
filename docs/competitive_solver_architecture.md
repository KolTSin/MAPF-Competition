# Competitive HTN + Reservation Architecture

This document captures a practical architecture for MAPF-Competition-style Hospital solving that preserves HTN-style decomposition while remaining robust under runtime pressure.

## Design goals

- Keep high-level task decomposition (deliver goals, clear blockers, park objects safely).
- Avoid full joint-state optimal search blowup.
- Respect Hospital dynamics (synchronous joint actions, color-constrained box manipulation, conservative occupancy semantics).
- Enable incremental progress with local repair instead of global restart.

## End-to-end flow

```text
SearchClient
    ↓
LevelParser
    ↓
CompetitiveSolver
    ↓
LevelAnalyzer
    ↓
TaskGenerator
    ↓
DependencyBuilder / TaskPrioritizer
    ↓
TaskScheduler
    ↓
BoxTransportPlanner / AgentPathPlanner
    ↓
ReservationTable
    ↓
PlanMerger
    ↓
OnlineExecutor
```

Core loop:

1. Re-analyze current state + static map structure.
2. Generate delivery + blocker + positioning tasks.
3. Attach dependencies and score task priorities.
4. Plan a reservation-safe batch with focused planners.
5. Merge plans into joint actions.
6. Execute safe prefix and replan.

## Proposed modules

### `analysis/`

- `LevelAnalysis`: static/semi-static structural facts (corridors, chokepoints, components, parking scores, goals).
- `LevelAnalyzer`: computes these facts (DFS/BFS/Tarjan + classification).
- `StaticDistances`: wall-only BFS cache used for heuristic and task-cost estimates.
- `ParkingCellAnalyzer`: robust temporary placement scoring for blocker relocation.

### `tasks/`

- `Task`: explicit typed units (`DeliverBoxToGoal`, `MoveBlockingBoxToParking`, `MoveAgentToGoal`, etc.).
- `TaskGenerator`: emits goal tasks and blocker-clearing tasks.
- `DependencyBuilder`: encodes ordering constraints from route interference.
- `TaskPrioritizer`: competition-tuned scoring (urgency, unlock value, chokepoint risk, estimated cost).
- `TaskScheduler`: chooses ready tasks, assigns agents, plans + reserves successful plans.
- `HTNTracePrinter`: observability/debug output for high-level decomposition and primitive sequence.

### `hospital/`

- `BoxTransportPlanner`: A* over `(agent_pos, box_pos, time)` for one active box.
- `AgentPathPlanner`: space-time A* for agent-only repositioning.
- `BlockerResolver`: turns route blockers into relocation tasks with parking targets.
- `LocalRepair`: delay/alternate-agent/alternate-parking/local-neighborhood replanning.

### `plan/`

- `TaskPlan`: primitive action result and trajectories for one task.
- `ReservationTable`: cell+edge+incoming occupancy reservations by time.
- `ConflictDetector`: pre-execution collision checks (vertex/swap/follow/box conflicts).
- `PlanMerger`: converts per-task plans into synchronized joint actions.

### `solvers/`

- `CompetitiveSolver`: orchestration loop.
- `SolverConfig`: centralized tuning knobs (budgets, penalties, toggles).

## Key algorithmic choices

1. **Single active box low-level search**
   - Search state: `(agent_pos, box_pos, time)`.
   - Treat all other boxes as obstacles unless relocation task explicitly targets one.

2. **Reservation-first multi-agent coordination**
   - Each successful task reserves cell/edge occupancy in time.
   - New tasks must satisfy conservative movement constraints (no follow-into-occupied-at-start, no swap, no shared destination).

3. **Corridor/chokepoint hygiene**
   - Parking scoring strongly penalizes goals, corridors, articulation points, and chokepoints.
   - Blockers on bottlenecks become explicit high-priority relocation tasks.

4. **Local repair over global optimality**
   - Try small changes first: delay, alternate agent, alternate parking, local release-and-replan.
   - Execute safe prefix when full batch repair is not worth the budget.

## Incremental implementation plan

### Phase 1 — Static analysis foundation

Implement and validate:

- `LevelAnalysis`
- `LevelAnalyzer`
- `StaticDistances`
- `ParkingCellAnalyzer`

Diagnostics to print: corridor cells, chokepoints, dead ends, parking candidates, box goals, agent goals.

### Phase 2 — Single-box task planning

Implement and validate:

- `Task` (delivery-focused subset)
- `TaskGenerator` for `DeliverBoxToGoal`
- `TaskPlan`
- `BoxTransportPlanner`
- `HTNTracePrinter`

Target: reliable single-agent/single-box problem solving using primitive actions only.

### Phase 3 — Reservation-based scheduling

Implement:

- `ReservationTable`
- `AgentPathPlanner`
- `TaskScheduler`
- `PlanMerger`
- `ConflictDetector`

Target: small multi-agent coordination without full CBS.

### Phase 4 — Blocker relocation + dependencies

Implement:

- `BlockerResolver`
- blocker tasks in `TaskGenerator`
- `DependencyBuilder`
- `TaskPrioritizer`

Target: robust progress when boxes obstruct key routes.

### Phase 5 — Local repair

Implement:

- `LocalRepair`

Repair order:
1. delay task,
2. alternate agent,
3. alternate parking,
4. neighborhood release + local replan,
5. fallback to safe-prefix execution.

## Success criteria

A competition-viable solver should:

- make progress on multi-agent levels without global restart loops,
- avoid poisoning chokepoints with parked boxes,
- recover from partial execution failures quickly,
- produce interpretable HTN traces that explain why tasks were chosen.

## Priority classes to implement first

- Core engine: `BoxTransportPlanner`, `ReservationTable`.
- Competition quality: `TaskPrioritizer`, `LocalRepair`.
- Debuggability: `HTNTracePrinter`.
