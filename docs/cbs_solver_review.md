# CBSSolver / CBS stack review

This note summarizes concrete correctness risks and performance bottlenecks in the current CBS implementation.

## High-impact correctness issues

1. **High-level priority values are never maintained.**
   - `CBSFrontier` orders nodes by `sum_of_costs`, then `makespan`.
   - New nodes in `CBSSolver::solve` only update `makespan`, while `sum_of_costs` remains the default `0`.
   - Effect: the open list effectively degenerates to tie-breaking by makespan/index, not best-first CBS by cost.

2. **Vertex conflicts create accidental edge constraints.**
   - For all constraints, the solver reserves both edges from `cons.from`/`cons.to`.
   - For vertex conflicts, those arrays are never set in `ConflictDetector`, so default-initialized positions are used.
   - Effect: spurious reservations (often around `(0,0)`) can prune valid paths.

3. **Edge reservations are applied inconsistently with conflict semantics.**
   - Replanning branch `i` should apply only the specific forbidden move for that constrained agent.
   - Current code reserves both `from[0]->to[0]` and `from[1]->to[1]` for a given branch/agent.
   - Effect: over-constraining branches and reducing completeness/performance.

4. **Initial single-agent plans are not validated before root insertion.**
   - The root node is built from whatever `SpaceTimeAStar::search` returns.
   - If any agent has an invalid/empty plan, conflict checking asserts on empty positions.
   - Effect: crashes or undefined behavior on unsolved agents.

## Performance / scalability issues

1. **Heavy copying in hot path.**
   - `const CBSNode current = CT[current_index];` copies full plans + constraints each expansion.
   - Should be `const CBSNode& current = CT[current_index];`.

2. **Extensive `std::cerr` logging inside tight loops.**
   - Present in CBS branching, conflict detection, and low-level A* expansion.
   - Effect: major runtime overhead and noisy output under realistic instance sizes.

3. **Redundant reservation checks in low-level planner.**
   - `is_move_valid` checks `is_cell_reserved(..., next_time, ...)` twice.
   - Effect: extra hash lookups per successor with no added safety.

4. **Time-limit guard likely off-by-one.**
   - `if (next.time == max_time) continue;`
   - Should typically be `>= max_time` unless explicitly allowing exactly-`max_time` states.

5. **Agent goal encoding does not scale past 10 agents.**
   - Goal lookup relies on `'0' + agent_id` characters.
   - Effect: incorrect behavior once agent IDs exceed 9.

## Suggested fix order

1. Compute and set `sum_of_costs` (+ `makespan`) for root and every child node.
2. Split constraint types (vertex vs edge/follow) and only reserve fields relevant to each type.
3. In each branch, reserve only that branch agent's forbidden transition.
4. Validate all root low-level plans and fail fast if any are invalid.
5. Remove or gate debug logs behind a compile-time/runtime debug flag.
6. Apply cheap low-level cleanups (`const&` node access, dedup checks, time guard semantics).
