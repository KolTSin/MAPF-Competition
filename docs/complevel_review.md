# Competition Level Review

This note reviews every file in `complevels/` for three practical questions:

1. **What makes the level interesting?** The main structural or coordination feature that could stress a MAPF / Sokoban-style solver.
2. **What makes it uninteresting?** The aspect that may reduce search value, novelty, or fairness.
3. **Is it hard to solve?** A qualitative difficulty estimate based on static level structure plus the latest local benchmark in `benchmark_results_20260511_181033/summary.csv` where available.

Difficulty labels are intentionally solver-facing rather than mathematical guarantees:

- **Easy**: small state space or already solved quickly.
- **Medium**: has real coordination/search content but should be tractable with decent planning.
- **Hard**: likely to require strong conflict handling, decomposition, or domain heuristics.
- **Very hard**: large, dense, highly coupled, or timed out in the benchmark.
- **Unknown**: not present in the benchmark run.

## Executive summary

- The most benchmark-friendly but still useful levels are **Ccropper**, **MADS**, **MAchoupo2**, **MAybetwo**, **MAzonakis**, **noone**, **sigMAS**, **VNN**, **warehouse**, and **TourDeDTU**; all were solved, though TourDeDTU is much slower than the others.
- The most challenging suite members are the large/dense levels with many boxes or compartments: **Lily**, **MArtians**, **Minchia**, **PokeNOM**, **GroupEZ**, **mosAIc**, **AMC**, **BigSplit**, **NameHere**, **TheGate**, and **LiteralAI**.
- Some levels look aesthetically or thematically interesting but may be less diagnostic because they are either dominated by huge numbers of irrelevant boxes, disconnected regions, or very open areas that primarily stress parser/search scale rather than tight multi-agent reasoning.
- **TideMazes** exists in `complevels/` but was not included in the cited benchmark summary, so its difficulty is estimated statically only.

## Per-level review

| Level | Interesting because | Uninteresting / risk | Difficulty |
| --- | --- | --- | --- |
| AIMAS7 | Split upper/lower layout, many colored object groups, and narrow transitions create mixed decomposition and routing issues. | Three disconnected components and a large amount of open floor may make part of the map irrelevant if goals only touch subsets. | **Very hard** for the current solver: benchmark timed out/crashed with no solution. |
| AMC | Extremely box-dense storage-style level with repeated A/B patterns and many objects competing for space. | The 241 boxes can make it more of a scale/memory stress test than a clean coordination puzzle. | **Very hard**: timed out/crashed. |
| BigSplit | Large map with a visible maze half and a split-room half; ten agents and many boxes make it a strong interaction test. | Dense decorative W regions and many boxes can obscure the core puzzle and inflate branching. | **Very hard**: timed out/crashed. |
| Ccropper | Compact, highly constrained corridors with four independent colors; useful as a small bottleneck benchmark. | Because it is tiny and already solved in under a second, it may not differentiate stronger solvers. | **Easy-medium**: solved in 0.597 server seconds with a 59-step solution. |
| ClosedAI | Many agents, multiple colors, small rooms, and blocked-off pockets make it a good congestion/deadlock test. | Five disconnected components and many goals can create bookkeeping noise; benchmark produced actions but did not solve. | **Hard**: unsolved after 14.905 server seconds. |
| CphAirprt | Airport-like large map with ten agents, ten colors, and wide concourses connected by restricted passages. | Two components and broad open areas may reduce tactical richness in some regions. | **Very hard**: timed out/crashed. |
| DASH | Small but chaotic ten-agent board with several disconnected/open-ended areas; good for testing initial-state handling. | Some map edges are not fully enclosed, so it is less clean as a canonical hospital puzzle. | **Medium-hard**: benchmark returned unsolved immediately, suggesting solver/model incompatibility or early failure. |
| DECrunchy | Compact three-agent map with two dense object clusters and central transfer space; good for color-specific box ordering. | High corridor percentage may make it a narrow single-pattern puzzle once the intended ordering is found. | **Hard**: unsolved after 3.650 server seconds despite a long 249-step attempt. |
| DatzCrazy | Symmetric arena with dense central A region and side B columns; stresses interference around a central obstacle mass. | Four disconnected components and many same-letter boxes may make many objects irrelevant. | **Medium-hard**: unsolved after 6.271 server seconds, only 10 actions produced. |
| EpicfAIl | Combines a sparse upper maze with a dense lower cluster of labeled boxes, forcing cross-area planning. | The object layout may be more cluttered than purposeful, with many goals/boxes relative to only four agents. | **Hard**: unsolved after 10.834 server seconds. |
| GHandDirt | Mostly open field with diagonal bands of W boxes/obstacles, an unusual geometry for testing pathfinding around movable clutter. | Very open and almost no corridor structure, so it may not create many classic bottleneck conflicts. | **Hard**: unsolved after 52.019 server seconds. |
| GroupEZ | Huge 50x50 map with many rooms, 510 boxes, 209 goals, and 33 components; a maximal decomposition/scale challenge. | The number of disconnected regions and decorative clutter may dominate over meaningful solvability. | **Very hard**: timed out/crashed. |
| ISO | Ten agents/colors in a large maze with many dead ends and moderate box count; good for testing long-range navigation. | High wall/dead-end count may produce many inaccessible or low-value pockets. | **Very hard**: timed out/crashed. |
| Lily | Very large, very open, and extremely box-dense with 834 boxes and 273 goals; stresses large-scale assignment and memory. | The low wall density and massive text-like clutter make it more of a bulk-data stress test than a focused puzzle. | **Very hard**: timed out/crashed. |
| LiteralAI | Ten agents in a large 50x50 maze with ten boxes/goals; emphasizes long-distance routing and corridor planning over box volume. | If the ten tasks are separable, the level may become mostly single-agent pathfinding at scale. | **Very hard**: timed out/crashed. |
| MADS | Small three-agent/three-box layout with enough walls to require ordering but little noise. | It is very small and solved almost instantly, so it is mainly a smoke test. | **Easy**: solved in 0.085 server seconds with a 21-step solution. |
| MAGiC | Huge map, five colors, multiple components, and sparse task objects; useful for testing navigation and reachability. | Seven components can make the puzzle brittle if agents/boxes/goals are isolated in unintended ways. | **Very hard**: timed out/crashed. |
| MAchoupo2 | Two-agent, two-box level with meaningful maze geometry and four goals; good medium-sized coordination test. | Limited object count means difficulty mostly comes from path geometry, not rich box interactions. | **Medium**: solved in 2.479 server seconds with an 81-step solution. |
| MArtians | Massive 50x50 level with 1,167 boxes and multiple components; extreme stress test for parsing, assignment, and pruning. | So many boxes compared with only 37 goals risks overwhelming solvers with irrelevant movable objects. | **Very hard**: timed out/crashed. |
| MAybetwo | Very small two-component puzzle with three agents/boxes; useful for regression and color handling. | Too small to be interesting beyond basic correctness. | **Easy**: solved in 0.058 server seconds with a 19-step solution. |
| MAzonakis | Long rectangular maze with two agents and three boxes; forces sustained navigation in a constrained layout. | Low object count and no dead ends make it narrower in strategic variety. | **Medium**: solved in 1.917 server seconds with a 126-step solution. |
| Medibots | Large hospital-like space with ten agents, many boxes, and 55 goals; good multi-agent throughput challenge. | Only moderate corridor structure, so much of the difficulty may be assignment scale. | **Very hard**: timed out/crashed. |
| Minchia | Huge, open, and extremely dense with 1,378 boxes; a severe stress test for large object sets. | The box count is so high that it may be more pathological than illuminating for algorithm comparison. | **Very hard**: timed out/crashed. |
| NameHere | Large multi-room map with 486 boxes and 112 goals; tests both decomposition and dense object handling. | Five components and high clutter may hide the intended puzzle structure. | **Very hard**: timed out/crashed. |
| PFarthing | Six agents with only two boxes but many components and long routes; emphasizes reachability and agent positioning. | Low box count may make the level feel sparse once routing is understood. | **Hard**: unsolved after 27.886 server seconds despite a 152-step attempt. |
| PokeNOM | Huge level with 1,019 boxes, 25 components, and few goals; extreme object-noise and reachability challenge. | Like other massive box-text levels, it risks testing data volume more than planning quality. | **Very hard**: timed out/crashed. |
| QRscammer | Maze-like 37x37 level with many dead ends, ten agents, and 56 boxes; strong congestion and deadlock potential. | High dead-end/corridor density can make failures punishing and local. | **Hard-very hard**: unsolved after 75.143 server seconds. |
| Spiraling | Spiral/corridor-heavy design with ten colors and 21 boxes; excellent for testing long single-lane dependencies. | High corridor percentage may reduce branching once a solver finds the correct sequence. | **Hard-very hard**: unsolved after 130.719 server seconds after a 613-step attempt. |
| TBSTANS1 | Small, very corridor-heavy map with six agents and seven boxes; good compact collision benchmark. | Its size limits strategic depth and may overemphasize corridor shuffling. | **Medium-hard**: unsolved after 1.099 server seconds with 76 actions. |
| TeamAgent | Tiny three-agent corridor puzzle with six goals; useful for team/goal semantics and quick conflict checks. | Too small and too corridor-dominated to be a robust competition differentiator. | **Medium**: unsolved after 2.918 server seconds, but structurally small. |
| TheGate | Large single-component map with 91 boxes, 84 goals, and many corridor gates; strong bottleneck/throughput benchmark. | The scale may make diagnosing failures difficult without subproblem instrumentation. | **Very hard**: timed out/crashed. |
| TideMazes | Clean two-agent level spelling “TIDE MAZES” with one color and nine boxes; visually memorable and likely purposeful. | Not present in the benchmark summary, and single-color/two-agent structure may be simple if the maze is open enough. | **Unknown-medium**: static structure suggests tractable, but no benchmark result is available. |
| TourDeDTU | Large two-component map with nine agents and long maze routes; solved but slow, so it is a useful upper-mid benchmark. | Only 11 boxes on a big board can make much of the space feel like pathfinding overhead. | **Hard but solvable**: solved in 55.673 server seconds with a 151-step solution. |
| VNN | Four-agent/four-box symmetric room layout with repeated wall islands; concise and good for collision avoidance. | Symmetry and low object count make it less varied after the first solve. | **Medium**: solved in 6.033 server seconds with a 63-step solution. |
| ZOOM | Thematic text layout with large A block and many goals; visually distinctive and tests dense-object handling. | Six components and many same-letter boxes/goals may create noise or unreachable subregions. | **Medium-hard**: unsolved after 4.387 server seconds. |
| crayCray | Very corridor-heavy seven-agent puzzle with stacked lanes; strong compact test of lane swapping. | The layout is artificial and narrow, so it may reward specialized corridor tactics. | **Hard**: unsolved after 47.783 server seconds. |
| doggy | Nine agents with few boxes in a compact obstacle layout; emphasizes agent routing more than box pushing. | Only four boxes for nine agents can make many agents feel like moving obstacles. | **Very hard for current solver**: timed out/crashed. |
| donkeyK | Large multi-component, text/shape-heavy level with 231 boxes; mixes decorative density with real maze spaces. | Nineteen components and high object noise make intended interactions hard to read. | **Very hard**: timed out/crashed. |
| help | Moderate level with two colors, 20 boxes, and corridor-heavy interior; good focused bottleneck exercise. | Only two colors and text-like rows may limit strategic diversity. | **Very hard for current solver**: timed out/crashed. |
| jAIl | Wide jail-like level with six agents, 41 boxes, and long blocked bands; good for containment and escape routing. | Some dense A rows and two components may create clutter rather than meaningful choice. | **Hard-very hard**: unsolved after 112.796 server seconds. |
| merRAM | Large open maze with four agents and 16 boxes; many dead ends create navigation traps. | Huge open areas and only 16 boxes may dilute interaction density. | **Very hard**: timed out/crashed. |
| mosAIc | Huge mosaic-style level with 371 boxes, 78 components, and many patterned regions; strong decomposition stress test. | The very high component count can make it fragmented and potentially unfair if reachability is tricky. | **Very hard**: timed out/crashed. |
| noone | Very small three-agent/two-box puzzle with simple wall features; excellent smoke/regression test. | Not demanding; it should not be used to compare advanced solvers. | **Easy**: solved in 0.099 server seconds with a 12-step solution. |
| pacMAn | Thematic maze with central dense cluster and outer corridors; good for testing maze navigation around a box mass. | Five components and many repeated Z boxes may add decorative clutter. | **Hard-very hard**: unsolved after 116.162 server seconds. |
| pooh | Huge, mostly open map with two agents and decorative box letters; tests long-distance manipulation in sparse-agent setting. | Very low agent count and only two goals on a 50x50 board can make most of the map irrelevant. | **Very hard for current solver**: timed out/crashed. |
| rooMbA | Eight agents in a compact-to-large connected layout with rooms and corridors; good for congestion and room transitions. | Two components and moderate object count make it less extreme than the biggest levels. | **Very hard for current solver**: timed out/crashed. |
| sigMAS | Two-agent/two-color compact maze with 13 boxes and 15 goals; good solved benchmark with meaningful box work. | Small agent count limits multi-agent complexity. | **Medium-hard but solvable**: solved in 10.700 server seconds with a 171-step solution. |
| warehouse | Classic warehouse grid with pillars, five agents, and ten boxes; useful as a clean benchmark for throughput and aisle conflicts. | The regular pattern is less novel and may be easier for warehouse-specific heuristics. | **Medium-hard but solvable**: solved in 10.604 server seconds with a 173-step solution. |

## Suggested use

- Use **MADS**, **MAybetwo**, and **noone** as smoke tests.
- Use **Ccropper**, **MAchoupo2**, **MAzonakis**, **VNN**, **sigMAS**, and **warehouse** as normal regression tests.
- Use **TourDeDTU**, **QRscammer**, **Spiraling**, **jAIl**, and **pacMAn** as hard-but-informative stretch tests because the benchmark produced substantial partial attempts or a slow solve.
- Keep **Lily**, **MArtians**, **Minchia**, **PokeNOM**, **GroupEZ**, **mosAIc**, and **AMC** as stress tests rather than everyday regressions; their size/density can hide whether a solver failed due to planning, memory, or level noise.
