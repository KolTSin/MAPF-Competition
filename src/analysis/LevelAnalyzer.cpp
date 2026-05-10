#include "analysis/LevelAnalyzer.hpp"
#include "analysis/ParkingCellAnalyzer.hpp"
#include <queue>

namespace {
// Shared four-neighbor movement offsets. Each index describes one cardinal
// step: up, down, left, or right. All topology in this analyzer is based on
// the same movement model used by agents and boxes.
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, -1, 1};
// Diagonal directions: NW, NE, SW, SE
static constexpr int DDR[4] = {-1, -1, 1, 1};
static constexpr int DDC[4] = {-1, 1, -1, 1};
}

LevelAnalysis LevelAnalyzer::analyze(const Level& level, const State& state) const {
    // Allocate a grid-shaped result. The cells vector has one CellInfo for
    // every coordinate, including walls; wall entries keep their default flags
    // while traversable entries are filled in by the scan below.
    LevelAnalysis analysis;
    analysis.rows = level.rows;
    analysis.cols = level.cols;
    analysis.cells.resize(static_cast<std::size_t>(analysis.rows * analysis.cols));

    // Pass 1: classify each non-wall cell using only local static geometry.
    //
    // Input for each coordinate: wall/goal data from Level and the four static
    // neighbors around the coordinate.
    // Output for each free cell: CellInfo flags for free space, degree,
    // corridor/dead-end/room shape, goal type, plus an entry in free_cells.
    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            Position p{r, c};
            if (level.is_wall(r, c)) continue;

            CellInfo& info = analysis.at(p);
            info.is_free_static = true;
            analysis.free_cells.push_back(p);

            // Degree is the number of traversable cardinal neighbors. It gives
            // a cheap, intuitive local shape label:
            //   0-1 neighbors: dead end
            //   2 neighbors: corridor-like passage
            //   3-4 neighbors: room/intersection with maneuvering space
            int degree = 0;
            int blocked_diagonals = 0;

            // Cardinal openness flags.
            // DR/DC order is currently: north, south, west, east.
            bool north_free = false;
            bool south_free = false;
            bool west_free = false;
            bool east_free = false;

            // Count free cardinal neighbors.
            for (int i = 0; i < 4; ++i) {
                const int nr = r + DR[i];
                const int nc = c + DC[i];

                const bool free =
                    level.in_bounds(nr, nc) &&
                    !level.is_wall(nr, nc);

                if (!free) continue;

                ++degree;

                if (DR[i] == -1 && DC[i] == 0) north_free = true;
                if (DR[i] ==  1 && DC[i] == 0) south_free = true;
                if (DR[i] ==  0 && DC[i] == -1) west_free = true;
                if (DR[i] ==  0 && DC[i] ==  1) east_free = true;
            }

            // Count blocked diagonals.
            // A diagonal is "not free" if it is out of bounds or a wall.
            for (int i = 0; i < 4; ++i) {
                const int nr = r + DDR[i];
                const int nc = c + DDC[i];

                if (!level.in_bounds(nr, nc) || level.is_wall(nr, nc)) {
                    ++blocked_diagonals;
                }
            }

            const bool vertical_corridor =
                north_free && south_free && !west_free && !east_free;

            const bool horizontal_corridor =
                west_free && east_free && !north_free && !south_free;

            const bool straight_corridor =
                degree == 2 && (vertical_corridor || horizontal_corridor);

            const bool corner =
                degree == 2 && !straight_corridor;

            const bool three_way_intersection = degree == 3 && blocked_diagonals >= 3;
            const bool four_way_intersection = degree == 4 && blocked_diagonals >= 3;

            // Your diagonal rule.
            // I would apply it only to straight corridor cells, not corners.
            const bool diagonal_intersection =
                straight_corridor && blocked_diagonals >= 2;

            info.degree = degree;
            info.blocked_diagonals = blocked_diagonals;

            info.is_dead_end = degree <= 1;
            info.is_corner = corner;

            info.is_three_way_intersection = three_way_intersection;
            info.is_four_way_intersection = four_way_intersection;

            info.is_intersection =
                three_way_intersection ||
                four_way_intersection;

            // Keep your old broad labels.
            info.is_corridor = degree == 2;
            info.is_room = (degree - blocked_diagonals) >= 3;

            // Goal cells are annotated separately because parking or blocking a
            // goal can make a future task harder. The raw goal character also
            // tells callers whether the goal belongs to a box or an agent.
            const char g = level.goal_at(r, c);
            if (g != '\0' && g != ' ') {
                info.is_goal_cell = true;
                info.is_box_goal_cell = (g >= 'A' && g <= 'Z');
                info.is_agent_goal_cell = (g >= '0' && g <= '9');
            }
        }
    }

    // Pass 2: assign connected-component ids with breadth-first search.
    //
    // Input: the free_cells discovered above and static wall layout.
    // Output: component_id for every free cell. Cells with the same id are
    // mutually reachable when dynamic agents and boxes are ignored.
    int component = 0;
    std::vector<bool> visited(static_cast<std::size_t>(level.rows * level.cols), false);
    for (const Position start : analysis.free_cells) {
        const int si = analysis.index(start);
        if (visited[static_cast<std::size_t>(si)]) continue;

        std::queue<Position> q;
        q.push(start);
        visited[static_cast<std::size_t>(si)] = true;

        while (!q.empty()) {
            const Position cur = q.front();
            q.pop();
            analysis.at(cur).component_id = component;

            for (int i = 0; i < 4; ++i) {
                Position nxt{cur.row + DR[i], cur.col + DC[i]};
                if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col)) continue;

                const int ni = analysis.index(nxt);
                if (visited[static_cast<std::size_t>(ni)]) continue;

                visited[static_cast<std::size_t>(ni)] = true;
                q.push(nxt);
            }
        }
        ++component;
    }

    // Pass 3: derive higher-level category lists from the per-cell labels.
    //
    // Chokepoints are currently approximated with low-degree geometry: dead
    // ends and corridor cells are places where an object is likely to block
    // flow. is_articulation mirrors this conservative chokepoint label so
    // downstream heuristics can treat it as a strong "do not park here" signal.
    for (const Position p : analysis.free_cells) {
        CellInfo& info = analysis.at(p);
        info.is_chokepoint = info.is_dead_end || (info.is_corridor && info.degree <= 2);
        info.is_articulation = info.is_chokepoint;

        if (info.is_corridor) analysis.corridor_cells.push_back(p);
        if (info.is_room) analysis.room_cells.push_back(p);
        if (info.is_chokepoint) analysis.chokepoints.push_back(p);
    }

    // Pass 4: score and rank temporary parking cells.
    //
    // Input: static topology from this analysis plus dynamic boxes from State.
    // Output: parking_cells sorted from most desirable to least desirable and
    // parking_score written back to every free cell for callers that need the
    // raw numeric heuristic.
    ParkingCellAnalyzer parking;
    analysis.parking_cells = parking.find_parking_cells(level, state, analysis);
    for (const Position p : analysis.free_cells) {
        analysis.at(p).parking_score = parking.score_parking_cell(p, level, state, analysis);
    }

    return analysis;
}
