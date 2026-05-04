#include "analysis/LevelAnalyzer.hpp"
#include "analysis/ParkingCellAnalyzer.hpp"
#include <queue>

namespace { constexpr int DR[4] = {-1, 1, 0, 0}; constexpr int DC[4] = {0, 0, -1, 1}; }

LevelAnalysis LevelAnalyzer::analyze(const Level& level, const State& state) const {
    LevelAnalysis analysis; analysis.rows = level.rows; analysis.cols = level.cols;
    analysis.cells.resize(static_cast<std::size_t>(analysis.rows * analysis.cols));

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            Position p{r, c}; if (level.is_wall(r, c)) continue;
            CellInfo& info = analysis.at(p); info.is_free_static = true; analysis.free_cells.push_back(p);
            int degree = 0;
            for (int i = 0; i < 4; ++i) {
                int nr = r + DR[i], nc = c + DC[i];
                if (!level.in_bounds(nr, nc) || level.is_wall(nr, nc)) continue;
                ++degree;
            }
            info.degree = degree; info.is_dead_end = degree <= 1; info.is_corridor = degree == 2; info.is_room = degree >= 3;
            char g = level.goal_at(r, c);
            if (g != '\0' && g != ' ') {
                info.is_goal_cell = true;
                info.is_box_goal_cell = (g >= 'A' && g <= 'Z');
                info.is_agent_goal_cell = (g >= '0' && g <= '9');
            }
        }
    }

    int component = 0;
    std::vector<bool> visited(static_cast<std::size_t>(level.rows * level.cols), false);
    for (const Position start : analysis.free_cells) {
        const int si = analysis.index(start); if (visited[static_cast<std::size_t>(si)]) continue;
        std::queue<Position> q; q.push(start); visited[static_cast<std::size_t>(si)] = true;
        while (!q.empty()) {
            Position cur = q.front(); q.pop(); analysis.at(cur).component_id = component;
            for (int i = 0; i < 4; ++i) {
                Position nxt{cur.row + DR[i], cur.col + DC[i]};
                if (!level.in_bounds(nxt.row, nxt.col) || level.is_wall(nxt.row, nxt.col)) continue;
                int ni = analysis.index(nxt); if (visited[static_cast<std::size_t>(ni)]) continue;
                visited[static_cast<std::size_t>(ni)] = true; q.push(nxt);
            }
        }
        ++component;
    }

    for (const Position p : analysis.free_cells) {
        CellInfo& info = analysis.at(p);
        info.is_chokepoint = info.is_dead_end || (info.is_corridor && info.degree <= 2);
        info.is_articulation = info.is_chokepoint;
        if (info.is_corridor) analysis.corridor_cells.push_back(p);
        if (info.is_room) analysis.room_cells.push_back(p);
        if (info.is_chokepoint) analysis.chokepoints.push_back(p);
    }

    ParkingCellAnalyzer parking;
    analysis.parking_cells = parking.find_parking_cells(level, state, analysis);
    for (const Position p : analysis.free_cells) analysis.at(p).parking_score = parking.score_parking_cell(p, level, state, analysis);
    return analysis;
}
