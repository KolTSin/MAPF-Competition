#include "analysis/ParkingCellAnalyzer.hpp"
#include <algorithm>

namespace {
// Four-neighbor offsets used to inspect the immediate surroundings of a
// candidate parking cell. Parking quality depends not only on the cell itself
// but also on whether parking there would sit beside goals or chokepoints.
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, -1, 1};

constexpr int kAgentPathCellPenalty = 5000;
constexpr int kAgentPathExtraVisitPenalty = 250;
constexpr int kHardRejectScore = -100000;

int agent_path_penalty(int visits) {
    if (visits == 0) return 0;
    return kAgentPathCellPenalty + (visits - 1) * kAgentPathExtraVisitPenalty;
}

} // namespace

int ParkingCellAnalyzer::score_parking_cell(Position p, const Level& level, const State& state, const LevelAnalysis& analysis) const {
    static const std::vector<AgentPlan> kNoInitialAgentPlans;
    return score_parking_cell(p, level, state, analysis, kNoInitialAgentPlans);
}

int ParkingCellAnalyzer::score_parking_cell(Position p,
                                            const Level& level,
                                            const State& state,
                                            const LevelAnalysis& analysis,
                                            const std::vector<AgentPlan>& initial_agent_plans) const {
    // Hard rejection: cells outside the map, walls, occupied cells, and goals
    // are invalid parking spots. Returning the same very negative score keeps
    // them below every usable candidate while preserving a numeric API.
    if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) return kHardRejectScore;
    if (state.has_box(p.row, p.col)) return kHardRejectScore;

    const CellInfo& cell = analysis.at(p);
    if (cell.is_goal_cell) return kHardRejectScore;

    // Base score: any empty non-goal free cell may be needed as a fallback, but
    // isolated side pockets are much better parking than transit structures.
    int score = 5000;
    if (cell.is_room) score += 4000;
    if (cell.is_dead_end) score += 9000;
    if (cell.is_intersection) score -= 20000;

    // Topology penalties: corridors and connectors are often unavoidable in
    // narrow levels, so keep them rankable instead of collapsing them all into a
    // giant negative bucket. Dead ends are deliberately exempt from the connector
    // penalties because they are exactly the kind of low-traffic side pocket we
    // want to reward for temporary storage.
    if (!cell.is_dead_end) {
        if (cell.is_articulation) score -= 2500;
        if (cell.is_chokepoint) score -= 1500;
    }
    if (cell.is_corridor) score -= 1000;

    // Neighborhood scan: avoid parking next to goals and fragile connectors when
    // comparable alternatives exist, but do not punish true dead ends for being
    // adjacent to the corridor that enters them.
    bool adjacent_chokepoint = false;
    bool adjacent_goal = false;
    for (int i = 0; i < 4; ++i) {
        Position n{p.row + DR[i], p.col + DC[i]};
        if (!level.in_bounds(n.row, n.col) || level.is_wall(n.row, n.col)) continue;

        const CellInfo& neighbor = analysis.at(n);
        adjacent_chokepoint = adjacent_chokepoint || neighbor.is_chokepoint;
        adjacent_goal = adjacent_goal || neighbor.is_goal_cell;
    }

    if (adjacent_goal) score -= 2000;
    else score += 250;
    if (adjacent_chokepoint && !cell.is_dead_end) score -= 500;

    // Future route pressure: cells on likely future box deliveries or compatible
    // agent access paths are legal fallback positions, but they are poor parking
    // targets because they will probably have to be cleared again.
    const int route_visits = cell.future_route_visits;
    if (route_visits == 0) score += 2500;
    else score -= route_visits * 6000;

    // Prefer storage farther from current boxes and goals as a final tie-breaker;
    // this separates otherwise identical corridor cells in long narrow maps.
    score += std::min(cell.nearest_goal_or_box_distance, 12) * 50;

    // Dynamic plan-awareness: a parking cell that lies on one of the already
    // planned agent trajectories is still legal, but it is less desirable
    // because parking a blocker there is likely to force CBS-style repair to
    // add waits or detours for that agent.
    (void)initial_agent_plans;
    score -= agent_path_penalty(cell.agent_path_visits);

    return score;
}

std::vector<Position> ParkingCellAnalyzer::find_parking_cells(const Level& level, const State& state, const LevelAnalysis& analysis) const {
    static const std::vector<AgentPlan> kNoInitialAgentPlans;
    return find_parking_cells(level, state, analysis, kNoInitialAgentPlans);
}

std::vector<Position> ParkingCellAnalyzer::find_parking_cells(const Level& level,
                                                              const State& state,
                                                              const LevelAnalysis& analysis,
                                                              const std::vector<AgentPlan>& initial_agent_plans) const {
    // Score every statically free cell produced by LevelAnalyzer. The input list
    // already excludes walls, and score_parking_cell applies dynamic exclusions
    // such as current box occupancy.
    std::vector<std::pair<Position, int>> scored;
    scored.reserve(analysis.free_cells.size());
    for (const Position p : analysis.free_cells) {
        const int score = score_parking_cell(p, level, state, analysis, initial_agent_plans);
        // std::cerr << "pos: " << p.to_string() << " score: " << score << std::endl;
        if (score > 0) scored.emplace_back(p, score);
    }

    // Highest scores should be tried first by callers. Ties intentionally keep
    // the default sort behavior unspecified because all equal-scoring cells are
    // considered equally safe by this heuristic.
    std::sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.second > b.second; });

    // Return only positions. The numeric score remains available through
    // LevelAnalysis::CellInfo::parking_score when LevelAnalyzer populates it.
    std::vector<Position> result;
    result.reserve(scored.size());
    for (const auto& [p, _] : scored) result.push_back(p);

    return result;
}
