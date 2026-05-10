#include "analysis/ParkingCellAnalyzer.hpp"
#include <algorithm>
#include <iostream>

namespace {
// Four-neighbor offsets used to inspect the immediate surroundings of a
// candidate parking cell. Parking quality depends not only on the cell itself
// but also on whether parking there would sit beside goals or chokepoints.
constexpr int DR[4] = {-1, 1, 0, 0};
constexpr int DC[4] = {0, 0, -1, 1};

constexpr int kAgentPathCellPenalty = 5000;
constexpr int kAgentPathExtraVisitPenalty = 250;

int agent_path_penalty(Position p, const std::vector<AgentPlan>& initial_agent_plans) {
    int visits = 0;
    for (const AgentPlan& plan : initial_agent_plans) {
        for (const Position& pos : plan.positions) {
            if (pos == p) ++visits;
        }
    }

    if (visits == 0) return 0;
    return kAgentPathCellPenalty + (visits - 1) * kAgentPathExtraVisitPenalty;
}
}

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
    if (!level.in_bounds(p.row, p.col) || level.is_wall(p.row, p.col)) return -100000;
    if (state.has_box(p.row, p.col)) return -100000;

    const CellInfo& cell = analysis.at(p);
    if (cell.is_goal_cell) return -100000;

    // Base score: any empty non-goal free cell is potentially usable. Rooms get
    // a bonus because they usually have multiple ways to route around a parked
    // object, making them safer than narrow passages.
    int score = 0;
    score += 50;
    if (cell.is_room) score += 50000;
    if (cell.is_intersection) score -= 50000;

    // Topology penalties: the more a cell behaves like a bottleneck, the more
    // expensive it is to park there. Articulation/chokepoint penalties are large
    // enough to make those cells lose to ordinary open space even if they pass
    // the initial validity checks.
    if (cell.is_articulation) score -= 50000;
    if (cell.is_chokepoint) score -= 10000;
    if (cell.is_corridor) score -= 1000;
    if (cell.is_dead_end) score += 200;

    // Neighborhood scan: a cell can be locally valid but still risky if it sits
    // next to an important connector or a goal. We record whether any traversable
    // neighbor has those labels and award small bonuses only when the immediate
    // area is clear of those hazards.
    bool adjacent_chokepoint = false;
    bool adjacent_goal = false;
    for (int i = 0; i < 4; ++i) {
        Position n{p.row + DR[i], p.col + DC[i]};
        if (!level.in_bounds(n.row, n.col) || level.is_wall(n.row, n.col)) continue;

        const CellInfo& neighbor = analysis.at(n);
        adjacent_chokepoint = adjacent_chokepoint || neighbor.is_chokepoint;
        adjacent_goal = adjacent_goal || neighbor.is_goal_cell;
    }

    // Final local-safety bonuses. These do not override the hard exclusions or
    // bottleneck penalties; they simply prefer similarly good cells that are not
    // adjacent to fragile map features.
    if (!adjacent_chokepoint) score += 20;
    if (!adjacent_goal) score += 10;

    // Dynamic plan-awareness: a parking cell that lies on one of the already
    // planned agent trajectories is still legal, but it is less desirable
    // because parking a blocker there is likely to force CBS-style repair to
    // add waits or detours for that agent.
    score -= agent_path_penalty(p, initial_agent_plans);

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
        std::cerr << "pos: " << p.to_string() << " score: " << score << std::endl;
        if (score > -50000) scored.emplace_back(p, score);
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
