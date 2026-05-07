#pragma once

#include "domain/Level.hpp"
#include "domain/Position.hpp"
#include "state/State.hpp"

#include <array>
#include <vector>

// Precomputed static map data shared by all heuristic implementations. Building
// this once avoids repeating graph construction and BFS distance maps per state.
class HeuristicContext
{
public:
    static constexpr int INF = 1'000'000'000;
    static constexpr int MAX_BOX_TYPES = 26;
    static constexpr int MAX_AGENT_IDS = 10;

    explicit HeuristicContext(const Level& level);

    int rows() const noexcept { return rows_; }
    int cols() const noexcept { return cols_; }

    bool inBounds(int row, int col) const noexcept;
    bool isWall(int row, int col) const noexcept;
    char goalAt(int row, int col) const noexcept;

    int cellId(int row, int col) const noexcept;
    int cellId(const Position& pos) const noexcept;

    const Position& positionOf(int cellId) const noexcept;
    int rowOfCell(int cellId) const noexcept;
    int colOfCell(int cellId) const noexcept;

    const std::vector<int>& neighbors(int cellId) const noexcept;

    bool isDeadEndCell(int cellId) const noexcept;
    bool isCorridorCell(int cellId) const noexcept;

    const std::vector<int>& boxGoalCells(char boxLetter) const noexcept;
    const std::vector<int>& agentGoalCells(int agentId) const noexcept;

    int distanceToNearestBoxGoal(char boxLetter, int cellId) const noexcept;
    int distanceToAgentGoal(int agentId, int cellId) const noexcept;

    char boxAt(const State& state, int row, int col) const;
    int agentAt(const State& state, int row, int col) const;
    int numAgents(const State& state) const;
    Position agentPosition(const State& state, int agentId) const;

private:
    int rows_ = 0;
    int cols_ = 0;

    std::vector<bool> walls_;
    std::vector<char> goals_;

    // Mapping between dense free-cell ids and original grid coordinates.
    std::vector<int> cellIdOf_;
    std::vector<Position> positionOfCell_;
    std::vector<std::vector<int>> neighbors_;

    std::vector<int> degree_;
    std::vector<bool> deadEnd_;
    std::vector<bool> corridor_;

    std::array<std::vector<int>, MAX_BOX_TYPES> boxGoalCells_;
    std::array<std::vector<int>, MAX_AGENT_IDS> agentGoalCells_;

    // Multi-source BFS distance from every cell to the nearest matching goal.
    std::array<std::vector<int>, MAX_BOX_TYPES> distToBoxGoal_;
    std::array<std::vector<int>, MAX_AGENT_IDS> distToAgentGoal_;

    int flat(int row, int col) const noexcept;

    void buildStaticMap(const Level& level);
    void buildTopology();
    void buildGoalDistanceMaps();

    std::vector<int> multiSourceBfs(const std::vector<int>& sources) const;
};
