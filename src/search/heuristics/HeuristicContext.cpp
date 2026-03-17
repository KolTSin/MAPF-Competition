#include "HeuristicContext.hpp"

#include <queue>
#include <stdexcept>

namespace
{
    // Adjust ONLY these if Position uses different field names.
    int rowOf(const Position& p)
    {
        // Example alternatives:
        // return p.row;
        // return p.r;
        return p.row;
    }

    int colOf(const Position& p)
    {
        // Example alternatives:
        // return p.col;
        // return p.c;
        return p.col;
    }

    Position makePosition(int row, int col)
    {
        // If aggregate init works:
        return Position{row, col};

        // If not, adapt here.
    }

    bool isBoxGoalChar(char ch)
    {
        return ch >= 'A' && ch <= 'Z';
    }

    bool isAgentGoalChar(char ch)
    {
        return ch >= '0' && ch <= '9';
    }

    int boxIndex(char ch)
    {
        return ch - 'A';
    }

    int agentIndex(char ch)
    {
        return ch - '0';
    }
}

HeuristicContext::HeuristicContext(const Level& level)
{
    buildStaticMap(level);
    buildTopology();
    buildGoalDistanceMaps();
}

bool HeuristicContext::inBounds(int row, int col) const noexcept
{
    return 0 <= row && row < rows_ && 0 <= col && col < cols_;
}

int HeuristicContext::flat(int row, int col) const noexcept
{
    return row * cols_ + col;
}

bool HeuristicContext::isWall(int row, int col) const noexcept
{
    return walls_[flat(row, col)];
}

char HeuristicContext::goalAt(int row, int col) const noexcept
{
    return goals_[flat(row, col)];
}

int HeuristicContext::cellId(int row, int col) const noexcept
{
    return cellIdOf_[flat(row, col)];
}

int HeuristicContext::cellId(const Position& pos) const noexcept
{
    return cellId(rowOf(pos), colOf(pos));
}

const Position& HeuristicContext::positionOf(int cellId) const noexcept
{
    return positionOfCell_[cellId];
}

int HeuristicContext::rowOfCell(int cellId) const noexcept
{
    return rowOf(positionOfCell_[cellId]);
}

int HeuristicContext::colOfCell(int cellId) const noexcept
{
    return colOf(positionOfCell_[cellId]);
}

const std::vector<int>& HeuristicContext::neighbors(int cellId) const noexcept
{
    return neighbors_[cellId];
}

bool HeuristicContext::isDeadEndCell(int cellId) const noexcept
{
    return deadEnd_[cellId];
}

bool HeuristicContext::isCorridorCell(int cellId) const noexcept
{
    return corridor_[cellId];
}

const std::vector<int>& HeuristicContext::boxGoalCells(char boxLetter) const noexcept
{
    static const std::vector<int> empty;
    if (boxLetter < 'A' || boxLetter > 'Z')
        return empty;
    return boxGoalCells_[boxIndex(boxLetter)];
}

const std::vector<int>& HeuristicContext::agentGoalCells(int agentId) const noexcept
{
    static const std::vector<int> empty;
    if (agentId < 0 || agentId >= MAX_AGENT_IDS)
        return empty;
    return agentGoalCells_[agentId];
}

int HeuristicContext::distanceToNearestBoxGoal(char boxLetter, int cellId) const noexcept
{
    if (cellId < 0 || boxLetter < 'A' || boxLetter > 'Z')
        return INF;

    return distToBoxGoal_[boxIndex(boxLetter)][cellId];
}

int HeuristicContext::distanceToAgentGoal(int agentId, int cellId) const noexcept
{
    if (cellId < 0 || agentId < 0 || agentId >= MAX_AGENT_IDS)
        return INF;

    return distToAgentGoal_[agentId][cellId];
}

char HeuristicContext::boxAt(const State& state, int row, int col) const
{
    return state.box_at(row, col);
}

int HeuristicContext::agentAt(const State& state, int row, int col) const
{
    const int n = static_cast<int>(state.agent_positions.size());
    for (int agentId = 0; agentId < n; ++agentId)
    {
        const Position& p = state.agent_positions[agentId];
        if (rowOf(p) == row && colOf(p) == col)
            return agentId;
    }
    return -1;
}

int HeuristicContext::numAgents(const State& state) const
{
    return state.num_agents();
}

Position HeuristicContext::agentPosition(const State& state, int agentId) const
{
    return state.agent_positions[agentId];
}

void HeuristicContext::buildStaticMap(const Level& level)
{
    rows_ = level.rows;
    cols_ = level.cols;

    walls_.assign(rows_ * cols_, false);
    goals_.assign(rows_ * cols_, '\0');
    cellIdOf_.assign(rows_ * cols_, -1);
    positionOfCell_.clear();

    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            const int f = flat(row, col);

            walls_[f] = level.is_wall(row, col);
            goals_[f] = level.goal_at(row, col);

            if (!walls_[f])
            {
                cellIdOf_[f] = static_cast<int>(positionOfCell_.size());
                positionOfCell_.push_back(makePosition(row, col));
            }
        }
    }

    neighbors_.assign(positionOfCell_.size(), {});
    degree_.assign(positionOfCell_.size(), 0);
    deadEnd_.assign(positionOfCell_.size(), false);
    corridor_.assign(positionOfCell_.size(), false);

    for (auto& v : boxGoalCells_) v.clear();
    for (auto& v : agentGoalCells_) v.clear();

    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            if (isWall(row, col))
                continue;

            const int id = cellId(row, col);
            const char g = goalAt(row, col);

            if (isBoxGoalChar(g))
                boxGoalCells_[boxIndex(g)].push_back(id);
            else if (isAgentGoalChar(g))
                agentGoalCells_[agentIndex(g)].push_back(id);
        }
    }
}

void HeuristicContext::buildTopology()
{
    static constexpr int DR[4] = {-1, 1, 0, 0};
    static constexpr int DC[4] = {0, 0, -1, 1};

    for (int id = 0; id < static_cast<int>(positionOfCell_.size()); ++id)
    {
        const int row = rowOfCell(id);
        const int col = colOfCell(id);

        for (int k = 0; k < 4; ++k)
        {
            const int nr = row + DR[k];
            const int nc = col + DC[k];

            if (!inBounds(nr, nc) || isWall(nr, nc))
                continue;

            const int nid = cellId(nr, nc);
            if (nid >= 0)
                neighbors_[id].push_back(nid);
        }

        degree_[id] = static_cast<int>(neighbors_[id].size());
        deadEnd_[id] = (degree_[id] <= 1);
        corridor_[id] = (degree_[id] <= 2);
    }
}

std::vector<int> HeuristicContext::multiSourceBfs(const std::vector<int>& sources) const
{
    std::vector<int> dist(positionOfCell_.size(), INF);
    std::queue<int> q;

    for (int s : sources)
    {
        if (s < 0 || s >= static_cast<int>(positionOfCell_.size()))
            continue;

        if (dist[s] == 0)
            continue;

        dist[s] = 0;
        q.push(s);
    }

    while (!q.empty())
    {
        const int u = q.front();
        q.pop();

        for (int v : neighbors_[u])
        {
            if (dist[v] != INF)
                continue;

            dist[v] = dist[u] + 1;
            q.push(v);
        }
    }

    return dist;
}

void HeuristicContext::buildGoalDistanceMaps()
{
    for (int i = 0; i < MAX_BOX_TYPES; ++i)
        distToBoxGoal_[i] = multiSourceBfs(boxGoalCells_[i]);

    for (int i = 0; i < MAX_AGENT_IDS; ++i)
        distToAgentGoal_[i] = multiSourceBfs(agentGoalCells_[i]);
}