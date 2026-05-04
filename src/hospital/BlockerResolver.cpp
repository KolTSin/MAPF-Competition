#include "hospital/BlockerResolver.hpp"

std::vector<Task> BlockerResolver::generate_blocker_tasks(const Level& level,
                                                          const State& state,
                                                          const LevelAnalysis& analysis,
                                                          int& next_task_id) const {
    std::vector<Task> tasks;
    if (analysis.parking_cells.empty()) return tasks;

    for (int r = 0; r < state.rows; ++r) {
        for (int c = 0; c < state.cols; ++c) {
            const char b = state.box_at(r, c);
            if (b == '\0') continue;
            if (level.goal_at(r, c) != '\0') continue;

            Task t;
            t.type = TaskType::MoveBlockingBoxToParking;
            t.task_id = next_task_id++;
            t.box_id = b;
            t.box_pos = Position{r, c};
            t.parking_pos = analysis.parking_cells.front();
            t.goal_pos = t.parking_pos;
            t.agent_id = 0;
            tasks.push_back(t);
        }
    }
    return tasks;
}
