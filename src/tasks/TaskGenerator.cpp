#include "tasks/TaskGenerator.hpp"


bool TaskGenerator::is_box_goal(char goal_symbol) noexcept {
    return goal_symbol >= 'A' && goal_symbol <= 'Z';
}

bool TaskGenerator::can_agent_move_box(const Level& level, int agent_id, char box_id) {
    if (agent_id < 0 || agent_id >= static_cast<int>(level.agent_colors.size())) {
        return false;
    }
    if (box_id < 'A' || box_id > 'Z') {
        return false;
    }

    return level.agent_colors[agent_id] == level.box_colors[box_id - 'A'];
}

std::vector<Task> TaskGenerator::generate_delivery_tasks(const Level& level,
                                                         const State& state) const {
    std::vector<Task> tasks;
    int next_task_id = 0;

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < level.cols; ++c) {
            const char goal = level.goal_at(r, c);
            if (!is_box_goal(goal)) {
                continue;
            }

            const char existing_box = state.box_at(r, c);
            if (existing_box == goal) {
                continue;
            }

            Position box_pos{-1, -1};
            for (int br = 0; br < state.rows; ++br) {
                for (int bc = 0; bc < state.cols; ++bc) {
                    if (state.box_at(br, bc) == goal) {
                        box_pos = Position{br, bc};
                        break;
                    }
                }
                if (box_pos.row != -1) {
                    break;
                }
            }

            if (box_pos.row == -1) {
                continue;
            }

            int assigned_agent = -1;
            for (int agent_id = 0; agent_id < state.num_agents(); ++agent_id) {
                if (can_agent_move_box(level, agent_id, goal)) {
                    assigned_agent = agent_id;
                    break;
                }
            }

            if (assigned_agent == -1) {
                continue;
            }

            Task task;
            task.type = TaskType::DeliverBoxToGoal;
            task.task_id = next_task_id++;
            task.agent_id = assigned_agent;
            task.box_id = goal;
            task.box_pos = box_pos;
            task.goal_symbol = goal;
            task.goal_pos = Position{r, c};
            tasks.push_back(task);
        }
    }

    return tasks;
}
