#pragma once

#include "domain/Level.hpp"
#include "state/State.hpp"
#include "tasks/Task.hpp"

#include <string>
#include <vector>

class TaskGenerator {
public:
    [[nodiscard]] std::vector<Task> generate_delivery_tasks(const Level& level,
                                                            const State& state);
    [[nodiscard]] const std::vector<std::string>& skip_reasons() const noexcept { return skip_reasons_; }

private:
    [[nodiscard]] static bool is_box_goal(char goal_symbol) noexcept;
    [[nodiscard]] static bool can_agent_move_box(const Level& level, int agent_id, char box_id);

    std::vector<std::string> skip_reasons_{};
};
