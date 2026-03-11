#include "parser/LevelParser.hpp"
#include <stdexcept>
#include <string>
#include <vector>

namespace {
bool is_agent_char(char ch) {
    return ch >= '0' && ch <= '9';
}

bool is_box_char(char ch) {
    return ch >= 'A' && ch <= 'Z';
}

bool is_goal_char(char ch) {
    return (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z');
}
}

Level LevelParser::parse(std::istream& in) {
    Level level;
    std::vector<std::string> lines;
    std::string line;

    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (!line.empty()) {
            lines.push_back(line);
        }
    }

    if (lines.empty()) {
        throw std::runtime_error("Empty level input");
    }

    level.rows = static_cast<int>(lines.size());
    level.cols = 0;
    for (const auto& l : lines) {
        if (static_cast<int>(l.size()) > level.cols) {
            level.cols = static_cast<int>(l.size());
        }
    }

    level.grid.assign(level.rows, std::string(level.cols, ' '));

    for (int r = 0; r < level.rows; ++r) {
        for (int c = 0; c < static_cast<int>(lines[r].size()); ++c) {
            const char ch = lines[r][c];
            level.grid[r][c] = ch;

            if (is_agent_char(ch)) {
                level.agents.push_back(Agent{ch - '0', Color::Unknown, {r, c}});
            } else if (is_box_char(ch)) {
                level.boxes.push_back(Box{ch, Color::Unknown, {r, c}});
            } else if (is_goal_char(ch)) {
                level.goals.push_back(Goal{ch, {r, c}});
            }
        }
    }

    return level;
}