#include "parser/LevelParser.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>
#include "utils/StringUtils.hpp"

namespace {

bool is_agent_char(char ch) {
    return ch >= '0' && ch <= '9';
}

bool is_box_char(char ch) {
    return ch >= 'A' && ch <= 'Z';
}

bool is_goal_char(char ch) {
    return is_agent_char(ch) || is_box_char(ch);
}

std::string read_required_line(std::istream& in) {
    std::string line;
    if (!std::getline(in, line)) {
        throw std::runtime_error("Unexpected end of input");
    }

    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }

    return line;
}

void expect_exact(std::istream& in, const std::string& expected) {
    const std::string line = read_required_line(in);
    if (line != expected) {
        throw std::runtime_error("Expected '" + expected + "', got '" + line + "'");
    }
}

std::vector<std::string> read_section_lines_until_next_header(std::istream& in, std::string& next_header) {
    std::vector<std::string> lines;
    std::string line;

    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        if (!line.empty() && line[0] == '#') {
            next_header = line;
            return lines;
        }

        lines.push_back(line);
    }

    throw std::runtime_error("Unexpected end of input while reading section");
}

void parse_colors(Level& level, const std::vector<std::string>& color_lines) {
    for (const std::string& raw_line : color_lines) {
        const std::string line = str::trim_copy(raw_line);
        if (line.empty()) {
            continue;
        }

        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("Malformed color line: " + line);
        }

        const std::string color_name = str::trim_copy(line.substr(0, colon));
        const Color color = color_from_string(color_name);

        if (color == Color::Unknown) {
            throw std::runtime_error("Unknown color: " + color_name);
        }

        const std::string rhs = line.substr(colon + 1);
        std::size_t start = 0;

        while (start < rhs.size()) {
            const std::size_t comma = rhs.find(',', start);
            const std::string token = str::trim_copy(
                rhs.substr(start, comma == std::string::npos ? std::string::npos : comma - start)
            );

            if (!token.empty()) {
                const char c = token[0];

                if (is_agent_char(c)) {
                    level.agent_colors[c - '0'] = color;
                } else if (is_box_char(c)) {
                    level.box_colors[c - 'A'] = color;
                } else {
                    throw std::runtime_error("Invalid colored entity: " + token);
                }
            }

            if (comma == std::string::npos) {
                break;
            }
            start = comma + 1;
        }
    }
}

void initialize_level_and_state_storage(Level& level, State& state, int rows, int cols) {
    level.rows = rows;
    level.cols = cols;
    level.walls.assign(rows * cols, false);
    level.goals.assign(rows * cols, '\0');

    state.rows = rows;
    state.cols = cols;
    state.box_pos.assign(rows * cols, '\0');
}

void parse_initial_grid(const std::vector<std::string>& grid_lines, Level& level, State& state) {
    const int rows = static_cast<int>(grid_lines.size());
    int cols = 0;

    for (const auto& line : grid_lines) {
        cols = std::max(cols, static_cast<int>(line.size()));
    }

    initialize_level_and_state_storage(level, state, rows, cols);

    std::vector<bool> seen_agents(10, false);
    int max_agent_id = -1;

    for (int r = 0; r < rows; ++r) {
        const std::string& line = grid_lines[r];

        for (int c = 0; c < static_cast<int>(line.size()); ++c) {
            const char ch = line[c];

            if (ch == '+') {
                level.walls[level.index(r, c)] = true;
            } else if (is_agent_char(ch)) {
                const int id = ch - '0';
                seen_agents[id] = true;
                max_agent_id = std::max(max_agent_id, id);
            } else if (is_box_char(ch)) {
                state.set_box(r, c, ch);
            }
        }
    }

    if (max_agent_id >= 0) {
        state.agent_positions.assign(max_agent_id + 1, Position{-1, -1});

        for (int r = 0; r < rows; ++r) {
            const std::string& line = grid_lines[r];

            for (int c = 0; c < static_cast<int>(line.size()); ++c) {
                const char ch = line[c];

                if (is_agent_char(ch)) {
                    const int id = ch - '0';
                    state.agent_positions[id] = Position{r, c};
                }
            }
        }
    }

    for (int id = 0; id <= max_agent_id; ++id) {
        if (!seen_agents[id]) {
            throw std::runtime_error(
                "Agent ids must be contiguous from 0. Missing agent: " + std::to_string(id)
            );
        }
    }
}

void parse_goal_grid(const std::vector<std::string>& goal_lines, Level& level) {
    if (static_cast<int>(goal_lines.size()) != level.rows) {
        throw std::runtime_error("Goal grid row count does not match initial grid");
    }

    for (int r = 0; r < level.rows; ++r) {
        const std::string& line = goal_lines[r];

        for (int c = 0; c < static_cast<int>(line.size()); ++c) {
            const char ch = line[c];
            if (is_goal_char(ch)) {
                level.goals[level.index(r, c)] = ch;
            }
        }
    }
}

} // namespace

ParsedLevel LevelParser::parse(std::istream& in) {
    ParsedLevel parsed;

    expect_exact(in, "#domain");
    expect_exact(in, "hospital");

    expect_exact(in, "#levelname");
    parsed.level.name = read_required_line(in);

    expect_exact(in, "#colors");
    std::string next_header;
    const std::vector<std::string> color_lines = read_section_lines_until_next_header(in, next_header);
    parse_colors(parsed.level, color_lines);

    if (next_header != "#initial") {
        throw std::runtime_error("Expected #initial, got '" + next_header + "'");
    }
    const std::vector<std::string> initial_lines = read_section_lines_until_next_header(in, next_header);
    parse_initial_grid(initial_lines, parsed.level, parsed.initial_state);

    if (next_header != "#goal") {
        throw std::runtime_error("Expected #goal, got '" + next_header + "'");
    }
    const std::vector<std::string> goal_lines = read_section_lines_until_next_header(in, next_header);
    parse_goal_grid(goal_lines, parsed.level);

    if (next_header != "#end") {
        throw std::runtime_error("Expected #end, got '" + next_header + "'");
    }

    return parsed;
}