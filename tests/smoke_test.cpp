#include "hospital/HospitalTaskDecomposer.hpp"
#include "hospital/MapAnalysis.hpp"
#include "parser/LevelParser.hpp"

#include <iostream>
#include <optional>
#include <sstream>
#include <string>

namespace {
ParsedLevel parse_level(const std::string& level_text) {
    std::istringstream in(level_text);
    return LevelParser::parse(in);
}

std::optional<HospitalTask> find_relocation_for(const std::vector<HospitalTask>& tasks, const char box_symbol) {
    for (const HospitalTask& task : tasks) {
        if (task.type == HospitalTaskType::RelocateBox && task.box_symbol == box_symbol) {
            return task;
        }
    }
    return std::nullopt;
}

bool expect_true(const bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "[FAIL] " << message << '\n';
        return false;
    }
    return true;
}
}

int main() {
    const std::string level_text =
        "#domain\n"
        "hospital\n"
        "#levelname\n"
        "MAsimple3\n"
        "#colors\n"
        "red: 0, A\n"
        "green: 1, B\n"
        "#initial\n"
        "++++++++++++\n"
        "+  0     + +\n"
        "+ ++++++++A+\n"
        "+B         +\n"
        "++++++++++ +\n"
        "+1         +\n"
        "++++++++++++\n"
        "#goal\n"
        "++++++++++++\n"
        "+        +A+\n"
        "+ ++++++++ +\n"
        "+   B      +\n"
        "++++++++++ +\n"
        "+          +\n"
        "++++++++++++\n"
        "#end\n";

    ParsedLevel parsed = parse_level(level_text);
    MapAnalysis analysis(parsed.level);
    HospitalTaskDecomposer decomposer;
    const std::vector<HospitalTask> tasks = decomposer.decompose(parsed.level, parsed.initial_state, analysis);

    bool ok = true;

    const std::optional<HospitalTask> relocate_b = find_relocation_for(tasks, 'B');
    ok &= expect_true(relocate_b.has_value(), "Expected relocation task for blocking box B");
    if (relocate_b.has_value()) {
        ok &= expect_true(relocate_b->agent_id == 1, "Expected green agent 1 to be assigned to relocate box B");
        ok &= expect_true(
            relocate_b->destination.row == 2 && relocate_b->destination.col == 10,
            "Expected relocation destination for B to be nearest non-blocking cell (2,10)");
    }

    const std::optional<HospitalTask> relocate_a = find_relocation_for(tasks, 'A');
    ok &= expect_true(!relocate_a.has_value(), "Did not expect relocation task for non-blocking box A");

    const bool has_b_transport_dependency = [&] {
        for (const HospitalTask& task : tasks) {
            if (task.type == HospitalTaskType::TransportBox && task.box_symbol == 'B') {
                return true;
            }
        }
        return false;
    }();
    ok &= expect_true(has_b_transport_dependency, "Expected dependent transport task for box B");

    if (!ok) {
        return 1;
    }

    std::cout << "[PASS] smoke_test\n";
    return 0;
}
