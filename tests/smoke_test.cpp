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



std::optional<int> find_task_index(const std::vector<HospitalTask>& tasks, HospitalTaskType type, char box_symbol) {
    for (int i = 0; i < static_cast<int>(tasks.size()); ++i) {
        if (tasks[i].type == type && tasks[i].box_symbol == box_symbol) {
            return i;
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
        "+  0     +A+\n"
        "+ ++++++++ +\n"
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
            !(relocate_b->destination == relocate_b->source),
            "Expected relocation destination for B to differ from source");
    }

    const std::optional<HospitalTask> relocate_a = find_relocation_for(tasks, 'A');
    ok &= expect_true(!relocate_a.has_value(), "Did not expect relocation task for non-blocking box A");

    const std::optional<int> relocate_b_idx = find_task_index(tasks, HospitalTaskType::RelocateBox, 'B');
    const std::optional<int> transport_a_idx = find_task_index(tasks, HospitalTaskType::TransportBox, 'A');
    const std::optional<int> transport_b_idx = find_task_index(tasks, HospitalTaskType::TransportBox, 'B');

    ok &= expect_true(relocate_b_idx.has_value(), "Expected a relocation task index for B");
    ok &= expect_true(transport_b_idx.has_value(), "Expected dependent transport task for B");

    if (relocate_b_idx.has_value() && transport_b_idx.has_value()) {
        ok &= expect_true(*relocate_b_idx < *transport_b_idx, "Expected B relocation before restoring B");
    }

    if (transport_a_idx.has_value() && relocate_b_idx.has_value() && transport_b_idx.has_value()) {
        ok &= expect_true(*relocate_b_idx < *transport_a_idx, "Expected B relocation before A transport");
        ok &= expect_true(*transport_a_idx < *transport_b_idx, "Expected A transport before restoring B");
    }

    if (relocate_b.has_value()) {
        ok &= expect_true(relocate_b->phase == HospitalTaskPhase::ClearBlockers, "Expected relocation in ClearBlockers phase");
    }
    if (transport_a_idx.has_value()) {
        ok &= expect_true(tasks[*transport_a_idx].phase == HospitalTaskPhase::SolvePrimaryGoals, "Expected A transport in SolvePrimaryGoals phase");
    }
    if (transport_b_idx.has_value()) {
        ok &= expect_true(tasks[*transport_b_idx].phase == HospitalTaskPhase::RestoreRelocatedBoxes, "Expected B transport in RestoreRelocatedBoxes phase");
    }

    if (!ok) {
        return 1;
    }

    std::cout << "[PASS] smoke_test\n";
    return 0;
}
