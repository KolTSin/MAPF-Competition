#include <array>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <optional>
#include <regex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
std::optional<fs::path> find_levels_dir() {
    for (const fs::path& candidate : {fs::path{"levels"}, fs::path{"../levels"}, fs::path{"/levels"}}) {
        if (fs::exists(candidate) && fs::is_directory(candidate)) {
            return fs::canonical(candidate);
        }
    }
    return std::nullopt;
}

std::optional<fs::path> find_server_binary() {
    for (const fs::path& candidate : {fs::path{"./server.public"}, fs::path{"../server.public"}}) {
        if (fs::exists(candidate)) {
            return candidate;
        }
    }
    return std::nullopt;
}

std::optional<fs::path> find_client_binary() {
    for (const fs::path& candidate : {fs::path{"./searchclient"}, fs::path{"../build/searchclient"}}) {
        if (fs::exists(candidate)) {
            return candidate;
        }
    }
    return std::nullopt;
}

std::string run_command_capture_output(const std::string& command, int& exit_code) {
    std::array<char, 4096> buffer{};
    std::string output;

    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        exit_code = -1;
        return "Failed to launch command with popen.";
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        output += buffer.data();
    }

    const int status = pclose(pipe);
    exit_code = status;
    return output;
}
}

int main() {
    const std::optional<fs::path> levels_dir = find_levels_dir();
    const std::optional<fs::path> server_binary = find_server_binary();
    const std::optional<fs::path> client_binary = find_client_binary();

    if (!levels_dir.has_value()) {
        std::cerr << "[FAIL] Could not find levels directory.\n";
        return 1;
    }
    if (!server_binary.has_value()) {
        std::cerr << "[FAIL] Could not find server.public binary.\n";
        return 1;
    }
    if (!client_binary.has_value()) {
        std::cerr << "[FAIL] Could not find searchclient binary.\n";
        return 1;
    }

    const std::regex level_name_pattern("^MA[Ss]imple.*\\.lvl$");
    std::vector<fs::path> levels;
    for (const auto& entry : fs::directory_iterator(*levels_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string filename = entry.path().filename().string();
        if (std::regex_match(filename, level_name_pattern)) {
            levels.push_back(entry.path());
        }
    }

    std::sort(levels.begin(), levels.end());
    if (levels.size() != 5) {
        std::cerr << "[FAIL] Expected 5 MASimple*.lvl levels, found " << levels.size() << '\n';
        for (const fs::path& level : levels) {
            std::cerr << "  - " << level << '\n';
        }
        return 1;
    }

    bool all_solved = true;
    for (const fs::path& level : levels) {
        std::string command =
            "\"" + server_binary->string() + "\""
            + " \"" + level.string() + "\""
            + " -c \"" + client_binary->string() + " --solver task_driven_hospital --heuristic goal_count\" -g 2>&1";

        int exit_code = 0;
        const std::string output = run_command_capture_output(command, exit_code);
        const bool solved = output.find("Level solved: Yes.") != std::string::npos;

        std::cout << "[LEVEL] " << level.filename().string()
                  << " solved=" << (solved ? "Yes" : "No")
                  << " exit=" << exit_code
                  << '\n';

        if (!solved) {
            all_solved = false;
            std::cerr << "[FAIL] Level not solved: " << level << '\n';
            std::cerr << output << '\n';
        }
    }

    if (!all_solved) {
        return 1;
    }

    std::cout << "[PASS] masimple_levels_test\n";
    return 0;
}
