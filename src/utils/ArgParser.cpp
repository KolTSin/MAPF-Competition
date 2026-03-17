#include "utils/ArgParser.hpp"
#include "client/SearchConfig.hpp"
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

SolverType parseSolver(const std::string& s) {
    static const std::unordered_map<std::string, SolverType> table = {
        {"naive", SolverType::NAIVE},
        {"sequential", SolverType::SEQUENTIAL},
        {"spacetime_astar", SolverType::SPACETIME_ASTAR},
    };

    auto it = table.find(s);
    if (it == table.end()) {
        throw std::invalid_argument("Unknown solver: " + s);
    }
    return it->second;
}

} // namespace

SearchConfig parseArgs(int argc, char** argv) {
    SearchConfig cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            cfg.help = true;
        } else if (arg == "--verbose" || arg == "-v") {
            cfg.verbose = true;
        } else if (arg == "--solver") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--solver requires a value");
            }
            cfg.solver = parseSolver(argv[++i]);
        } else if (arg == "--heuristic") {
            if (i + 1 >= argc) {
                throw std::invalid_argument("--heuristic requires a value");
            }
            cfg.heuristic_name = argv[++i];
        } else {
            throw std::invalid_argument("Unknown argument: " + arg);
        }
    }

    return cfg;
}