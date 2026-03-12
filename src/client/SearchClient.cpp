#include "client/SearchClient.hpp"
#include "parser/LevelParser.hpp"
#include "solvers/NaiveSolver.hpp"

#include <iostream>

void SearchClient::run() {
    std::cout << "MJES" << '\n';
    std::cout.flush();
    ParsedLevel parsed = LevelParser::parse(std::cin);

    NaiveSolver solver;
    auto plan = solver.solve(parsed.level, parsed.initial_state);

    for (const auto& joint_action : plan.steps) {
        std::cout << joint_action.to_string() << '\n';
        std::cout.flush();

        std::string response;
        if (!std::getline(std::cin, response)) {
            break;
        }

        std::cerr << "Server response: " << response << '\n';
    }
}