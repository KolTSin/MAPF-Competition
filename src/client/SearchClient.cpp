#include "client/SearchClient.hpp"
#include "parser/LevelParser.hpp"
#include "solvers/SequentialSolver.hpp"

#include <iostream>

void SearchClient::run() {
    std::cout << "MJES" << '\n';
    std::cout.flush();
    ParsedLevel parsed = LevelParser::parse(std::cin);

    SequentialSolver solver;
    Plan plan = solver.solve(parsed.level, parsed.initial_state);

    for (const JointAction& joint_action : plan.steps) {
        std::cout << joint_action.to_string() << '\n';
        std::cout.flush();
        std::cerr << joint_action.to_string() << '\n';

        std::string response;
        if (!std::getline(std::cin, response)) {
            break;
        }

        std::cerr << "Server response: " << response << '\n';
    }
}