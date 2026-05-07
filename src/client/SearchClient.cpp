#include "client/SearchClient.hpp"
#include "parser/LevelParser.hpp"
#include "solvers/SequentialSolver.hpp"
#include "search/heuristics/HeuristicFactory.hpp"
#include "solvers/SolverFactory.hpp"
#include "client/SearchClient.hpp"
#include "search/heuristics/HeuristicFactory.hpp"
#include "solvers/SolverFactory.hpp"

#include <iostream>

SearchClient::~SearchClient() = default;

SearchClient::SearchClient(SearchConfig config)
    : config_(std::move(config)) {}

void SearchClient::initializeSearchComponents(
    const Level& level,
    const HeuristicContext& heuristic_context
) {
    heuristic_ = HeuristicFactory::create(
        config_.heuristic_name,
        level,
        heuristic_context
    );

    solver_ = makeSolver(config_.solver);
}

void SearchClient::run() {
    std::cout << "MJES" << '\n';
    std::cout.flush();

    ParsedLevel parsed = LevelParser::parse(std::cin);
    HeuristicContext heuristic_context(parsed.level);
    initializeSearchComponents(parsed.level, heuristic_context);

    // SequentialSolver solver;
    Plan plan = solver_ -> solve(parsed.level, parsed.initial_state, *heuristic_);

    for (const JointAction& joint_action : plan.steps) {
        std::cout << joint_action.to_string() << '\n';
        std::cout.flush();
        std::cerr << joint_action.to_string() << '\n';

        std::string response;
        if (!std::getline(std::cin, response)) {
            break;
        }

        // std::cerr << "Server response: " << response << '\n';
    }
}