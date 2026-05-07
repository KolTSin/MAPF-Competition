// include/client/SearchClient.hpp
#pragma once

#include "client/SearchConfig.hpp"
#include "domain/Level.hpp"
#include "search/heuristics/HeuristicContext.hpp"
#include <memory>

class Solver;
class IHeuristic;

// Orchestrates parsing a level from stdin, constructing solver/heuristic components, and printing a plan.
class SearchClient {
public:
    explicit SearchClient(SearchConfig config);
    ~SearchClient();
    void run();

private:
    SearchConfig config_;
    std::unique_ptr<IHeuristic> heuristic_;
    std::unique_ptr<Solver> solver_;

    void initializeSearchComponents(const Level& level, const HeuristicContext& heuristic_context);
};