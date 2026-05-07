#pragma once

#include "search/heuristics/Heuristic.hpp"
#include "search/heuristics/HeuristicContext.hpp"

#include "domain/Level.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

// Name-based registry so command-line heuristic choices stay decoupled from concrete classes.
class HeuristicFactory
{
public:
    using Creator = std::function<std::unique_ptr<IHeuristic>(
        const Level&,
        const HeuristicContext&
    )>;

    class Registrar
    {
    public:
        Registrar(const std::string& name, Creator creator);
    };

    static std::unique_ptr<IHeuristic> create(
        const std::string& name,
        const Level& level,
        const HeuristicContext& context
    );

    static std::vector<std::string> available();

private:
    static bool registerCreator(const std::string& name, Creator creator);
};

#define REGISTER_HEURISTIC(heuristic_name, heuristic_type)                     \
    namespace                                                                  \
    {                                                                          \
        const HeuristicFactory::Registrar heuristic_type##_registrar(          \
            heuristic_name,                                                    \
            [](const Level& level, const HeuristicContext& context) {          \
                return std::make_unique<heuristic_type>(level, context);       \
            }                                                                  \
        );                                                                     \
    }
