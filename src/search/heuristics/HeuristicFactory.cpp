#include "HeuristicFactory.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <unordered_map>

namespace
{
    std::unordered_map<std::string, HeuristicFactory::Creator>& registry()
    {
        static std::unordered_map<std::string, HeuristicFactory::Creator> instance;
        return instance;
    }

    std::string normalizeName(std::string name)
    {
        for (char& ch : name)
        {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            if (ch == '_')
                ch = '-';
        }
        return name;
    }
}

HeuristicFactory::Registrar::Registrar(const std::string& name, Creator creator)
{
    registerCreator(name, std::move(creator));
}

bool HeuristicFactory::registerCreator(const std::string& name, Creator creator)
{
    return registry().emplace(normalizeName(name), std::move(creator)).second;
}

std::unique_ptr<IHeuristic> HeuristicFactory::create(
    const std::string& name,
    const Level& level,
    const HeuristicContext& context
)
{
    const auto key = normalizeName(name);
    const auto it = registry().find(key);

    if (it == registry().end())
    {
        std::string msg = "Unknown heuristic '" + name + "'. Available: ";
        auto names = available();
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            if (i > 0)
                msg += ", ";
            msg += names[i];
        }
        throw std::runtime_error(msg);
    }

    return it->second(level, context);
}

std::vector<std::string> HeuristicFactory::available()
{
    std::vector<std::string> names;
    names.reserve(registry().size());

    for (const auto& [name, _] : registry())
        names.push_back(name);

    std::sort(names.begin(), names.end());
    return names;
}