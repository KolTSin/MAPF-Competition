#pragma once

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include "utils/StringUtils.hpp"

enum class Color {
    Blue,
    Red,
    Cyan,
    Purple,
    Green,
    Orange,
    Pink,
    Grey,
    Lightblue,
    Brown,
    Unknown
};

inline std::string to_string(Color c) {
    switch (c) {
        case Color::Blue: return "blue";
        case Color::Red: return "red";
        case Color::Cyan: return "cyan";
        case Color::Purple: return "purple";
        case Color::Green: return "green";
        case Color::Orange: return "orange";
        case Color::Pink: return "pink";
        case Color::Grey: return "grey";
        case Color::Lightblue: return "lightblue";
        case Color::Brown: return "brown";
        default: return "unknown";
    }
}

inline Color color_from_string(const std::string& raw) {
    const std::string s = str::lowercase_copy(str::trim_copy(raw));

    if (s == "blue") return Color::Blue;
    if (s == "red") return Color::Red;
    if (s == "cyan") return Color::Cyan;
    if (s == "purple") return Color::Purple;
    if (s == "green") return Color::Green;
    if (s == "orange") return Color::Orange;
    if (s == "pink") return Color::Pink;
    if (s == "grey") return Color::Grey;
    if (s == "lightblue") return Color::Lightblue;
    if (s == "brown") return Color::Brown;

    return ;
}