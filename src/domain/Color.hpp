#pragma once

#include <string>

enum class Color {
    Blue,
    Red,
    Green,
    Cyan,
    Magenta,
    Orange,
    Pink,
    Yellow,
    Unknown
};

inline std::string to_string(Color c) {
    switch (c) {
        case Color::Blue: return "blue";
        case Color::Red: return "red";
        case Color::Green: return "green";
        case Color::Cyan: return "cyan";
        case Color::Magenta: return "magenta";
        case Color::Orange: return "orange";
        case Color::Pink: return "pink";
        case Color::Yellow: return "yellow";
        default: return "unknown";
    }
}