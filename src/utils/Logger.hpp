#pragma once

#include <iostream>
#include <string>

namespace Logger {
inline void info(const std::string& msg) {
    std::cerr << "[INFO] " << msg << '\n';
}

inline void warn(const std::string& msg) {
    std::cerr << "[WARN] " << msg << '\n';
}
}