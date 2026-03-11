#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace str {

    inline std::string trim_copy(std::string s) {
        auto not_space = [](unsigned char ch) { return !std::isspace(ch); };

        s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
        s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());

        return s;
    }

    inline std::string lowercase_copy(std::string s) {
        std::transform(
            s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        return s;
    }
}