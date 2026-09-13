#pragma once

// The one DS_* switch left, and a word about every other.
//
// The relay used to take two dozen DS_* switches from the environment. They
// are gone: each now does what its default did. What is left is
// DS_FRESH_PAIRING, which scripts/run.sh reads and the binary never sees. Any
// other DS_* name in the environment is most likely one of the old ones set out
// of habit, so main() says it is ignored rather than letting a run look as if
// it had used it.

#include <algorithm>
#include <array>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <unistd.h>

inline constexpr std::array<std::string_view, 1> known_switches = {"DS_FRESH_PAIRING"};

// NAME=VALUE entries for every DS_* variable that is not a known switch.
inline std::vector<std::string> unknown_switches(std::span<const char *const> env)
{
    std::vector<std::string> unknown;
    for (const char *raw : env) {
        if (!raw)
            break;
        const std::string_view entry{raw};
        if (!entry.starts_with("DS_"))
            continue;
        const std::string_view name = entry.substr(0, entry.find('='));
        if (std::ranges::find(known_switches, name) == known_switches.end())
            unknown.emplace_back(entry);
    }
    std::ranges::sort(unknown);
    return unknown;
}

inline std::vector<std::string> unknown_switches()
{
    const char *const *env = environ;
    std::size_t n = 0;
    while (env && env[n])
        n++;
    return unknown_switches({env, n});
}
