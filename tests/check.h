// A test runner small enough to read in one sitting.
//
// No framework, because the point of these tests is that they run anywhere the
// project builds, with nothing to install. Cases register themselves at static
// init and main() walks the list.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace check {

struct Case {
    const char *name;
    void (*fn)();
};

inline std::vector<Case> &cases()
{
    static std::vector<Case> v;
    return v;
}

inline int &failures()
{
    static int n = 0;
    return n;
}

inline const char *&current()
{
    static const char *n = "";
    return n;
}

inline void fail(const char *file, int line, const std::string &what)
{
    failures()++;
    std::printf("  FAIL %s:%d\n       %s\n", file, line, what.c_str());
}

struct Register {
    Register(const char *name, void (*fn)())
    {
        cases().push_back({name, fn});
    }
};

inline int run()
{
    for (const auto &c : cases()) {
        current() = c.name;
        const int before = failures();
        c.fn();
        std::printf("%-4s %s\n", failures() == before ? "ok" : "FAIL", c.name);
    }
    if (failures())
        std::printf("\n%d check(s) failed\n", failures());
    else
        std::printf("\nall %zu cases passed\n", cases().size());
    return failures() ? 1 : 0;
}

// Renders bytes the way the captures show them, so a failure can be matched
// against a dump by eye.
inline std::string hex(const uint8_t *p, size_t n)
{
    std::string s;
    char buf[4];
    for (size_t i = 0; i < n; i++) {
        std::snprintf(buf, sizeof(buf), "%02x", p[i]);
        s += buf;
        if (i + 1 < n)
            s += ' ';
    }
    return s;
}

}  // namespace check

#define TEST(name)                                                            \
    static void name();                                                       \
    static ::check::Register reg_##name(#name, name);                         \
    static void name()

#define CHECK(cond)                                                           \
    do {                                                                      \
        if (!(cond))                                                          \
            ::check::fail(__FILE__, __LINE__, "CHECK(" #cond ")");            \
    } while (0)

#define CHECK_EQ(a, b)                                                        \
    do {                                                                      \
        const auto _a = (a);                                                  \
        const auto _b = (b);                                                  \
        if (!(_a == _b))                                                      \
            ::check::fail(__FILE__, __LINE__,                                 \
                          std::string(#a " == " #b "  (")                     \
                              + std::to_string(_a) + " vs "                   \
                              + std::to_string(_b) + ")");                    \
    } while (0)
