// Zero-dependency test micro-framework. Each test_*.cpp has a main() that
// runs CHECK/REQUIRE-style macros; failure prints file:line and the values.
#pragma once
#include <cmath>
#include <cstdio>
#include <string>

namespace testkit {
inline int failures = 0;
inline int checks = 0;
inline const char* current = "";

inline void report(const char* expr, const char* file, int line, const std::string& extra = "")
{
    ++failures;
    std::fprintf(stderr, "FAIL [%s] %s:%d: %s %s\n", current, file, line, expr, extra.c_str());
}
// value rendering for failure messages
inline std::string dbg(const std::string& s) { return s; }
inline std::string dbg(const char* s) { return s ? s : "(null)"; }
inline std::string dbg(bool b) { return b ? "true" : "false"; }
template <class T>
inline std::string dbg(const T& v) { return std::to_string(v); }
inline int summary(const char* suite)
{
    if (failures)
    {
        std::fprintf(stderr, "%s: %d/%d checks FAILED\n", suite, failures, checks);
        return 1;
    }
    std::fprintf(stderr, "%s: all %d checks passed\n", suite, checks);
    return 0;
}
} // namespace testkit

#define TEST(name)                                   \
    do                                               \
    {                                                \
        testkit::current = name;                     \
        std::fprintf(stderr, "--- %s\n", name);      \
    } while (0)

#define CHECK(expr)                                  \
    do                                               \
    {                                                \
        ++testkit::checks;                           \
        if (!(expr)) testkit::report(#expr, __FILE__, __LINE__); \
    } while (0)

#define CHECK_EQ(a, b)                                                             \
    do                                                                             \
    {                                                                              \
        ++testkit::checks;                                                        \
        auto _va = (a);                                                           \
        auto _vb = (b);                                                           \
        if (!(_va == _vb))                                                         \
            testkit::report(#a " == " #b, __FILE__, __LINE__,                      \
                            "got " + testkit::dbg(_va) + " want " + testkit::dbg(_vb)); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                      \
    do                                                                             \
    {                                                                              \
        ++testkit::checks;                                                        \
        double _va = (a), _vb = (b);                                              \
        if (!(std::fabs(_va - _vb) <= (tol)))                                      \
            testkit::report(#a " ~= " #b, __FILE__, __LINE__,                      \
                            "got " + std::to_string(_va) + " want " + std::to_string(_vb)); \
    } while (0)

#define CHECK_THROWS(expr, ExcType)                             \
    do                                                          \
    {                                                           \
        ++testkit::checks;                                     \
        bool caught = false;                                    \
        try { (void)(expr); }                                   \
        catch (const ExcType&) { caught = true; }               \
        catch (...) {}                                          \
        if (!caught) testkit::report("throws " #ExcType ": " #expr, __FILE__, __LINE__); \
    } while (0)
