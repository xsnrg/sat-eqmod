// Build identity (--version, startup log line, rotctld status header).
// SATEQ_VERSION / SATEQ_GIT come from the build system (CMake injects the
// project() version plus the git HEAD sha as of CONFIGURE time — edit a
// file and rebuild without reconfiguring still shows the old sha, which
// is fine for bench forensics: reconfigure with `cmake -B build`).
// The fallbacks keep bare-compiler builds working.
#pragma once
#include <string>

#ifndef SATEQ_VERSION
#define SATEQ_VERSION "dev"
#endif
#ifndef SATEQ_GIT
#define SATEQ_GIT "nogit"
#endif

namespace sateq {

inline std::string version_string()
{
    return std::string(SATEQ_VERSION) + " (git " + SATEQ_GIT +
           ", built " __DATE__ " " __TIME__ ")";
}

} // namespace sateq
