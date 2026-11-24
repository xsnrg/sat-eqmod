// Layered key=value configuration for sateqd sessions.
//
// File format (shared by station config and mount profiles):
//   # comment, or trailing after value
//   key = value
//   [section]           <- switches the active section; keys inside are
//                          stored prefixed as "section.key" (a key already
//                          starting with "section." is stored as written)
// A Config is a flat string map plus typed accessor helpers with defaults,
// and "was set" queries so callers can distinguish an operator-chosen value
// from a built-in default (e.g. park pose defaults depend on orientation).
//
// Precedence: load the HIGHEST-precedence source first, then pull the rest
// in with merge_defaults() (first value wins). Unused-key detection via
// unused_keys() surfaces typos and stale profile entries at startup.
#pragma once
#include <map>
#include <string>
#include <vector>

namespace sateq {

class Config
{
public:
    // Parse one "key = value" or "[section]" line. `origin` is only used in
    // diagnostics. Returns false on a malformed line (warns, does not throw).
    bool parse_line(const std::string& line, const std::string& origin);

    // Parse a whole file. Returns false if it cannot be opened; parse errors
    // warn line-by-line but do not fail the load.
    bool load(const std::string& path);

    // Copy every key from `other` that is not already present (first value
    // wins: load highest-precedence source first).
    void merge_defaults(const Config& other);

    bool has(const std::string& key) const;
    std::string str(const std::string& key, const std::string& dflt = "") const;
    double num(const std::string& key, double dflt) const;         // throws on garbage
    int integer(const std::string& key, int dflt) const;
    bool flag(const std::string& key, bool dflt) const;            // yes/no/true/false/1/0

    // Diagnostics collected during parsing (caller decides where to print).
    const std::vector<std::string>& warnings() const { return warnings_; }

    // Keys that were read via accessors (for startup "unused key" checks —
    // a key nobody consumed is usually a typo or a stale profile).
    void note_used(const std::string& key) const;
    std::vector<std::string> unused_keys() const;

private:
    std::map<std::string, std::string> kv_;
    std::string section_;                       // "" = top level
    std::vector<std::string> warnings_;
    mutable std::vector<std::string> used_;
};

} // namespace sateq
