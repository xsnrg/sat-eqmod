#include "sateq/config.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace sateq {

namespace {

std::string trim(const std::string& s)
{
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

// strip a trailing "# comment" that is separated from the value by whitespace
// (so "path#1.conf" style values survive intact)
std::string strip_comment(const std::string& s)
{
    for (size_t i = 1; i < s.size(); ++i)
        if (s[i] == '#' && (s[i - 1] == ' ' || s[i - 1] == '\t'))
            return s.substr(0, i);
    return s;
}

} // namespace

bool Config::parse_line(const std::string& line, const std::string& origin)
{
    std::string t = trim(line);
    if (t.empty() || t[0] == '#')
        return true;

    if (t[0] == '[')
    {
        size_t close = t.find(']');
        if (close == std::string::npos)
        {
            warnings_.push_back(origin + ": unterminated [section] in: " + t);
            return false;
        }
        section_ = trim(t.substr(1, close - 1));
        return true;
    }

    t = strip_comment(t);
    size_t eq = t.find('=');
    if (eq == std::string::npos)
    {
        warnings_.push_back(origin + ": malformed line (expected key = value): " + t);
        return false;
    }
    std::string k = trim(t.substr(0, eq));
    std::string v = trim(t.substr(eq + 1));
    if (k.empty())
    {
        warnings_.push_back(origin + ": empty key in: " + t);
        return false;
    }
    if (!section_.empty() && k.rfind(section_ + ".", 0) != 0)
        k = section_ + "." + k;
    kv_[k] = v; // within one file, later duplicate wins
    return true;
}

bool Config::load(const std::string& path)
{
    std::ifstream f(path);
    if (!f)
    {
        warnings_.push_back("cannot open config: " + path);
        return false;
    }
    section_.clear(); // a trailing [section] from a previous file must not leak
    std::string line;
    while (std::getline(f, line))
        parse_line(line, path);
    section_.clear(); // and must not leak into the next file either
    return true;
}

void Config::merge_defaults(const Config& other)
{
    for (auto& kv : other.kv_)
        if (!kv_.count(kv.first))
            kv_[kv.first] = kv.second;
}

bool Config::has(const std::string& key) const
{
    bool found = kv_.count(key) != 0;
    if (found)
        note_used(key);
    return found;
}

std::string Config::str(const std::string& key, const std::string& dflt) const
{
    auto it = kv_.find(key);
    if (it == kv_.end())
        return dflt;
    note_used(key);
    return it->second;
}

double Config::num(const std::string& key, double dflt) const
{
    auto it = kv_.find(key);
    if (it == kv_.end())
        return dflt;
    note_used(key);
    try
    {
        size_t used = 0;
        double v = std::stod(it->second, &used);
        if (used != it->second.size())
            throw std::invalid_argument("trailing garbage");
        return v;
    }
    catch (const std::exception&)
    {
        throw std::runtime_error("config: bad numeric value '" + it->second + "' for key " + key);
    }
}

int Config::integer(const std::string& key, int dflt) const
{
    auto it = kv_.find(key);
    if (it == kv_.end())
        return dflt;
    note_used(key);
    try
    {
        size_t used = 0;
        long v = std::stol(it->second, &used);
        if (used != it->second.size())
            throw std::invalid_argument("trailing garbage");
        return static_cast<int>(v);
    }
    catch (const std::exception&)
    {
        throw std::runtime_error("config: bad integer value '" + it->second + "' for key " + key);
    }
}

bool Config::flag(const std::string& key, bool dflt) const
{
    auto it = kv_.find(key);
    if (it == kv_.end())
        return dflt;
    note_used(key);
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    if (v == "yes" || v == "true" || v == "1" || v == "on")  return true;
    if (v == "no" || v == "false" || v == "0" || v == "off") return false;
    throw std::runtime_error("config: bad boolean value '" + it->second + "' for key " + key);
}

void Config::note_used(const std::string& key) const
{
    if (std::find(used_.begin(), used_.end(), key) == used_.end())
        used_.push_back(key);
}

std::vector<std::string> Config::unused_keys() const
{
    std::vector<std::string> out;
    for (auto& kv : kv_)
        if (std::find(used_.begin(), used_.end(), kv.first) == used_.end())
            out.push_back(kv.first);
    return out;
}

} // namespace sateq
