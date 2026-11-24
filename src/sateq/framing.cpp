#include "sateq/framing.h"

namespace sateq {

std::string build_command(char cmd, char axis, const std::string& arg)
{
    std::string s;
    s.reserve(4 + arg.size());
    s += LEAD;
    s += cmd;
    s += axis;
    s += arg;
    s += TERM;
    return s;
}

std::optional<std::string> ResponseAccumulator::feed(unsigned char c)
{
    if (c == static_cast<unsigned char>(TERM))
    {
        if (buf_.empty())
            return std::nullopt;  // stray CR noise, keep waiting
        std::string frame;
        frame.swap(buf_);
        return frame;
    }
    buf_ += static_cast<char>(c);
    return std::nullopt;
}

std::optional<std::string> ResponseAccumulator::feed(const std::string& bytes)
{
    std::optional<std::string> out;
    for (unsigned char c : bytes)
    {
        auto f = feed(c);
        if (f)
        {
            out = f;  // last complete frame in this chunk wins
        }
    }
    return out;
}

ParsedResponse parse_response(const std::string& frame)
{
    ParsedResponse r;
    if (frame.empty())
        return r;  // malformed
    if (frame[0] == ACK)
    {
        r.ok         = true;
        r.malformed  = false;
        r.payload    = frame.substr(1);
    }
    else if (frame[0] == NAK)
    {
        r.ok        = false;
        r.malformed = false;
        if (frame.size() >= 2)
        {
            char c = frame[1];
            r.nak_code = (c >= '0' && c <= '9') ? c - '0'
                       : (c >= 'A' && c <= 'F') ? 10 + (c - 'A')
                       : (c >= 'a' && c <= 'f') ? 10 + (c - 'a')
                                                : -1;
        }
    }
    return r;
}

} // namespace sateq
