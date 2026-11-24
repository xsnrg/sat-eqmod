// Wire framing for the SkyWatcher motor-board protocol.
// Frames: ':' + cmd-char + axis-digit('1'|'2') + optional arg + CR (0x0d).
// Responses: CR-terminated; first char '=' OK (payload after it), '!' + code
// = NAK. There is no '#' terminator anywhere (LX200 habit is fatal here).
#pragma once
#include <optional>
#include <string>

namespace sateq {

constexpr char LEAD = ':';
constexpr char TERM = '\r';
constexpr char ACK  = '=';
constexpr char NAK  = '!';

// Build one framed command string (with trailing CR), e.g.
//  build_command('e', '1', "")       -> ":e1\r"
//  build_command('I', '1', "00B289") -> ":I100B289\r"
std::string build_command(char cmd, char axis, const std::string& arg);

// Incremental response collector: feed bytes as they arrive; feed() returns
// the complete frame (ACK/NAK + payload, no CR) exactly when a CR terminates
// it. If CR arrives with an empty buffer (stray line noise), it is dropped
// and feed() keeps waiting.
class ResponseAccumulator {
public:
    std::optional<std::string> feed(unsigned char c);
    std::optional<std::string> feed(const std::string& bytes);
    void reset() { buf_.clear(); }
    const std::string& partial() const { return buf_; }

private:
    std::string buf_;
};

// Classification of a complete response frame (no CR).
struct ParsedResponse {
    bool ok = false;
    std::string payload;   // after '=' when ok
    int nak_code = -1;     // when !ok and NAK
    bool malformed = true; // true when frame is empty or unparsable
};
ParsedResponse parse_response(const std::string& frame);

} // namespace sateq
