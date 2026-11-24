// Byte-order helpers for the SkyWatcher/Synta motor-board protocol.
// Exact port of indi-eqmod skywatcher.cpp (lines ~1917-1953) so that we
// produce and consume byte-for-byte identical frames.
#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>

namespace sateq {

inline int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    throw std::invalid_argument(std::string("bad hex digit: ") + c);
}

// Parse 6 hex chars with byte pairs swapped: "A7FD00" -> 0x00FDA7 (=64935).
// This is the canonical "reversed u24" encoding for :a :b :E :j :I :S :H :M :U
// payloads in BOTH directions (responses and command arguments).
inline uint32_t revu24_parse(const char s[6])
{
    uint32_t res = static_cast<uint32_t>(hex_nibble(s[4]));
    res = (res << 4) | static_cast<unsigned>(hex_nibble(s[5]));
    res = (res << 4) | static_cast<unsigned>(hex_nibble(s[2]));
    res = (res << 4) | static_cast<unsigned>(hex_nibble(s[3]));
    res = (res << 4) | static_cast<unsigned>(hex_nibble(s[0]));
    res = (res << 4) | static_cast<unsigned>(hex_nibble(s[1]));
    return res;
}

// Inverse of revu24_parse: value -> 6 hex chars, byte pairs swapped.
inline void revu24_format(uint32_t n, char str[7])
{
    const char* hexa = "0123456789ABCDEF";
    str[0] = hexa[(n & 0xF0) >> 4];
    str[1] = hexa[(n & 0x0F)];
    str[2] = hexa[(n & 0xF000) >> 12];
    str[3] = hexa[(n & 0x0F00) >> 8];
    str[4] = hexa[(n & 0xF00000) >> 20];
    str[5] = hexa[(n & 0x0F0000) >> 16];
    str[6] = '\0';
}

// Motor board version payload: 6 hex chars, raw low byte = mount code.
// (Verified live: HDX-110 "=020B04" -> code 0x04 EQ8;
//  Sirius "=020401" -> code 0x01 HEQ5.)
inline uint8_t mount_code_from_version_payload(const char s[6])
{
    return static_cast<uint8_t>(hex_nibble(s[4]) << 4 | hex_nibble(s[5]));
}

// :g high-speed ratio: plain 2 hex chars.
inline uint32_t highstr_parse(const char s[2])
{
    return static_cast<uint32_t>(hex_nibble(s[0]) << 4 | hex_nibble(s[1]));
}

} // namespace sateq
