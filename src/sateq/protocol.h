// Protocol constants for the SkyWatcher/Synta motor-board command set.
// Character codes and semantics transcribed from indi-eqmod skywatcher.h
// (ref/indi-3rdparty/indi-eqmod), cross-checked against Sky-Watcher's own
// motor-controller doc ("How the motor controller control the motor speed",
// inter-static.skywatcher.com/downloads/skywatcher_motor_controller_command_set.pdf,
// the authoritative vendor spec for our 2.xx-firmware boards).
//
// NAK catalog (official, error frame = '!' + code + CR):
//   0 unknown command / not on this model   4 not initialized (no :F this
//   1 command length error                     power cycle — :F re-arms
//   2 motor not stopped (:G/:I-HIGH while    5 driver sleeping (":Bn1")
//     running)                                 7/8 PEC training / no PEC data
//   3 invalid (non-uppercase-hex) character
//
// WARNING: ':Q1 55AA' = "run bootloader, no response" (vendor doc) — a dead
// foot-gun; it is a WRITE and the board-layer gate keeps it queued forever.
#pragma once
#include <cstdint>
#include <string>

namespace sateq {

enum Cmd : char {
    Initialize              = 'F',  // arm axis after power-on (:J NAKs 4 until
                                    // sent); does NOT disturb the counter (live)
    InquireMCVersion        = 'e',  // payload 6 hex, raw low byte = mount code
    InquireGridPerRevolution= 'a',  // steps per 360 deg  (revu24)
    InquireTimerInterruptFreq= 'b', // steps per worm     (revu24)  [name is legacy]
    InquireHighSpeedRatio   = 'g',  // 2 hex chars
    InquirePecCycle         = 's',
    AxisStopInstant         = 'L',  // e-stop axis
    NotInstantAxisStop      = 'K',  // decelerate to stop
    SetAxisPositionCmd      = 'E',  // revu24
    GetAxisPosition         = 'j',  // revu24
    GetAxisStatus           = 'f',  // 3 hex: d1 slew/dir/speed bits, d2 running, d3 initialized
    SetSnapAuxMode          = 'O',  // EQ8 class
    SetMotionMode           = 'G',  // arg: 2 chars [mode][dir]
    SetGotoTargetIncrement  = 'H',  // revu24
    SetBreakPointIncrement  = 'M',  // revu24
    SetGotoTarget           = 'S',  // revu24 absolute
    SetBreakStep            = 'U',  // revu24
    SetStepPeriod           = 'I',  // revu24 period (us)
    StartMotion             = 'J',  // uses mode previously set by 'G'
    GetStepPeriod           = 'D',
    SleepWake               = 'B',  // NOT "activate": arg 0=wake 1=sleep
                                    // (vendor doc; NAK 5 = asleep). NEVER send
                                    // 1 — it powers down the stepper driver.
                                    // sateqd never uses this command.
    SetRAGuideRate          = 'P',
    GetHomePosition         = 'd',  // EQ8 class: home sensor encoder count
    SetFeatureCmd           = 'W',  // EQ8 class
    GetFeatureCmd           = 'q',  // EQ8 class probe: NAK '!0' on non-EQ8
    SetPolarLedCmd          = 'V',
};

enum Axis : char { AxisRA = '1', AxisDE = '2' };

// Read-only queries: these NEVER change board/motor state (pure inquiries).
// Everything else in the command set — including 'L'/'K' stops and 'F'/'E'
// counter writes — is a WRITE and is gated by Board::block_writes(). When a
// mount is powered on the bench and nobody has consented to motion, only
// commands passing this test may reach the wire.
inline bool is_query_command(char c)
{
    switch (c) {
        case InquireMCVersion:            // 'e'
        case InquireGridPerRevolution:    // 'a'
        case InquireTimerInterruptFreq:   // 'b'
        case InquireHighSpeedRatio:       // 'g'
        case InquirePecCycle:             // 's'
        case GetAxisPosition:             // 'j'
        case GetAxisStatus:               // 'f'
        case GetStepPeriod:               // 'D'
        case GetHomePosition:             // 'd'
        case GetFeatureCmd:               // 'q'
            return true;
        default:
            return false;
    }
}

// Motion-mode argument for 'G': [mode][dir]
// mode: '0' goto high, '1' slew low, '2' goto low, '3' slew high
// dir : '0' forward,  '1' backward
inline std::string motion_mode_arg(int mode, int dir); // implemented in board
enum MotionMode { GOTO_HIGH = '0', SLEW_LOW = '1', GOTO_LOW = '2', SLEW_HIGH = '3' };
enum Direction { FORWARD = '0', BACKWARD = '1' };

// Mount codes (MCVersion raw low byte, verified live on HDX=0x04, Sirius=0x01)
enum MountCode : uint8_t {
    MC_EQ6 = 0x00, MC_HEQ5 = 0x01, MC_EQ5 = 0x02, MC_EQ3 = 0x03,
    MC_EQ8 = 0x04, MC_AZEQ6 = 0x05, MC_AZEQ5 = 0x06,
    MC_STAR_ADVENTURER = 0x0A, MC_SA_GTI = 0x0C,
    MC_EQ8R_PRO = 0x20, MC_AZEQ6_PRO = 0x22, MC_EQ6R_PRO = 0x23,
    MC_EQ6_PRO = 0x24, MC_CQ350_PRO = 0x25, MC_EQ5_PRO = 0x31,
    MC_GT = 0x80, MC_MF = 0x81, MC_DOB = 0x90, MC_AZGTI = 0xA5,
};

inline bool is_eq8_class_features(uint8_t code)
{
    switch (code) {
        case MC_EQ8: case MC_AZEQ6: case MC_AZEQ5:
        case MC_STAR_ADVENTURER: case MC_SA_GTI:
        case MC_EQ6R_PRO: case MC_EQ6_PRO: case MC_AZGTI:
            return true;
        default: return false;
    }
}

inline const char* mount_code_name(uint8_t code)
{
    switch (code) {
        case MC_EQ6: return "EQ6";
        case MC_HEQ5: return "HEQ5";
        case MC_EQ5: return "EQ5";
        case MC_EQ3: return "EQ3";
        case MC_EQ8: return "EQ8";
        case MC_AZEQ6: return "AZEQ6";
        case MC_AZEQ5: return "AZEQ5";
        case MC_STAR_ADVENTURER: return "Star Adventurer";
        case MC_SA_GTI: return "SA-GTi";
        case MC_EQ8R_PRO: return "EQ8-R Pro";
        case MC_AZEQ6_PRO: return "AZEQ6 Pro";
        case MC_EQ6R_PRO: return "EQ6-R Pro";
        case MC_EQ6_PRO: return "EQ6 Pro";
        case MC_CQ350_PRO: return "CQ350 Pro";
        case MC_EQ5_PRO: return "EQ5 Pro";
        case MC_GT: return "GT (unsupported)";
        case MC_MF: return "MF (unsupported)";
        case MC_DOB: return "DOB (unsupported)";
        case MC_AZGTI: return "AZ-GTi";
        default: return "unknown";
    }
}

// Driver-observed rate limits (× sidereal). indi-eqmod MIN_RATE/MAX_RATE.
constexpr double MIN_RATE_X_SIDEREAL = 0.05;
constexpr double MAX_RATE_X_SIDEREAL = 800.0;
constexpr double LOWSPEED_RATE       = 128.0;  // above this: divide by hs-ratio, high mode
constexpr double STELLAR_DAY_SEC     = 86164.098903691;
constexpr double STELLAR_SPEED_ARCSEC_S = 15.041067179;
constexpr int    MIN_PERIOD_US_DEFAULT  = 6;

// Step period for a desired slew rate, EXACTLY as indi-eqmod computes it
// (period in us; >LOWSPEED_RATE folds in the high-speed ratio).
// Return value also reports whether high-speed motion mode is required.
struct SlewPeriod {
    uint32_t period_us;
    bool     use_high_speed;
};
SlewPeriod period_for_rate(double rate_x_sidereal, uint32_t steps_worm,
                           uint32_t steps_360, uint32_t hs_ratio);

} // namespace sateq
