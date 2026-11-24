// Minimal Hamlib rotctld server compatible with the NET-rotctl client
// (rotctl -m 2, GPredict). Wire protocol VERIFIED empirically against
// hamlib 4.7.2 (2026-11-15):
//   client opens with "\dump_state\n"; server replies the key=value body
//   below terminated by "done". Commands are one per line ('\n'):
//     P az el   -> "RPRT 0"            (set position; GPredict's tracker)
//     p         -> "az\nel\n"          (get position; no RPRT on success)
//     S | \stop -> "RPRT 0"            (stop)
//     K | \park -> "RPRT 0"            (park = stop + hold)
//     R | \reset-> "RPRT 0"            (clear safety latches)
//     \get_info -> name line + RPRT 0
//     1 | caps  -> name + "Max/Min Azimuth/Elevation:" lines + RPRT 0
//                  (GridTracker2 regex-matches "Max Elevation:" for az+el)
//     \get_version \get_capability similar canned
//     status  -> multi-line operator panel + "RPRT 0"  [sat-eqmod extension,
//               no hamlib client sends it; empty handler support -> RPRT -1]
//     q | Q     -> close connection
//   unknown command -> "RPRT -1". A leading '+' (legacy protocol) is
//   stripped. All numbers are doubles.
// Multiple clients may be connected at once (e.g. GridTracker2 following +
// a rotctl monitor); commands from any client act on the shared Tracker.
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace sateq {

// What the server needs from the motion side; Tracker implements it.
// All methods must be safe to call from the server thread.
class RotctldHandler {
public:
    virtual ~RotctldHandler() = default;
    virtual void on_set_target(double az_deg, double el_deg) = 0;
    virtual void on_get_position(double& az_deg, double& el_deg) = 0;
    // Called on the server thread the instant a command LINE was received,
    // BEFORE it is handled (handling may block on the tracker lock while a
    // board stop/restart waits for the deceleration tail). The transport
    // watchdog must be fed by ARRIVAL, not handling completion: a stall of
    // our own would otherwise look like a dead client (live sweeps 2,3,4:
    // 6 s :K-waits false-tripped the watchdog at a 1 Hz-pushing client).
    virtual void on_client_rx() {}
    virtual void on_stop() = 0;
    virtual void on_park() = 0;
    virtual void on_reset() = 0;  // clear safety latch
    virtual std::string on_get_info() { return "sat-eqmod"; }
    // sat-eqmod extension `status`: multi-line human-readable operator
    // panel (each line ends \n; the server appends RPRT). Empty = not
    // supported by this handler (reply RPRT -1, like an unknown command).
    virtual std::string on_status() { return ""; }
};

class RotctldServer {
public:
    RotctldServer(int port, RotctldHandler& handler);
    ~RotctldServer();

    void start();            // spawns accept thread; returns immediately
    void stop();             // joins thread, closes listener
    int bound_port() const { return port_; }

    // Observe connect/disconnect (live client count, called from client
    // threads on every change). Set before start(); last-client-drops
    // (count 0) is the operator-leaving signal for the motion side.
    void set_client_observer(std::function<void(int live_clients)> cb)
    {
        clients_cb_ = std::move(cb);
    }

private:
    void accept_loop();
    void client_session(int fd);  // runs on per-client thread
    void serve_client(int fd);    // read/dispatch loop, returns on EOF/close
    void reap_finished();         // join finished client threads

    int port_;
    int listen_fd_ = -1;
    RotctldHandler& handler_;
    std::atomic<bool> running_{false};
    std::thread th_;
    std::mutex mu_;               // guards fds_ and threads_
    std::set<int> fds_;           // live client fds (for stop() wake-up)
    std::vector<std::future<void>> clients_;
    std::function<void(int)> clients_cb_;  // set before start()
};

// Command processing factored out for unit testing (no sockets needed).
// Returns the exact bytes to send for one input line ("RPRT 0\n" etc.);
// empty return means close the connection.
std::string rotctld_handle_line(const std::string& line, RotctldHandler& handler);

} // namespace sateq
