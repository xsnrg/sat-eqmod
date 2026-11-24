#include "sateq/rotctld.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace sateq {

namespace {

const char* DUMP_STATE_BODY =
    "1\n"
    "1\n"
    "min_az=-180.000000\n"
    "max_az=540.000000\n"
    "min_el=-90.000000\n"
    "max_el=90.000000\n"
    "south_zero=0\n"
    "rot_type=AzEl\n"
    "done\n";

bool parse_two_doubles(const std::string& s, double& a, double& b)
{
    std::istringstream is(s);
    return static_cast<bool>(is >> a >> b);
}

} // namespace

std::string rotctld_handle_line(const std::string& raw, RotctldHandler& handler)
{
    std::string line = raw;
    // strip \r, leading '+' (legacy protocol), surrounding spaces
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
        line.pop_back();
    if (!line.empty() && line[0] == '+')
        line.erase(0, 1);
    size_t b = line.find_first_not_of(" \t");
    if (b == std::string::npos)
        return "RPRT -1\n";
    line.erase(0, b);
    while (!line.empty() && (line.back() == ' ' || line.back() == '\t'))
        line.pop_back();

    // split command token from args
    std::string cmd = line.substr(0, line.find_first_of(" \t"));
    std::string args = cmd.size() < line.size() ? line.substr(cmd.size() + 1) : "";
    // long-name normalization (hamlib sends "\name" or bare "name")
    if (!cmd.empty() && cmd[0] == '\\')
        cmd.erase(0, 1);

    if (cmd == "dump_state")
        return DUMP_STATE_BODY;
    if (cmd == "P" || cmd == "set_pos" || cmd == "set_position")
    {
        double az, el;
        if (!parse_two_doubles(args, az, el))
            return "RPRT -1\n";
        handler.on_set_target(az, el);
        return "RPRT 0\n";
    }
    if (cmd == "p" || cmd == "get_pos")
    {
        double az = 0, el = 0;
        handler.on_get_position(az, el);
        char buf[64];
        snprintf(buf, sizeof(buf), "%.4f\n%.4f\n", az, el);
        return buf;
    }
    if (cmd == "S" || cmd == "stop")
    {
        handler.on_stop();
        return "RPRT 0\n";
    }
    if (cmd == "K" || cmd == "park")
    {
        handler.on_park();
        return "RPRT 0\n";
    }
    if (cmd == "R" || cmd == "reset")
    {
        handler.on_reset();
        return "RPRT 0\n";
    }
    if (cmd == "M" || cmd == "move")
    {
        // Manual movement: safest interpretation for a satellite daemon is a
        // stop; GPredict never sends it. Accept quietly.
        handler.on_stop();
        return "RPRT 0\n";
    }
    if (cmd == "get_info" || cmd == "_")
    {
        return handler.on_get_info() + "\nRPRT 0\n";
    }
    if (cmd == "1")
    {
        // Hamlib "caps" (numeric): name + human-readable limits, mirroring
        // the dump_state numbers above. GridTracker2's rotor client enables
        // elevation following when a line matches /^Max Elevation:\s*(-?\d+)/
        // (rotor.js parse), so this reply is its az+el capability signal.
        // No bare-numeric lines here: GT2 pairs consecutive numeric-only
        // lines as an az/el position.
        return handler.on_get_info() +
               "\nMax Azimuth: 540.00\nMin Azimuth: -180.00\n"
               "Max Elevation: 90.00\nMin Elevation: -90.00\nRPRT 0\n";
    }
    if (cmd == "get_version" || cmd == "get_capability" || cmd == "dump_caps")
    {
        // minimal but structurally valid extended-ish replies
        return handler.on_get_info() + "\n0.1\n0.1\nRPRT 0\n";
    }
    if (cmd == "status")
    {
        // sat-eqmod operator panel (see RotctldHandler::on_status). Read-
        // only like `p`; GT2/GPredict never send it, unknown elsewhere.
        std::string s = handler.on_status();
        if (s.empty())
            return "RPRT -1\n";
        return s + "RPRT 0\n";
    }
    if (cmd == "q" || cmd == "Q")
        return "";  // close
    return "RPRT -1\n";
}

// ---------------- server ----------------

RotctldServer::RotctldServer(int port, RotctldHandler& handler)
    : port_(port), handler_(handler) {}

RotctldServer::~RotctldServer() { stop(); }

void RotctldServer::start()
{
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0)
        throw std::runtime_error("socket: " + std::string(std::strerror(errno)));
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
        throw std::runtime_error("bind port " + std::to_string(port_) + ": " + std::strerror(errno));
    if (::listen(listen_fd_, 4) != 0)
        throw std::runtime_error("listen: " + std::string(std::strerror(errno)));
    if (port_ == 0)
    {
        socklen_t len = sizeof(addr);
        ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
        port_ = ntohs(addr.sin_port);
    }
    running_ = true;
    th_ = std::thread(&RotctldServer::accept_loop, this);
}

void RotctldServer::stop()
{
    running_ = false;
    if (listen_fd_ >= 0)
    {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (th_.joinable())
        th_.join();
    // Wake every live client thread (recv returns 0 on peer shutdown).
    {
        std::lock_guard<std::mutex> lk(mu_);
        for (int fd : fds_)
            ::shutdown(fd, SHUT_RDWR);
    }
    for (auto& f : clients_)
        f.wait();
    clients_.clear();
}

void RotctldServer::reap_finished()
{
    std::lock_guard<std::mutex> lk(mu_);
    auto it = clients_.begin();
    while (it != clients_.end())
    {
        if (it->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            it = clients_.erase(it);
        else
            ++it;
    }
}

void RotctldServer::accept_loop()
{
    while (running_)
    {
        int cfd = ::accept(listen_fd_, nullptr, nullptr);
        if (cfd < 0)
        {
            if (running_)
                continue;
            break;
        }
        reap_finished();
        // One thread per client: multiple trackers/monitors may connect at
        // once; stop() shuts every live fd down and joins via the futures.
        std::lock_guard<std::mutex> lk(mu_);
        clients_.push_back(std::async(std::launch::async,
                                      &RotctldServer::client_session, this, cfd));
    }
}

void RotctldServer::client_session(int fd)
{
    int live = 0;
    {
        std::lock_guard<std::mutex> lk(mu_);
        fds_.insert(fd);
        live = static_cast<int>(fds_.size());
    }
    if (clients_cb_)
        clients_cb_(live);
    serve_client(fd);
    ::close(fd);
    {
        std::lock_guard<std::mutex> lk(mu_);
        fds_.erase(fd);
        live = static_cast<int>(fds_.size());
    }
    if (clients_cb_)
        clients_cb_(live);
}

void RotctldServer::serve_client(int fd)
{
    // Split receive from dispatch: the handler blocks on the tracker lock,
    // which a board stop/restart can hold for seconds while it waits for a
    // deceleration tail. With a single-threaded read/dispatch loop those
    // stalls prevented recv entirely, so later client commands were neither
    // read nor arrival-stamped and the transport watchdog false-tripped at
    // a client that was pushing the whole time (live sweep-5, t=149).
    // The reader thread therefore ONLY recvs: it arrival-stamps every line
    // and queues it; this thread dispatches and answers.
    struct Queue {
        std::mutex m;
        std::condition_variable cv;
        std::deque<std::string> lines;
        bool closed = false;
    };
    auto q = std::make_shared<Queue>();
    std::thread reader([this, fd, q] {
        std::string buf;
        char chunk[512];
        for (;;)
        {
            ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
            if (n <= 0)
                break;
            buf.append(chunk, static_cast<size_t>(n));
            size_t nl;
            while ((nl = buf.find('\n')) != std::string::npos)
            {
                std::string line = buf.substr(0, nl);
                buf.erase(0, nl + 1);
                handler_.on_client_rx(); // bytes are on the wire (arrival)
                std::lock_guard<std::mutex> lk(q->m);
                if (q->lines.size() >= 1024)
                {
                    // 1024 unread lines is ~15 min of abuse behind a 6 s
                    // stall; treat as a runaway client and kill the session
                    ::shutdown(fd, SHUT_RDWR);
                    break;
                }
                q->lines.push_back(std::move(line));
                q->cv.notify_one();
            }
        }
        {
            std::lock_guard<std::mutex> lk(q->m);
            q->closed = true;
        }
        q->cv.notify_all();
    });

    bool quit = false;
    while (!quit)
    {
        std::string line;
        {
            std::unique_lock<std::mutex> lk(q->m);
            q->cv.wait(lk, [&] { return !q->lines.empty() || q->closed; });
            if (q->lines.empty() && q->closed)
                break;
            line = std::move(q->lines.front());
            q->lines.pop_front();
        }
        std::string resp;
        try
        {
            resp = rotctld_handle_line(line, handler_);
        }
        catch (const std::exception&)
        {
            // Board/serial faults (NAK, malformed reply) must never
            // terminate this client thread — or with it, the process.
            // Report a hamlib generic error and keep the connection.
            resp = "RPRT -1\n";
        }
        if (resp.empty())
        {
            quit = true; // Q/q or close
            break;
        }
        size_t off = 0;
        while (off < resp.size())
        {
            ssize_t w = ::send(fd, resp.data() + off, resp.size() - off, MSG_NOSIGNAL);
            if (w <= 0)
            {
                quit = true;
                break;
            }
            off += static_cast<size_t>(w);
        }
    }
    ::shutdown(fd, SHUT_RDWR); // unblock the reader parked in recv()
    reader.join();
}

} // namespace sateq
