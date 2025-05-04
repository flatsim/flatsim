#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "multiverse/simulator.hpp"
#include "rerun/recording_stream.hpp"

static struct termios orig_termios;

// Restore original terminal settings on exit
void reset_terminal_mode() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios); }

// Put terminal into raw, non‐blocking mode
void set_nonblocking_input() {
    struct termios new_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    new_termios = orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    std::atexit(reset_terminal_mode);
}

// Read *all* pending key‐press bytes and return them in a vector
std::vector<unsigned char> poll_keypresses() {
    std::vector<unsigned char> keys;
    unsigned char buf[32];
    ssize_t n;
    while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
        for (ssize_t i = 0; i < n; ++i) {
            keys.push_back(buf[i]);
        }
    }
    return keys;
}

int main() {
    set_nonblocking_input();

    // Connect to Rerun recording stream
    auto rec = std::make_shared<rerun::RecordingStream>("multiverse", "space");
    auto rec_running = rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy");
    if (rec_running.is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // … your world setup here …
    concord::Datum world_datum{51.987305, 5.663625, 53.801823};
    mvs::Size world_size{100.0f, 100.0f, 100.0f};
    mvs::Size grid_size{1.0f, 1.0f, 1.0f};

    auto sim = std::make_shared<mvs::Simulator>(rec);
    sim->init(world_datum, world_size, grid_size);

    auto last_time = std::chrono::steady_clock::now();
    std::cout << "Running… (press ESC to quit)\n";

    bool keep_running = true;
    while (keep_running) {
        // 1) fetch all pending keys
        auto keys = poll_keypresses();
        for (auto k : keys) {
            if (k == 27) { // ESC
                keep_running = false;
            } else {
                sim->on_key(static_cast<char>(k));
            }
        }

        // 2) advance simulation
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> elapsed = now - last_time;
        last_time = now;
        sim->tick(elapsed.count());

        // 3) throttle CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Shutting down.\n";
    return 0;
}
