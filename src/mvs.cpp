#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "multiverse/simulator.hpp"
#include "rerun/recording_stream.hpp"

static struct termios orig_termios;

// Restore original terminal settings
void reset_terminal_mode() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios); }

// Put terminal into raw, non‐blocking mode
void set_nonblocking_input() {
    struct termios new_termios;
    // get current
    tcgetattr(STDIN_FILENO, &orig_termios);
    new_termios = orig_termios;

    // disable canonical mode, echo, signals
    new_termios.c_lflag &= ~(ICANON | ECHO);
    // minimum of number input read.
    new_termios.c_cc[VMIN] = 0;
    // timeout (in deciseconds) for read
    new_termios.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    // also set stdin non‐blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // ensure we restore on exit
    std::atexit(reset_terminal_mode);
}

// Return −1 if no key was pressed; otherwise the unsigned char
int poll_keypress() {
    unsigned char c;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n == 1)
        return c;
    return -1;
}

int main() {
    set_nonblocking_input();

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
        // 1) check for key
        int key = poll_keypress();
        if (key != -1) {
            if (key == 27) { // ESC
                keep_running = false;
            } else {
                sim->on_key(static_cast<char>(key));
            }
        }

        // 2) tick sim
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
