// MVS (multi-vehicle sandbox) (LOCAL mode)
//
// Simplified migration of `examples_old/mvs.cpp` to the current Agent/Simulator APIs.
//
// Controls:
// - Keyboard:
//   - `0`..`9` select robot index
//   - `w`/`s` throttle +/- (selected robot)
//   - `a`/`d` steering  +/- (selected robot)
//   - `x` reset steering+throttle to 0
// - Optional joystick:
//   - Reads `/dev/input/js0`
//   - Axis 0 = steering, Axis 1 = throttle
//
// Run:
//   ./build/linux/x86_64/release/mvs_local [--joystick]

#include "flatsim/agent.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include "rerun/recording_stream.hpp"
#include "flatsim/utils.hpp"

#include <algorithm>
#include "flatsim/utils.hpp"
#include <chrono>
#include "flatsim/utils.hpp"
#include <cmath>
#include "flatsim/utils.hpp"
#include <cstring>
#include "flatsim/utils.hpp"
#include <fcntl.h>
#include "flatsim/utils.hpp"
#include <iostream>
#include "flatsim/utils.hpp"
#include <linux/joystick.h>
#include "flatsim/utils.hpp"
#include <termios.h>
#include "flatsim/utils.hpp"
#include <thread>
#include "flatsim/utils.hpp"
#include <unistd.h>
#include "flatsim/utils.hpp"
#include <vector>
#include "flatsim/utils.hpp"

static void set_stdin_raw(bool enable, termios &old_termios) {
    if (enable) {
        tcgetattr(STDIN_FILENO, &old_termios);
        termios new_termios = old_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
    }
}

static float clamp01(float v) { return std::clamp(v, -1.0f, 1.0f); }

int main(int argc, char **argv) {
    bool use_joystick = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--joystick") == 0) {
            use_joystick = true;
        }
    }

    // Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("mvs_local", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(3000.0f, 3000.0f, datum, rec);

    // Spawn a small fleet of machines.
    std::vector<agent::Agent *> robots;
    robots.reserve(6);

    robots.push_back(&sim.spawn_agent("examples/machines/tractor.json", utils::make_pose_2d(0.0, 0.0, 0.0),
                                      std::string("tractor_0"), pigment::RGB(0, 255, 100)));
    robots.push_back(&sim.spawn_agent("examples/machines/trailer.json", utils::make_pose_2d(0.0, -5.0, 0.0),
                                      std::string("trailer_0"), pigment::RGB(255, 150, 0)));
    robots.push_back(&sim.spawn_agent("examples/machines/oxbo_harvester.json", utils::make_pose_2d(10.0, 10.0, 0.0),
                                      std::string("oxbo_0"), pigment::RGB(255, 200, 0)));
    robots.push_back(&sim.spawn_agent("examples/machines/trailer.json", utils::make_pose_2d(0.0, -10.0, 0.0),
                                      std::string("trailer_1"), pigment::RGB(255, 100, 50)));
    robots.push_back(&sim.spawn_agent("examples/machines/truck.json", utils::make_pose_2d(20.0, 0.0, 0.0), std::string("truck_0"),
                                      pigment::RGB(100, 100, 255)));
    robots.push_back(&sim.spawn_agent("examples/machines/husky.json", utils::make_pose_2d(30.0, 10.0, 0.0), std::string("husky_0"),
                                      pigment::RGB(128, 0, 255)));

    std::cout << "=== MVS (LOCAL) ===\n";
    std::cout << "Spawned " << robots.size() << " robots\n";
    std::cout << "Keyboard: 0-9 select, w/s throttle, a/d steering, x reset, Ctrl+C quit\n";
    if (use_joystick) {
        std::cout << "Joystick enabled: /dev/input/js0 (axis 0 steer, axis 1 throttle)\n";
    }
    std::cout << std::endl;

    // Optional joystick init
    int js_fd = -1;
    unsigned char num_axes = 0, num_buttons = 0;
    if (use_joystick) {
        js_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        if (js_fd < 0) {
            std::cerr << "Warning: cannot open /dev/input/js0, continuing with keyboard only\n";
        } else {
            ioctl(js_fd, JSIOCGAXES, &num_axes);
            ioctl(js_fd, JSIOCGBUTTONS, &num_buttons);
        }
    }

    termios old_termios{};
    set_stdin_raw(true, old_termios);

    int selected = 0;
    float steering = 0.0f;
    float throttle = 0.0f;

    const float dt = 0.016f;
    int step_count = 0;

    while (true) {
        // Keyboard input
        char ch = 0;
        while (read(STDIN_FILENO, &ch, 1) == 1) {
            if (ch >= '0' && ch <= '9') {
                const int idx = ch - '0';
                if (idx >= 0 && idx < static_cast<int>(robots.size())) {
                    selected = idx;
                    std::cout << "[Select] robot " << selected << " (" << robots[selected]->name() << ")\n";
                }
            } else if (ch == 'w') {
                throttle = clamp01(throttle + 0.05f);
            } else if (ch == 's') {
                throttle = clamp01(throttle - 0.05f);
            } else if (ch == 'a') {
                steering = clamp01(steering - 0.05f);
            } else if (ch == 'd') {
                steering = clamp01(steering + 0.05f);
            } else if (ch == 'x') {
                steering = 0.0f;
                throttle = 0.0f;
            }
        }

        // Joystick input (overrides steering/throttle continuously)
        if (js_fd >= 0) {
            js_event e;
            while (read(js_fd, &e, sizeof(e)) == sizeof(e)) {
                const auto type = e.type & ~JS_EVENT_INIT;
                if (type == JS_EVENT_AXIS && e.number < num_axes) {
                    const int axis = int(e.number);
                    const float value = e.value / 32767.0f;
                    if (axis == 0) {
                        steering = value;
                    } else if (axis == 1) {
                        throttle = -value;
                        throttle = (std::fabs(throttle) < 0.05f) ? 0.0f : throttle;
                    }
                }
            }
        }

        // Apply controls: selected robot moves, others stop.
        for (int i = 0; i < static_cast<int>(robots.size()); ++i) {
            if (i == selected) {
                robots[i]->set_linear(throttle);
                robots[i]->set_angular(steering);
            } else {
                robots[i]->set_linear(0.0f);
                robots[i]->set_angular(0.0f);
            }
        }

        sim.tick(dt);
        if (step_count % 2 == 0) {
            sim.tock();
        }

        if (step_count % 120 == 0) {
            const auto pos = robots[selected]->get_position();
            std::cout << "[Status] sel=" << selected << " steer=" << steering << " thr=" << throttle << " pos=("
                      << pos.point.x << "," << pos.point.y << ")\n";
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Unreachable, but keep cleanup correct.
    set_stdin_raw(false, old_termios);
    if (js_fd >= 0) {
        close(js_fd);
    }
    return 0;
}

