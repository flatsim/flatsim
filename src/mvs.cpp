#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "multiverse/simulator.hpp"
#include "rerun/recording_stream.hpp"

int main() {
    // 1) Open joystick device
    const char *js_device = "/dev/input/js0";
    int js_fd = open(js_device, O_RDONLY | O_NONBLOCK);
    if (js_fd < 0) {
        std::perror("Opening joystick failed");
        return 1;
    }

    // 2) Query number of axes/buttons
    unsigned char num_axes = 0, num_buttons = 0;
    ioctl(js_fd, JSIOCGAXES, &num_axes);
    ioctl(js_fd, JSIOCGBUTTONS, &num_buttons);

    // 3) Prepare state storage
    std::vector<int> axis_states(num_axes, 0);
    std::vector<char> button_states(num_buttons, 0);

    // 4) (Optional) Print joystick name
    char js_name[128] = "Unknown";
    if (ioctl(js_fd, JSIOCGNAME(sizeof(js_name)), js_name) >= 0) {
        std::cout << "Joystick: " << js_name << "  Axes: " << int(num_axes) << "  Buttons: " << int(num_buttons)
                  << std::endl;
    }

    // 5) Connect to Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("multiverse", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // 6) Set up your world and simulator
    concord::Datum world_datum{51.987305, 5.663625, 53.801823};
    concord::Size world_size{50.0f, 50.0f, 100.0f};
    float grid_size = 0.5f;

    auto sim = std::make_shared<mvs::Simulator>(rec);
    sim->init(world_datum, world_size, grid_size);

    for (int i = 0; i < 4; ++i) {
        std::cout << "creating robot " << i << std::endl;
        concord::Pose robot_pose;
        robot_pose.point.enu.x = i * 3;
        robot_pose.point.enu.y = i * 3;
        robot_pose.point.enu.toWGS(world_datum);
        robot_pose.angle.yaw = 0.0f;
        concord::Size chassis_size{0.8f, 1.4f, 0.0f};
        std::vector<concord::Size> wheel_sizes;
        wheel_sizes.push_back({0.1f, 0.2f, 0.0f});
        wheel_sizes.push_back({0.1f, 0.2f, 0.0f});
        wheel_sizes.push_back({0.2f, 0.4f, 0.0f});
        wheel_sizes.push_back({0.2f, 0.4f, 0.0f});
        sim->add_robot(robot_pose, chassis_size, wheel_sizes);
    }

    auto last_time = std::chrono::steady_clock::now();
    std::cout << "Running… (Ctrl-C to quit)\n";

    // 7) Main loop: only joystick + simulation tick
    while (true) {
        // --- read one joystick event if available ---
        js_event e;
        ssize_t bytes = read(js_fd, &e, sizeof(e));
        if (bytes == sizeof(e)) {
            auto type = e.type & ~JS_EVENT_INIT;
            if (type == JS_EVENT_AXIS && e.number < num_axes) {
                float normalized = e.value / 32767.0f;
                axis_states[e.number] = e.value;
                sim->on_joystick_axis(int(e.number), normalized);
            } else if (type == JS_EVENT_BUTTON && e.number < num_buttons) {
                button_states[e.number] = e.value;
                sim->on_joystick_button(int(e.number), e.value != 0);
            }
        }

        // --- advance your simulation by elapsed time ---
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt = now - last_time;
        last_time = now;
        sim->tick(dt.count());

        // --- small sleep to cap CPU usage ---
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(js_fd);
    return 0;
}
