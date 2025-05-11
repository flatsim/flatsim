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

    int selected_robot_idx = 0;

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
    concord::Size world_size{200.0f, 200.0f, 100.0f};
    float grid_size = 2.0f;

    auto sim = std::make_shared<mvs::Simulator>(rec);
    sim->init(world_datum, world_size, grid_size);

    for (int i = 0; i < 4; ++i) {
        std::cout << "creating robot " << i << std::endl;
        concord::Pose robot_pose;
        robot_pose.point.enu.x = i * 3;
        robot_pose.point.enu.y = i * 3;
        robot_pose.point.enu.toWGS(world_datum);
        robot_pose.angle.yaw = 0.0f;

        float width = 0.8f;
        float height = 1.95f;

        concord::Size chassis_size{width, height, 0.0f};

        std::vector<concord::Bound> wheels;
        concord::Size w_size{width * 0.25, height * 0.2f, 0.0f};
        wheels.push_back(concord::Bound(concord::Pose(width / 2, (height / 2) * 0.6, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, (height / 2) * 0.6, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(width / 2, 0.1, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, 0.1, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(width / 2, (-height / 2) * 0.7, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, (-height / 2) * 0.7, 0.0f), w_size));

        std::vector<float> steerings_max = {-0.3f, -0.3f, 0.0f, 0.0f, 1.0f, 1.0f};
        std::vector<float> throttles_max = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f};
        std::pair<std::vector<float>, std::vector<float>> controls{steerings_max, throttles_max};

        std::vector<concord::Bound> karosseries;
        concord::Size k_size{width * 1.26f, height * 0.23f, 0.0f};
        karosseries.push_back(concord::Bound(concord::Pose(0, (height / 2) + k_size.y / 2, 0.0f), k_size));
        k_size = concord::Size(width * 0.9, height * 0.15f, 0.0f);
        pigment::RGB color = pigment::RGB::random();
        karosseries.push_back(concord::Bound(concord::Pose(0, -height / 2 - k_size.y / 2, 0.0f), k_size));

        pigment::RGB robot_color = pigment::RGB::random();
        sim->add_robot(robot_pose, robot_color, chassis_size, wheels, controls, karosseries);
    }

    auto last_time = std::chrono::steady_clock::now();
    std::cout << "Running… (Ctrl-C to quit)\n";

    // 7) Main loop: only joystick + simulation tick
    while (true) {
        // --- read one joystick event if available ---
        for (int i = 0; i < sim->num_robots(); ++i) {
            if (selected_robot_idx == i) {
                continue;
            }
            sim->get_robot(i).set_linear(0.0f);
            sim->get_robot(i).set_angular(0.0f);
        }
        js_event e;
        ssize_t bytes = read(js_fd, &e, sizeof(e));
        if (bytes == sizeof(e)) {
            auto type = e.type & ~JS_EVENT_INIT;
            if (type == JS_EVENT_AXIS && e.number < num_axes) {
                int axis = int(e.number);
                float value = e.value / 32767.0f;
                if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
                    if (axis == 0) {
                        float steering = value * 30.0f;
                        sim->get_robot(selected_robot_idx).set_angular(steering);
                    }

                    if (axis == 1) {
                        float throttle = value * 0.2f;
                        sim->get_robot(selected_robot_idx).set_linear(throttle);
                    }
                }
            } else if (type == JS_EVENT_BUTTON && e.number < num_buttons) {
                int button = int(e.number);
                bool pressed = e.value != 0;
                if (button < 4 && pressed) {
                    selected_robot_idx = button;
                    std::cout << "Selected robot #" << selected_robot_idx << std::endl;
                }
                if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
                    if ((button == 4 || button == 5) && pressed) {
                        sim->get_robot(selected_robot_idx).respawn();
                    }
                    if (button == 9 && pressed) {
                        sim->get_robot(selected_robot_idx).pulsining = true;
                    }
                }
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
