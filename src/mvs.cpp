#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "multiverse/machines.hpp"
#include "multiverse/simulator.hpp"
#include "multiverse/types.hpp"
#include "rerun/recording_stream.hpp"

int main() {
    bool joystk = true;
    int selected_robot_idx = 0;
    int js_fd;
    unsigned char num_axes = 0, num_buttons = 0;
    if (joystk) {
        // 1) Open joystick device
        const char *js_device = "/dev/input/js0";
        js_fd = open(js_device, O_RDONLY | O_NONBLOCK);
        if (js_fd < 0) {
            std::perror("Opening joystick failed");
            return 1;
        }

        // 2) Query number of axes/buttons
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
    }

    // 5) Connect to Rerun
    auto rec = std::make_shared<rerun::RecordingStream>("multiverse", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // 6) Set up your world and simulator
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{300.0f, 300.0f, 300.0f};

    auto sim = std::make_shared<mvs::Simulator>(rec);
    sim->init(world_datum, world_size);

    std::vector<concord::WGS> coordinates;
    coordinates.push_back(concord::WGS(51.98765392402663, 5.660072928621929, 0.0));
    coordinates.push_back(concord::WGS(51.98816428304869, 5.661754957062072, 0.0));
    coordinates.push_back(concord::WGS(51.989850316694316, 5.660416700858434, 0.0));
    coordinates.push_back(concord::WGS(51.990417354104295, 5.662166255987472, 0.0));
    coordinates.push_back(concord::WGS(51.991078888673854, 5.660969191951295, 0.0));
    coordinates.push_back(concord::WGS(51.989479848375254, 5.656874619070777, 0.0));
    coordinates.push_back(concord::WGS(51.988156722216644, 5.657715633290422, 0.0));
    coordinates.push_back(concord::WGS(51.98765392402663, 5.660072928621929, 0.0));

    concord::Polygon polygon;
    polygon.from_wgs(coordinates, world_datum);

    mvs::LayerInfo layer_info;
    layer_info.name = "grid";
    layer_info.uuid = "grid";
    layer_info.type = "field";
    layer_info.can_accept = {"pea"};
    layer_info.color = pigment::RGB(rand() % 255, rand() % 255, rand() % 255);
    layer_info.bound = polygon.get_obb(world_datum);
    layer_info.resolution = 0.2f;
    layer_info.field = polygon;
    sim->add_layer(layer_info, true);

    sim->add_robot(mvs::oxbo_harvester(concord::Pose(10 * 0, 10 * 0, 0.0f), "oxbo" + std::to_string(0),
                                       pigment::RGB(255, 200, 0)));
    sim->add_robot(mvs::oxbo_harvester(concord::Pose(10 * 1, 10 * 1, 0.0f), "oxbo" + std::to_string(1),
                                       pigment::RGB(255, 200, 0)));
    sim->add_robot(mvs::oxbo_harvester(concord::Pose(10 * 2, 10 * 2, 0.0f), "oxbo" + std::to_string(2),
                                       pigment::RGB(255, 200, 0)));

    auto last_time = std::chrono::steady_clock::now();
    std::cout << "Running… (Ctrl-C to quit)\n";

    // 7) Main loop: only joystick + simulation tick
    while (true) {
        // --- read one joystick event if available ---
        for (int i = 0; i < sim->num_robots(); ++i) {
            if (selected_robot_idx != i) {
                sim->set_controls(i, 0.0f, 0.0f);
            }
        }
        if (joystk) {
            js_event e;
            ssize_t bytes = read(js_fd, &e, sizeof(e));
            if (bytes == sizeof(e)) {
                auto type = e.type & ~JS_EVENT_INIT;
                if (type == JS_EVENT_AXIS && e.number < num_axes) {
                    int axis = int(e.number);
                    float value = e.value / 32767.0f;
                    if (selected_robot_idx >= 0 && selected_robot_idx < 4) {
                        if (axis == 0) {
                            float steering = value;
                            sim->get_robot(selected_robot_idx).set_angular(steering);
                        }

                        if (axis == 1) {
                            float throttle = value;
                            throttle = (fabs(throttle) < 0.05f) ? 0.0f : throttle;
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
                        if ((button == 5) && pressed) {
                            sim->get_robot(selected_robot_idx).toggle_work("front");
                        }
                        if ((button == 4) && pressed) {
                            sim->get_robot(selected_robot_idx).respawn();
                        }
                        if (button == 9 && pressed) {
                            sim->get_robot(selected_robot_idx).pulse();
                        }
                        if (button == 10 && pressed) {
                            sim->get_robot(selected_robot_idx).pulse();
                        }
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
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    }

    if (joystk) {
        close(js_fd);
    }
    return 0;
}
