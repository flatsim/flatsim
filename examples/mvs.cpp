#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "flatsim/loader.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/types.hpp"
#include "rerun/recording_stream.hpp"

#include "geotiv/geotiv.hpp"

int main(int argc, char *argv[]) {
    bool joystk = false;

    if (argc > 1) {
        if (std::strcmp(argv[1], "--joystick") == 0) {
            joystk = true;
        }
    }

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
    auto rec = std::make_shared<rerun::RecordingStream>("flatsim", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // 6) Set up your world and simulator
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{300.0f, 300.0f, 300.0f};

    auto sim = std::make_shared<fs::Simulator>(rec);
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
    for (const auto &wgs_coord : coordinates) {
        auto enu_coord = wgs_coord.toENU(world_datum);
        polygon.addPoint(concord::Point{enu_coord.x, enu_coord.y, enu_coord.z});
    }

    fs::LayerInfo layer_info;
    layer_info.name = "grid";
    layer_info.uuid = "grid";
    layer_info.type = "field";
    layer_info.can_accept = {"pea"};
    layer_info.color = pigment::RGB(rand() % 255, rand() % 255, rand() % 255);
    layer_info.bound = polygon.get_obb();
    layer_info.resolution = 0.2f;
    layer_info.field = polygon;
    sim->add_layer(layer_info, true);

    geotiv::RasterCollection rc;
    rc.heading = concord::Euler{0.0, 0.0, 0.0};
    rc.resolution = layer_info.resolution;

    geotiv::Layer layer;
    layer.grid = sim->get_layer(0).get_grid_data();
    layer.samplesPerPixel = 1; // single channel
    layer.planarConfig = 1;    // chunky
    rc.layers.push_back(layer);

    std::filesystem::path outPath = "output.tif";
    geotiv::WriteRasterCollection(rc, outPath);

    // Load machines from JSON files
    std::filesystem::path machines_dir = "examples/machines";

    try {
        // Load tractor from JSON
        auto tractor = fs::Loader::load_from_json(machines_dir / "tractor.json", concord::Pose(10 * 0, 10 * 0, 0.0f),
                                                  "tractor0", pigment::RGB(0, 255, 100));
        sim->add_robot(tractor);
        std::cout << "Loaded tractor from JSON\n";

        // Load trailer from JSON
        auto trailer =
            fs::Loader::load_from_json(machines_dir / "trailer.json", concord::Pose(10 * 0, 10 * 0 - 5, 0.0f),
                                       "trailer1", pigment::RGB(255, 150, 0));
        sim->add_robot(trailer);
        std::cout << "Loaded trailer from JSON\n";

        // Load oxbo harvester from JSON
        auto oxbo = fs::Loader::load_from_json(machines_dir / "oxbo_harvester.json",
                                               concord::Pose(10 * 1, 10 * 1, 0.0f), "oxbo2", pigment::RGB(255, 200, 0));
        sim->add_robot(oxbo);
        std::cout << "Loaded oxbo harvester from JSON\n";

        // Load second trailer from JSON
        auto trailer2 =
            fs::Loader::load_from_json(machines_dir / "trailer.json", concord::Pose(10 * 0, 10 * 0 - 10, 0.0f),
                                       "trailer2", pigment::RGB(255, 100, 50));
        sim->add_robot(trailer2);
        std::cout << "Loaded second trailer from JSON\n";

        // Load truck from JSON
        auto truck = fs::Loader::load_from_json(machines_dir / "truck.json", concord::Pose(10 * 2, 10 * 0, 0.0f),
                                                "big_truck3", pigment::RGB(100, 100, 255));
        sim->add_robot(truck);
        std::cout << "Loaded truck from JSON\n";

    } catch (const std::exception &e) {
        std::cerr << "Error loading machines: " << e.what() << std::endl;
        std::cerr << "Make sure JSON files exist in: " << machines_dir << std::endl;
        return 1;
    }

    // Setup keyboard input (non-blocking)
    struct termios old_termios, new_termios;
    tcgetattr(STDIN_FILENO, &old_termios);
    new_termios = old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    auto last_time = std::chrono::steady_clock::now();
    std::cout << "Running… (Ctrl-C to quit)\n";
    std::cout << "Keyboard: 0-9 to select robots\n";
    std::cout << "Joystick: Button 11 = attach, Button 12 = detach\n";

    // 7) Main loop: keyboard + joystick + simulation tick
    while (true) {
        // --- Handle keyboard input for robot selection ---
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            if (key >= '0' && key <= '9') {
                int robot_num = key - '0';
                if (robot_num < sim->num_robots()) {
                    selected_robot_idx = robot_num;
                    auto &robot = sim->get_robot(selected_robot_idx);
                    std::cout << "Keyboard selected robot #" << selected_robot_idx << " (" << robot.info.name << ")"
                              << std::endl;
                } else {
                    std::cout << "Robot #" << robot_num << " doesn't exist (only " << sim->num_robots() << " robots)\n";
                }
            }
        }

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
                    if (selected_robot_idx >= 0 && selected_robot_idx < sim->num_robots()) {
                        if ((button == 4) && pressed) {
                            sim->get_robot(selected_robot_idx).respawn();
                        }
                        if (button == 9 && pressed) {
                            sim->get_robot(selected_robot_idx).pulse();
                        }
                        if (button == 10 && pressed) {
                            sim->get_robot(selected_robot_idx)
                                .toggle_all_except_section_work("front", 2); // Toggle all except middle section
                        }

                        // Button 11 = Attach trailer
                        if (button == 11 && pressed) {
                            auto &robot = sim->get_robot(selected_robot_idx);
                            if (robot.try_connect_nearby()) {
                                std::cout << "Connected to nearby robot!" << std::endl;
                            } else {
                                std::cout << "No compatible robot nearby to connect" << std::endl;
                            }
                        }

                        // Button 12 = Detach trailer
                        if (button == 12 && pressed) {
                            auto &robot = sim->get_robot(selected_robot_idx);
                            if (robot.is_connected()) {
                                robot.disconnect_trailer();
                                std::cout << "Disconnected trailer!" << std::endl;
                            } else {
                                std::cout << "No trailer connected to disconnect" << std::endl;
                            }
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

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);

    if (joystk) {
        close(js_fd);
    }
    return 0;
}
