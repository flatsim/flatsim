#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <thread>
#include <unistd.h>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/sensor/gps_sensor.hpp"
#include "flatsim/robot/sensor/imu_sensor.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "rerun/recording_stream.hpp"

int main(int argc, char *argv[]) {
    std::cout << "=== Sensor Test with Joystick Control ===" << std::endl;
    std::cout << "Control the tractor with a joystick while sensors output to shared memory" << std::endl;
    std::cout << "Axis 0 = Steering, Axis 1 = Throttle" << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;

    // Initialize joystick
    const char *js_device = "/dev/input/js0";
    int js_fd = open(js_device, O_RDONLY | O_NONBLOCK);
    unsigned char num_axes = 0, num_buttons = 0;

    if (js_fd < 0) {
        std::cerr << "Warning: Failed to open joystick device " << js_device << std::endl;
        std::cerr << "Robot will be stationary. Connect a joystick to control it." << std::endl;
    } else {
        ioctl(js_fd, JSIOCGAXES, &num_axes);
        ioctl(js_fd, JSIOCGBUTTONS, &num_buttons);
        std::cout << "Joystick connected: " << int(num_axes) << " axes, " << int(num_buttons) << " buttons\n"
                  << std::endl;
    }

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("sensor_joystick_test", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    // Create simulator
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 300.0f};
    simulator.init(world_datum, world_size);

    // Load tractor
    try {
        auto tractor_info =
            fs::Loader::load_from_json("examples/machines/tractor.json",
                                       concord::Pose{concord::Point{0.0f, 0.0f}, concord::Euler{0.0f, 0.0f, -1.5708f}});
        simulator.add_robot(tractor_info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load tractor: " << e.what() << std::endl;
        return 1;
    }

    auto &tractor = simulator.get_robot(0);
    tractor.info.uuid = "test_sensors_joy";

    std::cout << "Tractor loaded: " << tractor.info.name << " (UUID: " << tractor.info.uuid << ")" << std::endl;

    // Add GPS sensor
    auto gps = std::make_unique<fs::GPSSensor>(10.0, true, 3.0, 0.02); // 10Hz, RTK enabled
    tractor.sensors.add(std::move(gps));
    std::cout << "\n[GPS] Added sensor (10Hz, RTK enabled)" << std::endl;
    std::cout << "  Output: /dev/shm/flatsim_" << tractor.info.uuid << "_GPS" << std::endl;
    std::cout << "  Format: /tmp/flatsim_" << tractor.info.uuid << "/GPS.format" << std::endl;

    // Add IMU sensor
    auto imu = std::make_unique<fs::IMUSensor>(100.0, 0.01, 0.001, 0.1); // 100Hz, realistic noise
    tractor.sensors.add(std::move(imu));
    std::cout << "\n[IMU] Added sensor (100Hz, 9-DOF)" << std::endl;
    std::cout << "  Output: /dev/shm/flatsim_" << tractor.info.uuid << "_IMU" << std::endl;
    std::cout << "  Format: /tmp/flatsim_" << tractor.info.uuid << "/IMU.format" << std::endl;

    std::cout << "\n=== Simulation Running ===" << std::endl;
    std::cout << "Use joystick to control the tractor" << std::endl;
    std::cout << "Sensor data is being written to shared memory\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f; // 60 FPS

    float steering = 0.0f;
    float throttle = 0.0f;
    int step_count = 0;

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        // Read joystick input
        if (js_fd >= 0) {
            js_event e;
            ssize_t bytes = read(js_fd, &e, sizeof(e));
            if (bytes == sizeof(e)) {
                auto type = e.type & ~JS_EVENT_INIT;

                if (type == JS_EVENT_AXIS && e.number < num_axes) {
                    int axis = int(e.number);
                    float value = e.value / 32767.0f;

                    if (axis == 0) {
                        // Axis 0 = Steering
                        steering = value;
                    } else if (axis == 1) {
                        // Axis 1 = Throttle (inverted)
                        throttle = -value;
                        // Apply deadzone
                        throttle = (std::fabs(throttle) < 0.05f) ? 0.0f : throttle;
                    }
                } else if (type == JS_EVENT_BUTTON && e.number < num_buttons) {
                    int button = int(e.number);
                    bool pressed = e.value != 0;

                    if (pressed && button == 0) {
                        // Button 0 = stop
                        steering = 0.0f;
                        throttle = 0.0f;
                        std::cout << "Button 0: Stop" << std::endl;
                    }
                }
            }
        }

        // Apply control to tractor
        tractor.controls.set_linear(throttle);
        tractor.controls.set_angular(steering);

        // Update simulator
        simulator.tick(dt);
        simulator.tock(5);

        // Print status every 2 seconds
        if (step_count % 120 == 0) {
            auto pos = tractor.get_position();
            double linear_vel, angular_vel;
            tractor.get_velocity(linear_vel, angular_vel);

            std::cout << "\n=== Time " << elapsed << "s ===" << std::endl;
            std::cout << "Control: steering=" << steering << ", throttle=" << throttle << std::endl;
            std::cout << "Position: (" << pos.point.x << ", " << pos.point.y << ")" << std::endl;
            std::cout << "Velocity: linear=" << linear_vel << " m/s, angular=" << angular_vel << " rad/s" << std::endl;

            // GPS data
            auto *gps_sensor = tractor.sensors.get<fs::GPSSensor>();
            if (gps_sensor) {
                auto gps_data = gps_sensor->get_gps_data();
                std::cout << "GPS: lat=" << gps_data.latitude << ", lon=" << gps_data.longitude
                          << ", RTK=" << static_cast<int>(gps_data.rtk_status) << ", Sats=" << gps_data.num_satellites
                          << std::endl;
            }

            // IMU data
            auto *imu_sensor = tractor.sensors.get<fs::IMUSensor>();
            if (imu_sensor) {
                auto imu_data = imu_sensor->get_imu_data();
                std::cout << "IMU: accel=(" << imu_data.accel_x << "," << imu_data.accel_y << "," << imu_data.accel_z
                          << ") m/s²"
                          << ", gyro_z=" << imu_data.gyro_z << " rad/s"
                          << ", yaw=" << imu_data.yaw << " rad" << std::endl;
            }
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    // Cleanup
    if (js_fd >= 0) {
        close(js_fd);
    }

    return 0;
}
