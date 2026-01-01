// Sensor Test with Joystick Control (LOCAL mode)
//
// Migrated from `examples_old/test_sensors_joystick.cpp` to the current Agent/Simulator APIs.
//
// Notes:
// - Reads Linux joystick events from `/dev/input/js0` (if present).
// - Writes GPS/IMU output to shared memory via SensorManager auto-SHM.
//
// Run:
//   ./build/linux/x86_64/release/test_sensors_joystick_local

#include "flatsim/agent.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/agent/sensor/gps_sensor.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/agent/sensor/imu_sensor.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include "rerun/recording_stream.hpp"
#include "flatsim/utils.hpp"

#include <chrono>
#include "flatsim/utils.hpp"
#include <cmath>
#include "flatsim/utils.hpp"
#include <fcntl.h>
#include "flatsim/utils.hpp"
#include <iostream>
#include "flatsim/utils.hpp"
#include <linux/joystick.h>
#include "flatsim/utils.hpp"
#include <numbers>
#include "flatsim/utils.hpp"
#include <thread>
#include "flatsim/utils.hpp"
#include <unistd.h>
#include "flatsim/utils.hpp"

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    std::cout << "=== Sensor Test with Joystick Control (LOCAL mode) ===" << std::endl;
    std::cout << "Axis 0 = Steering, Axis 1 = Throttle" << std::endl;
    std::cout << "Button 1 toggles PHTG" << std::endl;
    std::cout << "Press Ctrl+C to stop\n" << std::endl;

    // Initialize joystick (optional).
    const char *js_device = "/dev/input/js0";
    int js_fd = open(js_device, O_RDONLY | O_NONBLOCK);
    unsigned char num_axes = 0, num_buttons = 0;
    if (js_fd < 0) {
        std::cerr << "Warning: failed to open joystick device " << js_device << "\n"
                  << "Robot will be stationary unless you provide other control input.\n"
                  << std::endl;
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

    // Create simulator in LOCAL mode (single process)
    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(500.0f, 500.0f, datum, rec);

    // Spawn tractor with a stable UUID for SHM paths
    constexpr float kSpawnYaw = -static_cast<float>(std::numbers::pi / 2.0);
    auto &tractor = sim.spawn_agent("examples/machines/tractor.json", utils::make_pose_2d(0.0, 0.0, kSpawnYaw),
                                    std::string("test_sensors_joy"));

    std::cout << "Tractor loaded: " << tractor.name() << " (UUID: " << tractor.uuid() << ")" << std::endl;

    // Add GPS sensor (auto-enables SHM because Machine::init sets SensorManager robot UUID).
    tractor.machine().sensors.add(std::make_unique<fs::GPSSensor>(10.0, true, 3.0, 0.02));
    std::cout << "\n[GPS] Added (10Hz, RTK enabled)" << std::endl;
    std::cout << "  Output: /dev/shm/flatsim_" << tractor.uuid() << "_GPS" << std::endl;
    std::cout << "  Format: /tmp/flatsim_" << tractor.uuid() << "/GPS.format" << std::endl;

    // Add IMU sensor
    tractor.machine().sensors.add(std::make_unique<fs::IMUSensor>(100.0, 0.01, 0.001, 0.1));
    std::cout << "\n[IMU] Added (100Hz, 9-DOF)" << std::endl;
    std::cout << "  Output: /dev/shm/flatsim_" << tractor.uuid() << "_IMU" << std::endl;
    std::cout << "  Format: /tmp/flatsim_" << tractor.uuid() << "/IMU.format" << std::endl;

    float steering = 0.0f;
    float throttle = 0.0f;
    bool phtg = false;

    const float dt = 0.016f; // 60 FPS
    int step_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    auto *gps_sensor = tractor.machine().sensors.get<fs::GPSSensor>();
    auto *imu_sensor = tractor.machine().sensors.get<fs::IMUSensor>();

    while (true) {
        // Read joystick input
        if (js_fd >= 0) {
            js_event e;
            const ssize_t bytes = read(js_fd, &e, sizeof(e));
            if (bytes == sizeof(e)) {
                const auto type = e.type & ~JS_EVENT_INIT;

                if (type == JS_EVENT_AXIS && e.number < num_axes) {
                    const int axis = int(e.number);
                    const float value = e.value / 32767.0f;
                    if (axis == 0) {
                        steering = value;
                    } else if (axis == 1) {
                        throttle = -value; // inverted
                        throttle = (std::fabs(throttle) < 0.05f) ? 0.0f : throttle;
                    }
                } else if (type == JS_EVENT_BUTTON && e.number < num_buttons) {
                    const int button = int(e.number);
                    const bool pressed = (e.value != 0);
                    if (pressed && button == 1) {
                        phtg = !phtg;
                        std::cout << "[Joy] Button 1: PHTG=" << (phtg ? "on" : "off") << std::endl;
                    }
                }
            }
        }

        // Apply control (must be set before sim.tick so it is applied the same tick).
        tractor.set_linear(throttle);
        tractor.set_angular(steering);

        // Update sim
        sim.tick(dt);
        sim.tock();

        // Toggle PHTG flag for NMEA output
        if (gps_sensor) {
            gps_sensor->set_phtg_status(phtg);
        }

        // Print status every ~2 seconds
        if (step_count % 120 == 0) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
                                                                                 start_time)
                                     .count();
            const auto pos = tractor.get_position();
            float lin, ang;
            tractor.get_velocity(lin, ang);

            std::cout << "\n=== Time " << elapsed << "s ===" << std::endl;
            std::cout << "Control: steering=" << steering << ", throttle=" << throttle << std::endl;
            std::cout << "Position: (" << pos.point.x << ", " << pos.point.y << ")" << std::endl;
            std::cout << "Velocity: linear=" << lin << " (normalized), angular=" << ang << " (normalized)" << std::endl;

            if (gps_sensor) {
                const auto gps = gps_sensor->get_gps_data();
                std::cout << "GPS: lat=" << gps.latitude << ", lon=" << gps.longitude
                          << ", RTK=" << static_cast<int>(gps.rtk_status) << ", sats=" << gps.num_satellites
                          << std::endl;
            }
            if (imu_sensor) {
                const auto imu = imu_sensor->get_imu_data();
                std::cout << "IMU: accel=(" << imu.accel_x << "," << imu.accel_y << "," << imu.accel_z
                          << ") m/s^2"
                          << ", gyro_z=" << imu.gyro_z << " rad/s"
                          << ", yaw=" << imu.yaw << " rad" << std::endl;
            }
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Unreachable in normal operation
    if (js_fd >= 0) {
        close(js_fd);
    }
    return 0;
}

