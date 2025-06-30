#include "flatsim/simulator.hpp"
#include <chrono>
#include <cmath>
#include <execution>
#ifdef HAS_KOKKOS
#include <Kokkos_Core.hpp>
#endif

namespace fs {
    Simulator::Simulator(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {
#ifdef HAS_KOKKOS
        Kokkos::initialize();
        std::cout << "Execution space: " << typeid(Kokkos::DefaultHostExecutionSpace).name() << std::endl;
        std::cout << "Number of threads: " << Kokkos::DefaultHostExecutionSpace().concurrency() << std::endl;
#else
        unsigned int numThreads = std::thread::hardware_concurrency();
        spdlog::info("Using {} threads", numThreads);
#endif
    }
    Simulator::~Simulator() {
#ifdef HAS_KOKKOS
        Kokkos::finalize();
#endif
    }
#ifdef HAS_KOKKOS
    void Simulator::tick(float dt) {
        if (!world) {
            throw NullPointerException("world");
        }
        world->tick(dt);
        // Process robots sequentially (rerun logging needs sequential access)
        for (auto &robot : robots) {
            if (!robot) continue;
            robot->tick(dt);
        }

        if (!world->layers.empty()) {
            Kokkos::parallel_for("layer_tick",
                                 Kokkos::RangePolicy<Kokkos::DefaultHostExecutionSpace>(0, world->layers.size()),
                                 [&](int i) {
                                     // for (auto &robot : robots) {
                                     //     if (!robot) continue;
                                     //     // TODO: Add complicated functionality here (robot-layer interaction)
                                     // }
                                 });
        }
        Kokkos::fence();
    }
#else
    void Simulator::tick(float dt) {
        ticks++;
        if (!world) throw NullPointerException("world");
        // World tick
        world->tick(dt);
        // Process robots in parallel - pure physics, thread-safe
        std::for_each(std::execution::par, robots.begin(), robots.end(), [dt](auto &robott) {
            if (!robott) return;
            robott->tick(dt);
        });
        // Process layers separately, not nested - avoid thread contention
        std::for_each(std::execution::par, world->layers.begin(), world->layers.end(), [dt](auto &layer) {
            if (!layer) return;
            layer->tick(dt);
        });
    }

    void Simulator::tock(int rate) {
        tocks++;
        if (tocks % rate == 0) {
            world->tock();
            for (auto &robot : robots) robot->tock();
            tocks = 0;
        }
    }

    // Template implementation
    template <typename UserLoop> void Simulator::ticktock(UserLoop user_loop, int viz_fps) {
        std::atomic<bool> running{true};
        const auto viz_interval = std::chrono::milliseconds(1000 / viz_fps);

        // Background visualization thread - reads data only
        std::thread viz_thread([this, &running, viz_interval]() {
            while (running.load()) {
                auto viz_start = std::chrono::steady_clock::now();
                this->tock(1); // Always visualize when called

                // Maintain consistent frame rate
                auto viz_end = std::chrono::steady_clock::now();
                auto elapsed = viz_end - viz_start;
                if (elapsed < viz_interval) {
                    std::this_thread::sleep_for(viz_interval - elapsed);
                }
            }
        });

        // Main physics loop with user customization
        auto last_time = std::chrono::steady_clock::now();

        try {
            while (true) {
                // Calculate delta time
                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<float> dt = now - last_time;
                last_time = now;

                // Physics tick at full speed
                this->tick(dt.count());

                // User-defined logic (input handling, control logic, etc.)
                if (!user_loop(dt.count())) {
                    break; // User loop returns false to exit
                }

                // Small sleep to cap CPU usage
                std::this_thread::sleep_for(std::chrono::nanoseconds(100));
            }
        } catch (...) {
            running.store(false);
            if (viz_thread.joinable()) {
                viz_thread.join();
            }
            throw; // Re-throw the exception
        }

        // Clean shutdown
        running.store(false);
        if (viz_thread.joinable()) {
            viz_thread.join();
        }
    }

#endif

    void Simulator::init(concord::Datum datum, concord::Size world_size) {
        world = std::make_shared<fs::World>(rec);
        world->init(datum, world_size);
        world_datum = datum;
    }

    // ROBOT
    void Simulator::add_robot(RobotInfo robot_info) {
        if (!world) {
            throw NullPointerException("world");
        }

        for (auto &robot : robots) {
            if (robot && robot->info.uuid == robot_info.uuid) {
                spdlog::warn("Robot with uuid {} already exists, skipping", robot_info.uuid);
                return;
            }
        }

        auto physics_world = world->get_world();
        if (!physics_world) {
            throw NullPointerException("physics_world");
        }

        robots.emplace_back([&] {
            auto r = std::make_shared<Robot>(rec, physics_world, robots.size());
            r->init(world_datum, robot_info);
            r->set_simulator(this); // Give robot reference to simulator
            return r;
        }());
    }

    void Simulator::set_controls(uint robot_idx, float steering, float throttle) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->set_angular(steering);
        robots[robot_idx]->set_linear(throttle);
    }

    void Simulator::toggle_section_work(uint robot_idx, const std::string &karosserie_name, int section_id) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->toggle_section_work(karosserie_name, section_id);
    }

    void Simulator::toggle_all_sections_work(uint robot_idx, const std::string &karosserie_name) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->toggle_all_sections_work(karosserie_name);
    }

    void Simulator::toggle_all_except_section_work(uint robot_idx, const std::string &karosserie_name,
                                                   int except_section_id) {
        if (robot_idx >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(robot_idx) +
                                           " >= " + std::to_string(robots.size()));
        }

        if (!robots[robot_idx]) {
            throw NullPointerException("robot at index " + std::to_string(robot_idx));
        }

        robots[robot_idx]->toggle_all_except_section_work(karosserie_name, except_section_id);
    }

    Robot &Simulator::get_robot(uint i) {
        if (i >= robots.size()) {
            throw IndexOutOfRangeException("robot index " + std::to_string(i) + " >= " + std::to_string(robots.size()));
        }

        if (!robots[i]) {
            throw NullPointerException("robot at index " + std::to_string(i));
        }

        return *robots[i];
    }
    Robot &Simulator::get_robot(const std::string &uuid) {
        for (auto &robot : robots) {
            if (!robot) {
                continue; // Skip null robots
            }
            if (robot->info.uuid == uuid) {
                return *robot;
            }
        }
        throw EntityNotFoundException("Robot", uuid);
    };
    int Simulator::num_robots() const { return robots.size(); }

    // Spatial queries
    std::vector<std::pair<std::string, concord::Pose>> Simulator::get_all_robot_poses() const {
        std::vector<std::pair<std::string, concord::Pose>> poses;

        for (const auto &robot : robots) {
            if (robot) { // Check for null robots
                poses.emplace_back(robot->info.uuid, robot->info.bound.pose);
            }
        }

        return poses;
    }

    std::vector<Robot *> Simulator::get_all_robots() const {
        std::vector<Robot *> robot_ptrs;

        for (const auto &robot : robots) {
            if (robot) { // Check for null robots
                robot_ptrs.push_back(robot.get());
            }
        }

        return robot_ptrs;
    }

    Robot *Simulator::get_closest_robot(const Robot &from_robot, float max_distance) const {
        Robot *closest = nullptr;
        float min_distance = max_distance;

        concord::Point from_pos = from_robot.info.bound.pose.point;

        for (const auto &robot : robots) {
            if (!robot || robot->info.uuid == from_robot.info.uuid) {
                continue; // Skip null robots and self
            }

            concord::Point to_pos = robot->info.bound.pose.point;
            float distance = std::sqrt(std::pow(to_pos.x - from_pos.x, 2) + std::pow(to_pos.y - from_pos.y, 2));

            if (distance < min_distance) {
                min_distance = distance;
                closest = robot.get();
            }
        }

        return closest;
    }

    // WORLD
    void Simulator::add_layer(LayerInfo layer_info, bool noise) {
        if (!world) {
            throw NullPointerException("world");
        }

        for (auto &layer : world->layers) {
            if (layer && layer->info.uuid == layer_info.uuid) {
                spdlog::warn("Layer with uuid {} already exists, skipping", layer_info.uuid);
                return;
            }
        }

        auto layer = std::make_shared<Layer>(rec, world_datum);
        layer->init(layer_info);
        world->layers.push_back(layer);
        if (noise) {
            layer->add_noise();
        }
        layer->color_field();
        world->adjust_word();
    }
    concord::Datum Simulator::get_datum() const {
        if (!world_datum.is_set()) {
            throw InitializationException("Datum not set");
        }
        return world_datum;
    }
    Layer &Simulator::get_layer(uint i) {
        if (!world) {
            throw NullPointerException("world");
        }

        if (i >= world->layers.size()) {
            throw IndexOutOfRangeException("layer index " + std::to_string(i) +
                                           " >= " + std::to_string(world->layers.size()));
        }

        if (!world->layers[i]) {
            throw NullPointerException("layer at index " + std::to_string(i));
        }

        return *world->layers[i];
    }
    Layer &Simulator::get_layer(const std::string &uuid) {
        if (!world) {
            throw NullPointerException("world");
        }

        for (auto &layer : world->layers) {
            if (!layer) {
                continue; // Skip null layers
            }
            if (layer->info.uuid == uuid) {
                return *layer;
            }
        }
        throw EntityNotFoundException("Layer", uuid);
    };

    // RERUN MANAGEMENT
    void Simulator::reset_recording() {
        static int session_counter = 0;
        std::string session_id = "flatsim_session_" + std::to_string(++session_counter);

        // Create new recording stream
        rec = std::make_shared<rerun::RecordingStream>("flatsim", session_id);
        rec->spawn().exit_on_failure();
        rec->set_global();

        // Reset time state
        rec->reset_time();

        // Note: World and Robot classes would need to be updated to support
        // recording stream replacement if needed
    }

    void Simulator::clear_all_entities() {
        if (!rec) {
            return;
        }

        // Clear all entities recursively from root
        rec->log("", rerun::Clear::RECURSIVE);

        // Reset timeline
        rec->reset_time();
    }
} // namespace fs
