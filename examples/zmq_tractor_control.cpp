#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <boost/json.hpp>
#include <zmq.hpp>

#include "flatsim/core/loader.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/robot/types.hpp"
#include "rerun/recording_stream.hpp"

namespace json = boost::json;

class TractorZMQController {
  private:
    zmq::context_t context;
    zmq::socket_t socket;
    fs::Simulator &simulator;
    std::map<std::string, int> agent_to_robot_idx;
    std::map<std::string, concord::Point> agent_target_positions; // Track current targets
    std::map<std::string, bool> agent_arrived;                    // Track which agents have arrived
    float grid_cell_size;
    int grid_size;
    bool initialized;

  public:
    TractorZMQController(fs::Simulator &sim, const std::string &ipc_address)
        : context(1), socket(context, zmq::socket_type::rep), simulator(sim), grid_cell_size(10.0f), grid_size(0),
          initialized(false) {
        socket.connect(ipc_address);
        std::cout << "✓ Connected to " << ipc_address << std::endl;
    }

    void run() {
        std::cout << "Waiting for initialization and move commands..." << std::endl;

        while (true) {
            // Receive message
            zmq::message_t request;
            auto recv_result = socket.recv(request, zmq::recv_flags::none);

            // Parse JSON
            std::string msg_str(static_cast<char *>(request.data()), request.size());
            json::value message = json::parse(msg_str);
            json::object const &msg_obj = message.as_object();

            std::string msg_type = json::value_to<std::string>(msg_obj.at("type"));

            if (msg_type == "init") {
                handle_initialization(msg_obj);
            } else if (msg_type == "move_command") {
                handle_move_command(msg_obj);
            } else {
                std::cerr << "Unknown message type: " << msg_type << std::endl;
            }
        }
    }

  private:
    // Convert grid coordinates (0,0 = bottom-left) to world coordinates (0,0 = center)
    concord::Point grid_to_world(float grid_x, float grid_y) const {
        // COORDINATE SYSTEMS:
        // -------------------
        // Grid:  (0,0) = bottom-left corner, (grid_size-1, grid_size-1) = top-right corner
        // World: (0,0) = center of world, ranges from -50 to +50 for 100x100m world
        //
        // Example for 10x10 grid in 100x100m world:
        //   Grid (0,0)   -> World (-45, -45)  [bottom-left]
        //   Grid (5,5)   -> World (  5,   5)  [center]
        //   Grid (9,9)   -> World ( 45,  45)  [top-right]
        //
        // We add grid_cell_size/2 to center each tractor in its grid cell

        float world_size = grid_cell_size * static_cast<float>(grid_size);
        float half_world = world_size / 2.0f;

        float world_x = (grid_x * grid_cell_size) - half_world + (grid_cell_size / 2.0f);
        float world_y = (grid_y * grid_cell_size) - half_world + (grid_cell_size / 2.0f);

        return concord::Point{world_x, world_y};
    }

    void handle_initialization(json::object const &message) {
        grid_size = json::value_to<int>(message.at("grid_size"));
        int num_agents = json::value_to<int>(message.at("num_agents"));

        // Parse agents array
        json::array const &agents_arr = message.at("agents").as_array();
        std::vector<std::string> agent_names;
        for (const auto &agent : agents_arr) {
            agent_names.push_back(json::value_to<std::string>(agent));
        }

        std::cout << "\n=== ZMQ Initialization ===" << std::endl;
        std::cout << "Grid size: " << grid_size << "x" << grid_size << std::endl;
        std::cout << "Number of agents: " << num_agents << std::endl;
        std::cout << "Agent names: ";
        for (const auto &name : agent_names) {
            std::cout << name << " ";
        }
        std::cout << "\n" << std::endl;

        // Calculate grid cell size for wide turning radius
        // Grid is 10x10, but we want wide spacing for tractor turning
        float total_world_size = 100.0f; // World is 100x100m for compact navigation
        grid_cell_size = total_world_size / static_cast<float>(grid_size);

        std::cout << "Grid cell size: " << grid_cell_size << "m per cell" << std::endl;
        std::cout << "Total navigable area: " << total_world_size << "x" << total_world_size << "m" << std::endl;

        // Spawn tractors at initial positions
        for (size_t i = 0; i < agent_names.size(); ++i) {
            spawn_tractor(agent_names[i], i);
        }

        initialized = true;

        // Send acknowledgment
        json::object ack;
        ack["type"] = "init_ack";
        ack["timestamp"] = static_cast<int64_t>(std::time(nullptr));
        send_response(ack);

        std::cout << "✓ Initialization complete" << std::endl;
    }

    void spawn_tractor(const std::string &agent_name, int index) {
        try {
            // Calculate spawn position - place tractors in grid coordinates 1,1, 2,2, etc.
            // This ensures they're within the grid bounds
            float grid_spawn_x = 1.0f + (index * 1.0f);
            float grid_spawn_y = 1.0f + (index * 1.0f);

            auto spawn_point = grid_to_world(grid_spawn_x, grid_spawn_y);
            float spawn_yaw = 0.0f;

            concord::Pose spawn_pose{spawn_point, concord::Euler{0.0f, 0.0f, spawn_yaw}};

            auto tractor_info = fs::Loader::load_from_json("examples/machines/tractor.json", spawn_pose);
            tractor_info.name = agent_name;

            simulator.add_robot(tractor_info);
            int robot_idx = simulator.num_robots() - 1;
            agent_to_robot_idx[agent_name] = robot_idx;

            auto &tractor = simulator.get_robot(robot_idx);

            // Configure tracker for simple point-to-point navigation
            tractor.tracker->set_controller_type(navcon::TrackerType::CARROT);

            auto params = tractor.tracker->get_controller_params();
            params.carrot_distance = 2.0f; // Lookahead distance
            tractor.tracker->set_controller_params(params);

            std::cout << "  ✓ Spawned " << agent_name << " at grid (" << grid_spawn_x << ", " << grid_spawn_y
                      << ") -> world (" << spawn_point.x << ", " << spawn_point.y << ") with robot_idx=" << robot_idx
                      << std::endl;

        } catch (const std::exception &e) {
            std::cerr << "Failed to spawn " << agent_name << ": " << e.what() << std::endl;
            throw;
        }
    }

    void handle_move_command(json::object const &message) {
        if (!initialized) {
            std::cerr << "Error: Received move command before initialization" << std::endl;
            return;
        }

        int episode = json::value_to<int>(message.at("episode"));
        int step = json::value_to<int>(message.at("step"));
        json::object const &target_poses = message.at("target_poses").as_object();

        std::cout << "\n[Episode " << episode << ", Step " << step << "]" << std::endl;
        std::cout << "Moving tractors to target positions..." << std::endl;

        // Convert grid coordinates to world coordinates and set navigation goals
        for (const auto &kv : target_poses) {
            std::string agent_name = std::string(kv.key());
            json::object const &target = kv.value().as_object();

            if (agent_to_robot_idx.find(agent_name) == agent_to_robot_idx.end()) {
                std::cerr << "Unknown agent: " << agent_name << std::endl;
                continue;
            }

            int robot_idx = agent_to_robot_idx[agent_name];
            auto &tractor = simulator.get_robot(robot_idx);

            // Convert grid coordinates to world coordinates
            float grid_x = json::value_to<double>(target.at("x"));
            float grid_y = json::value_to<double>(target.at("y"));

            auto world_point = grid_to_world(grid_x, grid_y);

            // Get current position
            auto current_pos = tractor.get_position();

            std::cout << "  " << agent_name << ": (" << current_pos.point.x << ", " << current_pos.point.y << ") -> ("
                      << world_point.x << ", " << world_point.y << ")" << std::endl;
            std::cout << "    Grid: (" << grid_x << ", " << grid_y << ") -> World: (" << world_point.x << ", "
                      << world_point.y << ")" << std::endl;

            // Use set_goal() for single point navigation (not set_path())
            // This is the proper way to navigate to a single target point
            float position_tolerance = grid_cell_size * 0.3f; // Arrival tolerance
            navcon::NavigationGoal goal(world_point, position_tolerance);
            tractor.tracker->set_goal(goal);

            // Store the target position and reset arrival status
            agent_target_positions[agent_name] = world_point;
            agent_arrived[agent_name] = false;
        }

        // Wait for all tractors to reach their targets
        // NOTE: Each tractor stops IMMEDIATELY when it arrives (handled in wait_for_arrivals)
        wait_for_arrivals();

        // Clear navigation goals to prevent any future movement
        for (const auto &kv : target_poses) {
            std::string agent_name = std::string(kv.key());
            if (agent_to_robot_idx.find(agent_name) != agent_to_robot_idx.end()) {
                int robot_idx = agent_to_robot_idx[agent_name];
                auto &tractor = simulator.get_robot(robot_idx);
                tractor.tracker->clear_goal();
            }
        }

        // Send arrival confirmation
        json::object response;
        response["type"] = "arrived";
        response["timestamp"] = static_cast<int64_t>(std::time(nullptr));

        json::object agents_status;
        for (const auto &kv : target_poses) {
            std::string agent_name = std::string(kv.key());
            agents_status[agent_name] = true;
        }
        response["agents"] = agents_status;

        send_response(response);
        std::cout << "  ✓ All tractors arrived and stopped - sent confirmation" << std::endl;
    }

    void wait_for_arrivals() {
        const float dt = 0.016f;              // 60 FPS
        const int max_steps = 60 * 120;       // 2 minute timeout
        const int progress_interval = 60 * 2; // Print progress every 2 seconds

        int step_count = 0;
        bool all_arrived = false;

        while (!all_arrived && step_count < max_steps) {
            // FIRST: Check each tractor and STOP IT immediately when it arrives
            // This must happen BEFORE tick() so navcon doesn't send new commands
            all_arrived = true;
            for (const auto &[agent_name, robot_idx] : agent_to_robot_idx) {
                auto &tractor = simulator.get_robot(robot_idx);

                if (agent_arrived[agent_name]) {
                    // Already arrived and stopped, keep it stopped
                    tractor.controls.set_angular(0.0f);
                    tractor.controls.set_linear(0.0f);
                } else if (tractor.tracker->is_goal_reached()) {
                    // This tractor has JUST arrived - STOP IT IMMEDIATELY!
                    // CRITICAL: Clear the goal so tracker doesn't send more commands in tick()
                    tractor.tracker->clear_goal();
                    tractor.controls.set_angular(0.0f);
                    tractor.controls.set_linear(0.0f);
                    agent_arrived[agent_name] = true;
                    std::cout << "      ✓ " << agent_name << " ARRIVED and STOPPED" << std::endl;
                } else {
                    // This tractor is still moving
                    all_arrived = false;
                }
            }

            // SECOND: Run simulation step (navcon won't command stopped tractors)
            simulator.tick(dt);
            simulator.tock(5); // Update visualization every 5 ticks

            // Print progress
            if (step_count % progress_interval == 0 && !all_arrived) {
                std::cout << "    Progress at " << (step_count / 60) << "s:" << std::endl;
                for (const auto &[agent_name, robot_idx] : agent_to_robot_idx) {
                    auto &tractor = simulator.get_robot(robot_idx);
                    auto pos = tractor.get_position();

                    // Use stored target position, not get_current_target() which may be invalid after clear_goal()
                    auto target = agent_target_positions[agent_name];

                    float distance =
                        std::sqrt(std::pow(target.x - pos.point.x, 2) + std::pow(target.y - pos.point.y, 2));

                    std::cout << "      " << agent_name << ": distance to target = " << distance << "m";
                    if (agent_arrived[agent_name]) {
                        std::cout << " [ARRIVED]";
                    }
                    std::cout << std::endl;
                }
            }

            step_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
        }

        if (!all_arrived) {
            std::cout << "  ⚠ Warning: Timeout reached, not all tractors arrived" << std::endl;
        }
    }

    void send_response(json::object const &response) {
        std::string response_str = json::serialize(response);
        zmq::message_t reply(response_str.size());
        memcpy(reply.data(), response_str.c_str(), response_str.size());
        auto send_result = socket.send(reply, zmq::send_flags::none);
    }
};

int main(int argc, char *argv[]) {
    std::cout << "=== ZMQ Tractor Control Integration ===" << std::endl;

    std::string ipc_address = "ipc:///tmp/robot_control.ipc";
    if (argc > 1) {
        ipc_address = argv[1];
    }

    std::cout << "IPC Address: " << ipc_address << std::endl;

    // Initialize Rerun logging
    auto rec = std::make_shared<rerun::RecordingStream>("zmq_tractor_control", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // Create simulator with wide grid
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{100.0f, 100.0f, 300.0f}; // 100x100m grid for wide turning
    simulator.init(world_datum, world_size);

    std::cout << "Simulator initialized with 100x100m world" << std::endl;

    try {
        TractorZMQController controller(simulator, ipc_address);
        controller.run();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
