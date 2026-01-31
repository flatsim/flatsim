// Greenhouse fleet navigation demo (LOCAL mode - single process)
//
// Migrated from `examples_old/test_greenhouse_fleet.cpp` to the current Agent/Simulator APIs.
//
// Run:
//   ./build/linux/x86_64/release/test_greenhouse_fleet_local

#include <algorithm>
#include "flatsim/utils.hpp"
#include <chrono>
#include "flatsim/utils.hpp"
#include <cmath>
#include "flatsim/utils.hpp"
#include <fstream>
#include "flatsim/utils.hpp"
#include <iostream>
#include "flatsim/utils.hpp"
#include <map>
#include "flatsim/utils.hpp"
#include <mutex>
#include "flatsim/utils.hpp"
#include <queue>
#include "flatsim/utils.hpp"
#include <random>
#include "flatsim/utils.hpp"
#include <set>
#include "flatsim/utils.hpp"
#include <sstream>
#include "flatsim/utils.hpp"
#include <string>
#include "flatsim/utils.hpp"
#include <thread>
#include "flatsim/utils.hpp"
#include <vector>
#include "flatsim/utils.hpp"

#include "flatsim/agent.hpp"
#include "flatsim/utils.hpp"
#include "flatsim/simulator.hpp"
#include "flatsim/utils.hpp"
#include "pigment/pigment.hpp"
#include "flatsim/utils.hpp"
#include <rerun.hpp>
#include "flatsim/utils.hpp"

// =============================================================================
// Graph structures (loaded from YAML)
// =============================================================================

struct GraphNode {
    int id = 0;
    std::string name;
    float x = 0.0f;
    float y = 0.0f;
};

struct GraphEdge {
    int id = 0;
    int v1 = 0;
    int v2 = 0;
    bool bidirectional = false;
    std::string type;
};

struct NavGraph {
    std::string name;
    float width = 0;
    float length = 0;
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;
    std::map<int, std::vector<int>> adjacency;

    float offset_x() const { return -width / 2.0f; }
    float offset_y() const { return -length / 2.0f; }

    datapod::Point to_world(float x, float y) const { return {x + offset_x(), y + offset_y()}; }

    void build_adjacency() {
        adjacency.clear();
        for (const auto &edge : edges) {
            adjacency[edge.v1].push_back(edge.v2);
            if (edge.bidirectional) {
                adjacency[edge.v2].push_back(edge.v1);
            }
        }
    }

    const GraphNode *get_node(int id) const {
        for (const auto &node : nodes) {
            if (node.id == id) return &node;
        }
        return nullptr;
    }

    const GraphEdge *get_edge(int v1, int v2) const {
        for (const auto &edge : edges) {
            if ((edge.v1 == v1 && edge.v2 == v2) || (edge.bidirectional && edge.v1 == v2 && edge.v2 == v1)) {
                return &edge;
            }
        }
        return nullptr;
    }
};

static NavGraph load_nav_graph(const std::string &filename) {
    NavGraph graph;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open: " + filename);
    }

    std::string line;
    std::string current_section;
    GraphNode current_node;
    GraphEdge current_edge;
    bool in_node = false;
    bool in_edge = false;

    while (std::getline(file, line)) {
        const size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        line = line.substr(start);
        if (line.empty() || line[0] == '#') continue;

        if (line.find("nodes:") == 0) {
            current_section = "nodes";
            continue;
        }
        if (line.find("edges:") == 0) {
            current_section = "edges";
            continue;
        }
        if (line.find("name:") == 0 && current_section.empty()) {
            graph.name = line.substr(6);
            continue;
        }
        if (line.find("dimensions:") == 0) {
            current_section = "dimensions";
            continue;
        }
        if (current_section == "dimensions") {
            if (line.find("width:") == 0) {
                graph.width = std::stof(line.substr(7));
                continue;
            }
            if (line.find("length:") == 0) {
                graph.length = std::stof(line.substr(8));
                current_section = "";
                continue;
            }
        }

        if (current_section == "nodes") {
            if (line[0] == '-') {
                if (in_node) {
                    graph.nodes.push_back(current_node);
                }
                in_node = true;
                current_node = GraphNode{};
                const size_t id_pos = line.find("id:");
                if (id_pos != std::string::npos) {
                    current_node.id = std::stoi(line.substr(id_pos + 4));
                }
            } else if (in_node) {
                if (line.find("id:") == 0) {
                    current_node.id = std::stoi(line.substr(4));
                } else if (line.find("name:") == 0) {
                    current_node.name = line.substr(6);
                } else if (line.find("x:") == 0) {
                    current_node.x = std::stof(line.substr(3));
                } else if (line.find("y:") == 0) {
                    current_node.y = std::stof(line.substr(3));
                }
            }
        }

        if (current_section == "edges") {
            if (line[0] == '-') {
                if (in_edge) {
                    graph.edges.push_back(current_edge);
                }
                if (in_node) {
                    graph.nodes.push_back(current_node);
                    in_node = false;
                }
                in_edge = true;
                current_edge = GraphEdge{};
                const size_t id_pos = line.find("id:");
                if (id_pos != std::string::npos) {
                    current_edge.id = std::stoi(line.substr(id_pos + 4));
                }
            } else if (in_edge) {
                if (line.find("id:") == 0) {
                    current_edge.id = std::stoi(line.substr(4));
                } else if (line.find("v1:") == 0) {
                    current_edge.v1 = std::stoi(line.substr(4));
                } else if (line.find("v2:") == 0) {
                    current_edge.v2 = std::stoi(line.substr(4));
                } else if (line.find("bidirectional:") == 0) {
                    current_edge.bidirectional = (line.find("true") != std::string::npos);
                } else if (line.find("type:") == 0) {
                    current_edge.type = line.substr(6);
                }
            }
        }
    }

    if (in_node) graph.nodes.push_back(current_node);
    if (in_edge) graph.edges.push_back(current_edge);

    graph.build_adjacency();
    return graph;
}

// =============================================================================
// Resource Manager - handles locking of edges/nodes
// =============================================================================

class ResourceManager {
  public:
    bool try_claim_edge(int edge_id, int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (edge_claims_.count(edge_id) && edge_claims_[edge_id] != robot_id) {
            return false;
        }
        edge_claims_[edge_id] = robot_id;
        return true;
    }

    void release_edge(int edge_id, int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (edge_claims_.count(edge_id) && edge_claims_[edge_id] == robot_id) {
            edge_claims_.erase(edge_id);
        }
    }

    void release_all(int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = edge_claims_.begin(); it != edge_claims_.end();) {
            if (it->second == robot_id) {
                it = edge_claims_.erase(it);
            } else {
                ++it;
            }
        }
    }

    bool is_edge_available(int edge_id, int robot_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!edge_claims_.count(edge_id)) return true;
        return edge_claims_.at(edge_id) == robot_id;
    }

    int get_edge_owner(int edge_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!edge_claims_.count(edge_id)) return -1;
        return edge_claims_.at(edge_id);
    }

    std::map<int, int> get_edge_claims() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return edge_claims_;
    }

  private:
    mutable std::mutex mutex_;
    std::map<int, int> edge_claims_;
};

// =============================================================================
// Path Planner - A* with resource awareness
// =============================================================================

class PathPlanner {
  public:
    PathPlanner(const NavGraph &graph, ResourceManager &resource_mgr) : graph_(graph), resource_mgr_(resource_mgr) {}

    std::vector<int> find_path(int start_id, int goal_id, int robot_id) {
        if (start_id == goal_id) return {start_id};

        std::map<int, float> g_score;
        std::map<int, float> f_score;
        std::map<int, int> came_from;
        std::set<int> closed_set;

        auto cmp = [&f_score](int a, int b) { return f_score[a] > f_score[b]; };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open_queue(cmp);

        g_score[start_id] = 0;
        f_score[start_id] = heuristic(start_id, goal_id);
        open_queue.push(start_id);

        while (!open_queue.empty()) {
            int current = open_queue.top();
            open_queue.pop();

            if (current == goal_id) {
                return reconstruct_path(came_from, current);
            }
            if (closed_set.count(current)) continue;
            closed_set.insert(current);

            if (!graph_.adjacency.count(current)) continue;
            for (int neighbor : graph_.adjacency.at(current)) {
                if (closed_set.count(neighbor)) continue;

                const auto *edge = graph_.get_edge(current, neighbor);
                if (edge && !resource_mgr_.is_edge_available(edge->id, robot_id)) {
                    float tentative_g = g_score[current] + distance(current, neighbor) + 1000.0f;
                    if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                        came_from[neighbor] = current;
                        g_score[neighbor] = tentative_g;
                        f_score[neighbor] = tentative_g + heuristic(neighbor, goal_id);
                        open_queue.push(neighbor);
                    }
                    continue;
                }

                float tentative_g = g_score[current] + distance(current, neighbor);
                if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g;
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal_id);
                    open_queue.push(neighbor);
                }
            }
        }
        return {};
    }

  private:
    float heuristic(int a, int b) const {
        const auto *node_a = graph_.get_node(a);
        const auto *node_b = graph_.get_node(b);
        if (!node_a || !node_b) return 0;
        const float dx = node_a->x - node_b->x;
        const float dy = node_a->y - node_b->y;
        return std::sqrt(dx * dx + dy * dy);
    }
    float distance(int a, int b) const { return heuristic(a, b); }

    static std::vector<int> reconstruct_path(const std::map<int, int> &came_from, int current) {
        std::vector<int> path = {current};
        while (came_from.count(current)) {
            current = came_from.at(current);
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    const NavGraph &graph_;
    ResourceManager &resource_mgr_;
};

enum class RobotState { IDLE, PLANNING, MOVING, WAITING, COMPLETED };

struct RobotTask {
    int robot_id = 0;
    std::vector<int> planned_path;
    size_t current_path_index = 0;
    int current_node = -1;
    int target_node = -1;
    RobotState state = RobotState::IDLE;
    int wait_counter = 0;
    std::set<int> claimed_edges;

    bool has_reached_target() const { return current_path_index >= planned_path.size(); }

    int get_next_node() const {
        if (current_path_index + 1 < planned_path.size()) {
            return planned_path[current_path_index + 1];
        }
        return -1;
    }
};

static std::string generate_uuid() {
    static std::mt19937 gen(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<> dis(0, 15);
    std::uniform_int_distribution<> dis2(8, 11);

    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 8; i++) ss << dis(gen);
    ss << "-";
    for (int i = 0; i < 4; i++) ss << dis(gen);
    ss << "-4";
    for (int i = 0; i < 3; i++) ss << dis(gen);
    ss << "-";
    ss << dis2(gen);
    for (int i = 0; i < 3; i++) ss << dis(gen);
    ss << "-";
    for (int i = 0; i < 12; i++) ss << dis(gen);
    return ss.str();
}

static int find_nearest_node(const NavGraph &graph, double x, double y) {
    int best_id = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for (const auto &node : graph.nodes) {
        auto p = graph.to_world(node.x, node.y);
        const double dx = p.x - x;
        const double dy = p.y - y;
        const double d = std::sqrt(dx * dx + dy * dy);
        if (d < best_dist) {
            best_dist = d;
            best_id = node.id;
        }
    }
    return best_id;
}

static void visualize_graph_static(std::shared_ptr<rerun::RecordingStream> rec, const NavGraph &graph) {
    if (!rec) return;

    // Nodes as points.
    std::vector<rerun::Position3D> pts;
    for (const auto &n : graph.nodes) {
        auto p = graph.to_world(n.x, n.y);
        pts.push_back({static_cast<float>(p.x), static_cast<float>(p.y), 0.0f});
    }
    rec->log_static("graph/nodes", rerun::Points3D(pts).with_radii({0.12f}).with_colors({rerun::Color(200, 200, 200)}));

    // Edges as line strips.
    std::vector<rerun::LineStrip3D> lines;
    for (const auto &e : graph.edges) {
        const auto *n1 = graph.get_node(e.v1);
        const auto *n2 = graph.get_node(e.v2);
        if (!n1 || !n2) continue;
        auto p1 = graph.to_world(n1->x, n1->y);
        auto p2 = graph.to_world(n2->x, n2->y);
        lines.push_back(rerun::LineStrip3D(
            {rerun::Vec3D(static_cast<float>(p1.x), static_cast<float>(p1.y), 0.0f),
             rerun::Vec3D(static_cast<float>(p2.x), static_cast<float>(p2.y), 0.0f)}));
    }
    rec->log_static("graph/edges", rerun::LineStrips3D(lines).with_radii({0.03f}).with_colors({rerun::Color(100, 100, 100)}));
}

static void visualize_edge_claims(std::shared_ptr<rerun::RecordingStream> rec, const NavGraph &graph,
                                  const ResourceManager &resource_mgr) {
    if (!rec) return;
    const auto claims = resource_mgr.get_edge_claims();
    std::vector<rerun::LineStrip3D> lines;
    std::vector<rerun::Color> colors;
    for (const auto &e : graph.edges) {
        if (!claims.count(e.id)) continue;
        const auto *n1 = graph.get_node(e.v1);
        const auto *n2 = graph.get_node(e.v2);
        if (!n1 || !n2) continue;
        auto p1 = graph.to_world(n1->x, n1->y);
        auto p2 = graph.to_world(n2->x, n2->y);
        lines.push_back(rerun::LineStrip3D(
            {rerun::Vec3D(static_cast<float>(p1.x), static_cast<float>(p1.y), 0.05f),
             rerun::Vec3D(static_cast<float>(p2.x), static_cast<float>(p2.y), 0.05f)}));
        const int owner = claims.at(e.id);
        const rerun::Color c = (owner % 3 == 0) ? rerun::Color(255, 80, 80) : (owner % 3 == 1) ? rerun::Color(80, 255, 80)
                                                                                                 : rerun::Color(80, 80, 255);
        colors.push_back(c);
    }
    rec->log("graph/claims", rerun::LineStrips3D(lines).with_radii({0.08f}).with_colors(colors));
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    std::cout << "=== Greenhouse Fleet Navigation Demo (LOCAL mode) ===\n";

    // Optional graph file; fall back to a small built-in graph in `examples/greenhouse_blueprint_graph.yaml`.
    std::string graph_path = "examples/greenhouse_blueprint_graph.yaml";
    if (!std::ifstream(graph_path).good()) {
        std::cout << "[Warn] Missing " << graph_path << " (expected in repo). Aborting.\n";
        return 1;
    }

    NavGraph graph;
    try {
        graph = load_nav_graph(graph_path);
    } catch (const std::exception &e) {
        std::cerr << "Failed to load graph: " << e.what() << "\n";
        return 1;
    }
    std::cout << "Loaded graph '" << graph.name << "' nodes=" << graph.nodes.size() << " edges=" << graph.edges.size()
              << "\n";

    auto rec = std::make_shared<rerun::RecordingStream>("greenhouse_fleet", "space");
    (void)rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy");
    rec->log("", rerun::Clear::RECURSIVE);

    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};
    simulator::Simulator sim(200.0f, 200.0f, datum, rec);

    std::vector<pigment::RGB> colors = {{255, 80, 80}, {80, 255, 80}, {80, 80, 255}};

    // Pick start/goal nodes.
    std::vector<int> start_nodes;
    std::vector<int> goal_nodes;
    for (const auto &n : graph.nodes) {
        start_nodes.push_back(n.id);
        goal_nodes.push_back(n.id);
    }
    if (start_nodes.size() < 3 || goal_nodes.size() < 3) {
        std::cerr << "Graph too small for fleet demo\n";
        return 1;
    }

    constexpr int NUM_ROBOTS = 3;
    std::vector<RobotTask> tasks(NUM_ROBOTS);
    std::vector<agent::Agent *> robots;
    robots.reserve(NUM_ROBOTS);

    for (int i = 0; i < NUM_ROBOTS; ++i) {
        const auto *start_node = graph.get_node(start_nodes[i]);
        const auto *goal_node = graph.get_node(goal_nodes[(i + 3) % goal_nodes.size()]);
        if (!start_node || !goal_node) {
            std::cerr << "Invalid start/goal for robot " << i << "\n";
            return 1;
        }

        auto spawn_pos = graph.to_world(start_node->x, start_node->y);
        const std::string uuid = generate_uuid();

        auto &robot = sim.spawn_agent("examples/machines/urdf/husky.urdf", utils::make_pose_2d(spawn_pos.x, spawn_pos.y, 0.0f),
                                      uuid, colors[i]);
        robots.push_back(&robot);
        robot.set_speed(0.3f);
        robot.controls().tracker().set_enabled(true);
        robot.set_navigation_enabled(true);

        tasks[i].robot_id = i;
        tasks[i].current_node = start_node->id;
        tasks[i].target_node = goal_node->id;
        tasks[i].state = RobotState::PLANNING;

        std::cout << "Robot " << i << " spawned at node " << start_node->name << " -> goal " << goal_node->name << "\n";
    }

    ResourceManager resource_mgr;
    PathPlanner planner(graph, resource_mgr);
    visualize_graph_static(rec, graph);

    const float dt = 0.016f;
    int step_count = 0;

    while (true) {
        bool all_completed = true;
        for (const auto &t : tasks) {
            if (t.state != RobotState::COMPLETED) {
                all_completed = false;
                break;
            }
        }
        if (all_completed) {
            std::cout << "All robots completed their tasks\n";
            break;
        }

        for (int i = 0; i < NUM_ROBOTS; i++) {
            auto &task = tasks[i];
            auto &robot = *robots[i];

            switch (task.state) {
            case RobotState::PLANNING: {
                auto path = planner.find_path(task.current_node, task.target_node, i);
                if (path.empty()) {
                    std::cout << "Robot " << i << ": No path found, waiting...\n";
                    task.state = RobotState::WAITING;
                    task.wait_counter = 60;
                } else {
                    task.planned_path = path;
                    task.current_path_index = 0;
                    task.state = RobotState::MOVING;

                    std::vector<datapod::Point> waypoints;
                    for (int node_id : path) {
                        const auto *node = graph.get_node(node_id);
                        if (node) {
                            waypoints.push_back(graph.to_world(node->x, node->y));
                        }
                    }

                    auto params = robot.tracker()->get_controller_params();
                    params.linear_kp = 0.7f;
                    params.angular_kp = 0.8f;
                    params.lookahead_distance = 1.0f;
                    robot.tracker()->set_controller_params(params);
                    robot.controls().tracker().set_controller_type(drivekit::TrackerType::CARROT);
                    robot.controls().tracker().set_enabled(true);
                    robot.set_navigation_enabled(true);

                    robot.tracker()->set_path(drivekit::PathGoal(waypoints, 0.09f, 0.09f, false));
                }
                break;
            }
            case RobotState::MOVING: {
                const auto pos = robot.get_position();
                const int nearest = find_nearest_node(graph, pos.point.x, pos.point.y);

                if (nearest != task.current_node && nearest >= 0) {
                    for (int edge_id : task.claimed_edges) {
                        resource_mgr.release_edge(edge_id, i);
                    }
                    task.claimed_edges.clear();
                    for (size_t idx = task.current_path_index; idx < task.planned_path.size(); idx++) {
                        if (task.planned_path[idx] == nearest) {
                            task.current_path_index = idx;
                            break;
                        }
                    }
                    task.current_node = nearest;
                }

                const int next_node = task.get_next_node();
                if (next_node >= 0) {
                    const auto *edge = graph.get_edge(task.current_node, next_node);
                    if (edge) {
                        if (!resource_mgr.try_claim_edge(edge->id, i)) {
                            const int owner = resource_mgr.get_edge_owner(edge->id);
                            std::cout << "Robot " << i << ": Edge " << edge->id << " blocked by robot " << owner
                                      << ", waiting...\n";
                            robot.tracker()->clear_path();
                            task.state = RobotState::WAITING;
                            task.wait_counter = 30;
                        } else {
                            task.claimed_edges.insert(edge->id);
                        }
                    }
                }

                if (robot.tracker()->is_path_completed() || task.has_reached_target()) {
                    if (task.current_node == task.target_node) {
                        std::cout << "Robot " << i << ": Reached goal\n";
                        resource_mgr.release_all(i);
                        task.state = RobotState::COMPLETED;
                    } else {
                        task.state = RobotState::PLANNING;
                    }
                }
                break;
            }
            case RobotState::WAITING:
                task.wait_counter--;
                if (task.wait_counter <= 0) {
                    task.state = RobotState::PLANNING;
                }
                break;
            case RobotState::IDLE:
            case RobotState::COMPLETED:
                break;
            }
        }

        sim.tick(dt);
        sim.tock();

        if (step_count++ % 30 == 0) {
            visualize_edge_claims(rec, graph, resource_mgr);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}

