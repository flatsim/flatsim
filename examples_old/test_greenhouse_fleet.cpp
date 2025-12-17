#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

#include "flatsim/core/loader.hpp"
#include "flatsim/robot/types.hpp"
#include "flatsim/simulator.hpp"
#include "pigment/pigment.hpp"
#include "rerun.hpp"

// =============================================================================
// Graph structures (loaded from blueprint_graph.yaml)
// =============================================================================

struct GraphNode {
    int id;
    std::string name;
    float x;
    float y;
};

struct GraphEdge {
    int id;
    int v1;
    int v2;
    bool bidirectional;
    std::string type; // "gutter", "main_path", "cross"
};

struct NavGraph {
    std::string name;
    float width = 0;
    float length = 0;
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;
    std::map<int, std::vector<int>> adjacency; // node_id -> list of neighbor node_ids

    // Get offset to center graph in world (world is centered at origin)
    float offset_x() const { return -width / 2.0f; }
    float offset_y() const { return -length / 2.0f; }

    // Convert graph coordinates to world coordinates
    concord::Point to_world(float x, float y) const { return {x + offset_x(), y + offset_y()}; }

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

    // Find edge between two nodes
    const GraphEdge *get_edge(int v1, int v2) const {
        for (const auto &edge : edges) {
            if ((edge.v1 == v1 && edge.v2 == v2) || (edge.bidirectional && edge.v1 == v2 && edge.v2 == v1)) {
                return &edge;
            }
        }
        return nullptr;
    }
};

// =============================================================================
// Simple YAML parser for blueprint_graph.yaml
// =============================================================================

NavGraph load_nav_graph(const std::string &filename) {
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
        // Trim leading whitespace
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        line = line.substr(start);

        if (line.empty() || line[0] == '#') continue;

        // Check for section headers
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
                current_section = ""; // Done with dimensions
                continue;
            }
        }

        // Parse nodes section
        if (current_section == "nodes") {
            if (line[0] == '-') {
                if (in_node) {
                    graph.nodes.push_back(current_node);
                }
                in_node = true;
                current_node = GraphNode{};
                // Parse "- id: X" format
                size_t id_pos = line.find("id:");
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

        // Parse edges section
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
                size_t id_pos = line.find("id:");
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

    // Don't forget the last item
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
    // Try to claim an edge for a robot. Returns true if successful.
    bool try_claim_edge(int edge_id, int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (edge_claims_.count(edge_id) && edge_claims_[edge_id] != robot_id) {
            return false; // Edge claimed by another robot
        }
        edge_claims_[edge_id] = robot_id;
        return true;
    }

    // Try to claim a node for a robot. Returns true if successful.
    bool try_claim_node(int node_id, int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (node_claims_.count(node_id) && node_claims_[node_id] != robot_id) {
            return false; // Node claimed by another robot
        }
        node_claims_[node_id] = robot_id;
        return true;
    }

    // Release an edge claim
    void release_edge(int edge_id, int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (edge_claims_.count(edge_id) && edge_claims_[edge_id] == robot_id) {
            edge_claims_.erase(edge_id);
        }
    }

    // Release a node claim
    void release_node(int node_id, int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (node_claims_.count(node_id) && node_claims_[node_id] == robot_id) {
            node_claims_.erase(node_id);
        }
    }

    // Release all claims for a robot
    void release_all(int robot_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = edge_claims_.begin(); it != edge_claims_.end();) {
            if (it->second == robot_id) {
                it = edge_claims_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = node_claims_.begin(); it != node_claims_.end();) {
            if (it->second == robot_id) {
                it = node_claims_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Check if edge is available (not claimed or claimed by this robot)
    bool is_edge_available(int edge_id, int robot_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!edge_claims_.count(edge_id)) return true;
        return edge_claims_.at(edge_id) == robot_id;
    }

    // Get who owns an edge (-1 if unclaimed)
    int get_edge_owner(int edge_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!edge_claims_.count(edge_id)) return -1;
        return edge_claims_.at(edge_id);
    }

    // Get all edge claims (for visualization)
    std::map<int, int> get_edge_claims() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return edge_claims_;
    }

  private:
    mutable std::mutex mutex_;
    std::map<int, int> edge_claims_; // edge_id -> robot_id
    std::map<int, int> node_claims_; // node_id -> robot_id
};

// =============================================================================
// Path Planner - A* with resource awareness
// =============================================================================

class PathPlanner {
  public:
    PathPlanner(const NavGraph &graph, ResourceManager &resource_mgr) : graph_(graph), resource_mgr_(resource_mgr) {}

    // Find path from start node to goal node, avoiding claimed resources
    std::vector<int> find_path(int start_id, int goal_id, int robot_id) {
        if (start_id == goal_id) return {start_id};

        // A* implementation
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

            // Explore neighbors
            if (!graph_.adjacency.count(current)) continue;

            for (int neighbor : graph_.adjacency.at(current)) {
                if (closed_set.count(neighbor)) continue;

                // Check if edge is available
                const auto *edge = graph_.get_edge(current, neighbor);
                if (edge && !resource_mgr_.is_edge_available(edge->id, robot_id)) {
                    // Edge is claimed by another robot - add high cost instead of blocking
                    // This allows finding alternative routes
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

        return {}; // No path found
    }

  private:
    float heuristic(int a, int b) const {
        const auto *node_a = graph_.get_node(a);
        const auto *node_b = graph_.get_node(b);
        if (!node_a || !node_b) return 0;
        float dx = node_a->x - node_b->x;
        float dy = node_a->y - node_b->y;
        return std::sqrt(dx * dx + dy * dy);
    }

    float distance(int a, int b) const { return heuristic(a, b); }

    std::vector<int> reconstruct_path(const std::map<int, int> &came_from, int current) {
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

// =============================================================================
// Robot Task - represents a robot's current mission
// =============================================================================

enum class RobotState { IDLE, PLANNING, MOVING, WAITING, COMPLETED };

struct RobotTask {
    int robot_id;
    std::vector<int> planned_path; // Node IDs to visit
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

// =============================================================================
// Utility functions
// =============================================================================

std::string generate_uuid() {
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

// Find nearest node given world coordinates
int find_nearest_node(const NavGraph &graph, float world_x, float world_y) {
    // Convert world coords to graph coords
    float graph_x = world_x - graph.offset_x();
    float graph_y = world_y - graph.offset_y();

    int nearest = -1;
    float min_dist = std::numeric_limits<float>::max();
    for (const auto &node : graph.nodes) {
        float dx = node.x - graph_x;
        float dy = node.y - graph_y;
        float dist = dx * dx + dy * dy;
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node.id;
        }
    }
    return nearest;
}

// =============================================================================
// Visualization functions
// =============================================================================

// Robot colors for visualization (matching test_farmtrax.cpp style)
const std::vector<pigment::RGB> ROBOT_COLORS_RGB = {
    pigment::RGB{90, 196, 185}, // #5AC4B9 - Turquoise
    pigment::RGB{90, 153, 196}, // #5A99C4 - Sky Blue
    pigment::RGB{90, 101, 196}, // #5A65C4 - Periwinkle
};

const std::vector<rerun::Color> ROBOT_COLORS = {
    rerun::Color(90, 196, 185), // Turquoise
    rerun::Color(90, 153, 196), // Sky Blue
    rerun::Color(90, 101, 196), // Periwinkle
};

void visualize_graph_static(std::shared_ptr<rerun::RecordingStream> rec, const NavGraph &graph) {
    // Visualize all nodes as points
    std::vector<rerun::Position3D> node_positions;
    std::vector<rerun::Color> node_colors;

    for (const auto &node : graph.nodes) {
        auto wp = graph.to_world(node.x, node.y);
        node_positions.push_back({static_cast<float>(wp.x), static_cast<float>(wp.y), 0.1f});

        // Color nodes by type (based on name)
        if (node.name.find("_end") != std::string::npos) {
            node_colors.push_back(rerun::Color(255, 200, 100)); // Orange for end nodes
        } else if (node.name.find("MP_") != std::string::npos) {
            node_colors.push_back(rerun::Color(150, 150, 150)); // Gray for main path
        } else {
            node_colors.push_back(rerun::Color(200, 200, 255)); // Light blue for gutter turns
        }
    }

    rec->log_static("graph/nodes", rerun::Points3D(node_positions).with_colors(node_colors).with_radii({0.075f}));

    // Visualize edges as lines (grouped by type)
    std::vector<rerun::LineStrip3D> gutter_lines;
    std::vector<rerun::LineStrip3D> main_path_lines;
    std::vector<rerun::LineStrip3D> cross_lines;

    for (const auto &edge : graph.edges) {
        const auto *n1 = graph.get_node(edge.v1);
        const auto *n2 = graph.get_node(edge.v2);
        if (!n1 || !n2) continue;

        auto wp1 = graph.to_world(n1->x, n1->y);
        auto wp2 = graph.to_world(n2->x, n2->y);
        rerun::LineStrip3D line({{static_cast<float>(wp1.x), static_cast<float>(wp1.y), 0.05f},
                                 {static_cast<float>(wp2.x), static_cast<float>(wp2.y), 0.05f}});

        if (edge.type == "gutter") {
            gutter_lines.push_back(line);
        } else if (edge.type == "main_path") {
            main_path_lines.push_back(line);
        } else {
            cross_lines.push_back(line);
        }
    }

    // Log each edge type with different colors
    if (!gutter_lines.empty()) {
        rec->log_static(
            "graph/edges/gutter",
            rerun::LineStrips3D(gutter_lines).with_colors({rerun::Color(100, 150, 255)}).with_radii({0.02f}));
    }
    if (!main_path_lines.empty()) {
        rec->log_static(
            "graph/edges/main_path",
            rerun::LineStrips3D(main_path_lines).with_colors({rerun::Color(255, 100, 100)}).with_radii({0.02f}));
    }
    if (!cross_lines.empty()) {
        rec->log_static(
            "graph/edges/cross",
            rerun::LineStrips3D(cross_lines).with_colors({rerun::Color(200, 100, 255)}).with_radii({0.0125f}));
    }

    // Visualize greenhouse floor as a box (centered at origin)
    rec->log_static("greenhouse/floor",
                    rerun::Boxes3D::from_centers_and_sizes({{0.0f, 0.0f, -0.1f}}, {{graph.width, graph.length, 0.1f}})
                        .with_colors({rerun::Color(200, 230, 200, 100)}));

    // Main path area
    float main_path_y_min = 15.25f;
    float main_path_y_max = 19.75f;
    float main_path_center_y = (main_path_y_min + main_path_y_max) / 2.0f + graph.offset_y();
    float main_path_width = main_path_y_max - main_path_y_min;
    rec->log_static("greenhouse/main_path",
                    rerun::Boxes3D::from_centers_and_sizes({{0.0f, main_path_center_y, 0.01f}},
                                                           {{graph.width, main_path_width, 0.02f}})
                        .with_colors({rerun::Color(180, 180, 180, 150)}));
}

void visualize_claimed_edges(std::shared_ptr<rerun::RecordingStream> rec, const NavGraph &graph,
                             const ResourceManager &resource_mgr) {
    auto claims = resource_mgr.get_edge_claims();

    // Group claimed edges by robot
    std::map<int, std::vector<rerun::LineStrip3D>> robot_edges;

    for (const auto &[edge_id, robot_id] : claims) {
        // Find the edge
        for (const auto &edge : graph.edges) {
            if (edge.id == edge_id) {
                const auto *n1 = graph.get_node(edge.v1);
                const auto *n2 = graph.get_node(edge.v2);
                if (n1 && n2) {
                    auto wp1 = graph.to_world(n1->x, n1->y);
                    auto wp2 = graph.to_world(n2->x, n2->y);
                    robot_edges[robot_id].push_back(
                        rerun::LineStrip3D({{static_cast<float>(wp1.x), static_cast<float>(wp1.y), 0.2f},
                                            {static_cast<float>(wp2.x), static_cast<float>(wp2.y), 0.2f}}));
                }
                break;
            }
        }
    }

    // Clear previous claims
    rec->log("claims", rerun::Clear::RECURSIVE);

    // Log claimed edges for each robot
    for (const auto &[robot_id, lines] : robot_edges) {
        if (!lines.empty() && robot_id < static_cast<int>(ROBOT_COLORS.size())) {
            rec->log("claims/robot_" + std::to_string(robot_id),
                     rerun::LineStrips3D(lines).with_colors({ROBOT_COLORS[robot_id]}).with_radii({0.0375f}));
        }
    }
}

void visualize_robot_paths(std::shared_ptr<rerun::RecordingStream> rec, const NavGraph &graph,
                           const std::vector<RobotTask> &tasks) {
    for (size_t i = 0; i < tasks.size(); i++) {
        const auto &task = tasks[i];
        if (task.planned_path.empty()) continue;

        std::vector<rerun::Position3D> path_points;
        for (int node_id : task.planned_path) {
            const auto *node = graph.get_node(node_id);
            if (node) {
                auto wp = graph.to_world(node->x, node->y);
                path_points.push_back({static_cast<float>(wp.x), static_cast<float>(wp.y), 0.3f + i * 0.1f});
            }
        }

        if (path_points.size() >= 2) {
            // Create line strip from points
            std::vector<rerun::Vec3D> line_points;
            for (const auto &p : path_points) {
                line_points.push_back({p.x(), p.y(), p.z()});
            }

            rerun::Color color = i < ROBOT_COLORS.size() ? ROBOT_COLORS[i] : rerun::Color(255, 255, 255);
            rec->log("paths/robot_" + std::to_string(i),
                     rerun::LineStrips3D({rerun::LineStrip3D(line_points)}).with_colors({color}).with_radii({0.025f}));

            // Mark goal node
            if (!task.planned_path.empty()) {
                const auto *goal = graph.get_node(task.planned_path.back());
                if (goal) {
                    auto wp = graph.to_world(goal->x, goal->y);
                    rec->log("paths/robot_" + std::to_string(i) + "/goal",
                             rerun::Points3D({{static_cast<float>(wp.x), static_cast<float>(wp.y), 0.5f}})
                                 .with_colors({color})
                                 .with_radii({0.125f}));
                }
            }
        }
    }
}

void visualize_robot_status(std::shared_ptr<rerun::RecordingStream> rec, const NavGraph &graph,
                            const std::vector<RobotTask> &tasks, const std::vector<concord::Pose> &positions) {
    for (size_t i = 0; i < tasks.size(); i++) {
        const auto &task = tasks[i];
        const auto &pos = positions[i];

        std::string status;
        switch (task.state) {
        case RobotState::IDLE:
            status = "IDLE";
            break;
        case RobotState::PLANNING:
            status = "PLANNING";
            break;
        case RobotState::MOVING:
            status = "MOVING";
            break;
        case RobotState::WAITING:
            status = "WAITING";
            break;
        case RobotState::COMPLETED:
            status = "DONE";
            break;
        }

        // Log status as text above robot
        rec->log("status/robot_" + std::to_string(i), rerun::TextLog(status).with_level(rerun::TextLogLevel::Info));
    }
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char *argv[]) {
    std::cout << "=== Greenhouse Fleet Navigation Test ===" << std::endl;

    // Load navigation graph
    NavGraph graph;
    try {
        graph = load_nav_graph("blueprint_graph.yaml");
        std::cout << "Loaded graph: " << graph.nodes.size() << " nodes, " << graph.edges.size() << " edges"
                  << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Failed to load graph: " << e.what() << std::endl;
        std::cerr << "Please run: python3 blueprint_gen.py" << std::endl;
        return 1;
    }

    // Initialize rerun
    auto rec = std::make_shared<rerun::RecordingStream>("greenhouse_fleet", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    // Initialize simulator - world size much larger so border is not visible
    fs::Simulator simulator(rec);
    concord::Datum world_datum{51.98954034749562, 5.6584737410504715, 53.801823};
    concord::Size world_size{500.0f, 500.0f, 50.0f};
    simulator.init(world_datum, world_size);

    std::cout << "Greenhouse: " << graph.width << "m x " << graph.length << "m" << std::endl;
    std::cout << "World size: " << world_size.x << "m x " << world_size.y << "m" << std::endl;

    // Robot colors (matching test_farmtrax.cpp)
    const auto &colors = ROBOT_COLORS_RGB;

    // Find start and goal nodes - spawn at gutter positions 3, 6, 9 on center lane
    std::vector<int> start_nodes;
    std::vector<int> goal_nodes;

    // Find center lane nodes at gutter indices 2, 5, 8 (0-indexed = rows 3, 6, 9)
    int target_gutters[] = {2, 5, 8};

    for (int i = 0; i < 3; i++) {
        std::string target_name = "G0" + std::to_string(target_gutters[i]) + "_C";
        for (const auto &node : graph.nodes) {
            if (node.name == target_name) {
                start_nodes.push_back(node.id);
                break;
            }
        }
    }

    // Find goal nodes - gutter end nodes at different positions
    for (const auto &node : graph.nodes) {
        if (node.name.find("_end") != std::string::npos && goal_nodes.size() < 6) {
            goal_nodes.push_back(node.id);
        }
    }

    // Ensure we have enough nodes
    if (start_nodes.size() < 3) {
        for (size_t i = 0; i < 3 && i < graph.nodes.size(); i++) {
            start_nodes.push_back(graph.nodes[i].id);
        }
    }
    if (goal_nodes.size() < 3) {
        for (size_t i = graph.nodes.size() - 3; i < graph.nodes.size(); i++) {
            goal_nodes.push_back(graph.nodes[i].id);
        }
    }

    // Create robots
    constexpr int NUM_ROBOTS = 3;
    std::vector<RobotTask> tasks(NUM_ROBOTS);

    try {
        for (int i = 0; i < NUM_ROBOTS; ++i) {
            const auto *start_node = graph.get_node(start_nodes[i % start_nodes.size()]);
            if (!start_node) {
                std::cerr << "Invalid start node for robot " << i << std::endl;
                return 1;
            }

            // Convert graph coordinates to world coordinates (centered at origin)
            concord::Point spawn_pos = graph.to_world(start_node->x, start_node->y);

            auto husky_info = fs::Loader::load_from_json(
                "examples/machines/husky.json", concord::Pose{spawn_pos, concord::Euler{0.0f, 0.0f, 0.0f}}, colors[i]);

            husky_info.uuid = generate_uuid();
            husky_info.seqid = "greenhouse_robot_" + std::to_string(i);

            simulator.add_robot(husky_info);

            // Enable turn_first - robot will rotate to face target before moving forward
            auto &robot = simulator.get_robot(i);
            robot.state.turn_first = true;
            robot.set_speed(0.3f); // 30% speed - move slower for better visualization

            // Initialize task
            tasks[i].robot_id = i;
            tasks[i].current_node = start_nodes[i % start_nodes.size()];
            tasks[i].target_node = goal_nodes[(i * 2) % goal_nodes.size()]; // Different goals
            tasks[i].state = RobotState::PLANNING;

            std::cout << "Robot " << i << " spawned at node " << start_node->name << " (" << spawn_pos.x << ", "
                      << spawn_pos.y << ") [turn_first=true]" << std::endl;
            std::cout << "  Goal: node " << tasks[i].target_node << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Failed to create robots: " << e.what() << std::endl;
        return 1;
    }

    // Resource manager and path planner
    ResourceManager resource_mgr;
    PathPlanner planner(graph, resource_mgr);

    // Visualize the static graph structure
    visualize_graph_static(rec, graph);

    std::cout << "\nStarting fleet navigation simulation...\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float dt = 0.016f;
    int step_count = 0;

    while (true) {
        // Check if all robots completed
        bool all_completed = true;
        for (const auto &task : tasks) {
            if (task.state != RobotState::COMPLETED) {
                all_completed = false;
                break;
            }
        }
        if (all_completed) {
            std::cout << "All robots completed their tasks!" << std::endl;
            break;
        }

        // Update each robot's task
        for (int i = 0; i < NUM_ROBOTS; i++) {
            auto &task = tasks[i];
            auto &robot = simulator.get_robot(i);

            switch (task.state) {
            case RobotState::PLANNING: {
                // Plan a path to goal
                auto path = planner.find_path(task.current_node, task.target_node, i);
                if (path.empty()) {
                    std::cout << "Robot " << i << ": No path found, waiting..." << std::endl;
                    task.state = RobotState::WAITING;
                    task.wait_counter = 60; // Wait ~1 second
                } else {
                    task.planned_path = path;
                    task.current_path_index = 0;
                    task.state = RobotState::MOVING;

                    std::cout << "Robot " << i << ": Planned path with " << path.size() << " nodes" << std::endl;

                    // Build waypoints for the robot tracker (convert to world coordinates)
                    std::vector<concord::Point> waypoints;
                    for (int node_id : path) {
                        const auto *node = graph.get_node(node_id);
                        if (node) {
                            waypoints.push_back(graph.to_world(node->x, node->y));
                        }
                    }

                    // Configure tracker - use CARROT controller for align-then-move behavior
                    auto params = robot.tracker->get_controller_params();
                    params.linear_kp = 0.7f;  // Slower speed (was 2.0)
                    params.angular_kp = 0.8f; // Slower turning (was 2.5)
                    params.lookahead_distance = 1.0f;
                    robot.tracker->set_controller_params(params);
                    robot.tracker->set_controller_type(drivekit::TrackerType::CARROT);

                    drivekit::PathGoal path_goal(waypoints, 0.09f, 0.09f, false); // Very small reach threshold
                    robot.tracker->set_path(path_goal);
                }
                break;
            }

            case RobotState::MOVING: {
                // Check progress along path
                auto pos = robot.get_position();
                int nearest = find_nearest_node(graph, pos.point.x, pos.point.y);

                // Update current node if we've moved
                if (nearest != task.current_node) {
                    // Release old edge claims
                    for (int edge_id : task.claimed_edges) {
                        resource_mgr.release_edge(edge_id, i);
                    }
                    task.claimed_edges.clear();

                    // Update position in path
                    for (size_t idx = task.current_path_index; idx < task.planned_path.size(); idx++) {
                        if (task.planned_path[idx] == nearest) {
                            task.current_path_index = idx;
                            break;
                        }
                    }
                    task.current_node = nearest;
                }

                // Try to claim next edge
                int next_node = task.get_next_node();
                if (next_node >= 0) {
                    const auto *edge = graph.get_edge(task.current_node, next_node);
                    if (edge) {
                        if (!resource_mgr.try_claim_edge(edge->id, i)) {
                            // Edge blocked! Wait or replan
                            int owner = resource_mgr.get_edge_owner(edge->id);
                            std::cout << "Robot " << i << ": Edge " << edge->id << " blocked by robot " << owner
                                      << ", waiting..." << std::endl;

                            // Stop the robot
                            robot.tracker->clear_path();
                            task.state = RobotState::WAITING;
                            task.wait_counter = 30; // Wait half a second then replan
                        } else {
                            task.claimed_edges.insert(edge->id);
                        }
                    }
                }

                // Check if path completed
                if (robot.tracker->is_path_completed() || task.has_reached_target()) {
                    if (task.current_node == task.target_node) {
                        std::cout << "Robot " << i << ": Reached goal!" << std::endl;
                        resource_mgr.release_all(i);
                        task.state = RobotState::COMPLETED;
                    } else {
                        // Not at goal yet, replan
                        task.state = RobotState::PLANNING;
                    }
                }
                break;
            }

            case RobotState::WAITING: {
                task.wait_counter--;
                if (task.wait_counter <= 0) {
                    // Try to replan
                    task.state = RobotState::PLANNING;
                }
                break;
            }

            case RobotState::IDLE:
            case RobotState::COMPLETED:
                // Nothing to do
                break;
            }
        }

        // Simulator tick
        simulator.tick(dt);
        simulator.tock(5);

        // Update visualization every 10 frames
        if (step_count % 10 == 0) {
            // Visualize claimed edges
            visualize_claimed_edges(rec, graph, resource_mgr);

            // Visualize planned paths
            visualize_robot_paths(rec, graph, tasks);

            // Collect robot positions for status visualization
            std::vector<concord::Pose> positions;
            for (int i = 0; i < NUM_ROBOTS; i++) {
                positions.push_back(simulator.get_robot(i).get_position());
            }
            visualize_robot_status(rec, graph, tasks, positions);
        }

        // Progress report
        if (step_count % 120 == 0) {
            std::cout << "\n--- Progress at " << step_count / 60 << "s ---" << std::endl;
            for (int i = 0; i < NUM_ROBOTS; i++) {
                auto &task = tasks[i];
                auto &robot = simulator.get_robot(i);
                auto pos = robot.get_position();

                std::string state_str;
                switch (task.state) {
                case RobotState::IDLE:
                    state_str = "IDLE";
                    break;
                case RobotState::PLANNING:
                    state_str = "PLANNING";
                    break;
                case RobotState::MOVING:
                    state_str = "MOVING";
                    break;
                case RobotState::WAITING:
                    state_str = "WAITING";
                    break;
                case RobotState::COMPLETED:
                    state_str = "COMPLETED";
                    break;
                }

                const auto *cur_node = graph.get_node(task.current_node);
                std::string node_name = cur_node ? cur_node->name : "unknown";

                std::cout << "Robot " << i << ": " << state_str << " | Node: " << node_name << " | Pos(" << pos.point.x
                          << ", " << pos.point.y << ")" << std::endl;
            }
        }

        step_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\n=== Greenhouse Fleet Test Complete ===" << std::endl;
    return 0;
}
