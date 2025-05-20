#include "concord/types_basic.hpp"
#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"

namespace mvs {
    using namespace utils;
    inline mvs::RobotInfo oxbo_harvester(concord::Pose pose, std::string name) {
        mvs::RobotInfo robot_info;
        robot_info.RCI = 3;
        robot_info.name = name;
        robot_info.uuid = name;
        robot_info.type = "harvester";
        robot_info.works_on = {"pea"};
        robot_info.bound = concord::Bound(pose, concord::Size(2.8f, 7.68f, 0.0f));
        std::vector<concord::Bound> wheels;
        concord::Size w_size{2.8f * 0.25, 7.68f * 0.2f, 0.0f};
        wheels.push_back(concord::Bound(concord::Pose(2.8f / 2, (7.68f / 2) * 0.6, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-2.8f / 2, (7.68f / 2) * 0.6, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(2.8f / 2, 0.1, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-2.8f / 2, 0.1, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(2.8f / 2, (-7.68f / 2) * 0.7, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-2.8f / 2, (-7.68f / 2) * 0.7, 0.0f), w_size));
        robot_info.wheels = wheels;
        std::vector<float> steerings_max = {-deg2rad(14), -deg2rad(14), 0.0f, 0.0f, deg2rad(25), deg2rad(25)};
        std::vector<float> throttles_max = {0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f};
        std::vector<float> steerings_diff = {-deg2rad(2), deg2rad(2), 0.0f, 0.0f, deg2rad(4), -deg2rad(4)};
        robot_info.controlz = {steerings_max, throttles_max, steerings_diff};
        concord::Size k_size;
        k_size = concord::Size(2.8f * 1.26f, 7.68f * 0.23f, 0.0f);
        robot_info.karosseriez["front"] = concord::Bound(concord::Pose(0, (7.68f / 2) + k_size.y / 2, 0.0f), k_size);
        k_size = concord::Size(2.8f * 0.9, 7.68f * 0.15f, 0.0f);
        robot_info.karosseriez["back"] = concord::Bound(concord::Pose(0, -7.68f / 2 - k_size.y / 2, 0.0f), k_size);
        pigment::RGB robot_color = pigment::RGB(255, 200, 0);
        robot_info.color = robot_color;
        return robot_info;
    }
} // namespace mvs
