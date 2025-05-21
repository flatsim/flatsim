#include "concord/types_basic.hpp"
#include "multiverse/types.hpp"
#include "multiverse/utils.hpp"

namespace mvs {
    using namespace utils;

    inline mvs::RobotInfo oxbo_harvester(concord::Pose pose, std::string name,
                                         pigment::RGB color = pigment::RGB(255, 200, 0), std::string uuid = "") {
        // extract dimensions
        const float width = 2.8f;
        const float height = 7.68f;

        mvs::RobotInfo robot_info;
        robot_info.RCI = 3;
        robot_info.name = name;
        robot_info.uuid = uuid.empty() ? name : uuid;
        robot_info.type = "harvester";
        robot_info.works_on = {"pea"};

        // overall bound
        robot_info.bound = concord::Bound(pose, concord::Size(width, height, 0.0f));

        // wheels
        std::vector<concord::Bound> wheels;
        concord::Size w_size{width * 0.25f, height * 0.2f, 0.0f};
        wheels.push_back(concord::Bound(concord::Pose(width / 2, (height / 2) * 0.6f, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, (height / 2) * 0.6f, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(width / 2, 0.1f, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, 0.1f, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(width / 2, -(height / 2) * 0.7f, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, -(height / 2) * 0.7f, 0.0f), w_size));
        robot_info.wheels = wheels;

        // control limits
        std::vector<float> steerings_max = {-deg2rad(14), -deg2rad(14), 0.0f, 0.0f, deg2rad(25), deg2rad(25)};
        std::vector<float> throttles_max = {0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 5.0f};
        std::vector<float> steerings_diff = {-deg2rad(2), deg2rad(2), 0.0f, 0.0f, deg2rad(4), -deg2rad(4)};
        std::vector<bool> left_side = {true, false, true, false, true, false};
        robot_info.controlz = {steerings_max, throttles_max, steerings_diff};

        concord::Size k_size;
        KarosserieInfo kaross;
        kaross.name = "front";
        k_size = concord::Size(width * 1.26f, height * 0.23f, 0.0f);
        kaross.bound = concord::Bound(concord::Pose(0.0f, (height / 2) + k_size.y / 2, 0.0f), k_size);
        kaross.color = color;
        kaross.controllable = true;
        kaross.has_physics = false;
        robot_info.karos.push_back(kaross);

        kaross.name = "back";
        k_size = concord::Size(width * 0.9f, height * 0.15f, 0.0f);
        kaross.bound = concord::Bound(concord::Pose(0.0f, -(height / 2) - k_size.y / 2, 0.0f), k_size);
        kaross.color = color;
        kaross.controllable = false;
        kaross.has_physics = false;
        robot_info.karos.push_back(kaross);

        // color
        pigment::RGB robot_color = color;
        robot_info.color = robot_color;

        return robot_info;
    }

    inline mvs::RobotInfo tractor(concord::Pose pose, std::string name, pigment::RGB color = pigment::RGB(255, 200, 0),
                                  std::string uuid = "") {
        // extract dimensions
        const float width = 1.6f;
        const float height = 2.8f;

        mvs::RobotInfo robot_info;
        robot_info.RCI = 3;
        robot_info.name = name;
        robot_info.uuid = uuid.empty() ? name : uuid;
        robot_info.type = "tractor";
        robot_info.works_on = {"food"};

        // overall bound
        robot_info.bound = concord::Bound(pose, concord::Size(width, height, 0.0f));

        // wheels: front
        std::vector<concord::Bound> wheels;
        concord::Size front_w_size{width * 0.22f, height * 0.30f, 0.0f};
        wheels.push_back(concord::Bound(concord::Pose(width / 2, (height / 2) * 0.6f, 0.0f), front_w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, (height / 2) * 0.6f, 0.0f), front_w_size));

        // wheels: back
        concord::Size back_w_size{width * 0.4f, height * 0.5f, 0.0f};
        wheels.push_back(concord::Bound(concord::Pose(width / 2, -(height / 2) * 0.6f, 0.0f), back_w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, -(height / 2) * 0.6f, 0.0f), back_w_size));

        robot_info.wheels = wheels;

        // control limits
        std::vector<float> steerings_max = {deg2rad(35), deg2rad(35), 0.0f, 0.0f};
        std::vector<float> throttles_max = {0.0f, 0.0f, 3.0f, 3.0f};
        std::vector<float> steerings_diff = {-deg2rad(4), deg2rad(4), 0.0f, 0.0f};
        std::vector<bool> left_side = {true, false, true, false};
        robot_info.controlz = {steerings_max, throttles_max, steerings_diff};

        // karosserie
        concord::Size k_size = concord::Size(width * 0.50f, height * 0.05f, 0.0f);
        KarosserieInfo kaross;
        kaross.name = "front";
        kaross.bound = concord::Bound(concord::Pose(0.0f, (height / 2) + k_size.y / 2, 0.0f), k_size);
        kaross.color = color;
        kaross.controllable = true;
        kaross.has_physics = false;
        robot_info.karos.push_back(kaross);

        // color
        pigment::RGB robot_color = color;
        robot_info.color = robot_color;

        return robot_info;
    }
} // namespace mvs
