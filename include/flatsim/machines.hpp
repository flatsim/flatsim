#pragma once

#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"

namespace fs {

    inline fs::RobotInfo oxbo_harvester(concord::Pose pose, std::string name,
                                        pigment::RGB color = pigment::RGB(255, 200, 0), std::string uuid = "") {
        // extract dimensions
        const float width = 2.8f;
        const float height = 7.68f;

        fs::RobotInfo robot_info;
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
        std::vector<float> steerings_max = {utils::deg2rad(14),  utils::deg2rad(14), 0.0f, 0.0f,
                                            -utils::deg2rad(25), -utils::deg2rad(25)};
        std::vector<float> throttles_max = {0.2f, 0.2f, 0.0f, 0.0f, 0.9f, 0.9f};
        std::vector<float> steerings_diff = {-utils::deg2rad(2), utils::deg2rad(2), 0.0f, 0.0f,
                                             utils::deg2rad(4),  -utils::deg2rad(4)};
        std::vector<bool> left_side = {false, true, false, true, false, true};
        robot_info.controls = {steerings_max, throttles_max, steerings_diff, left_side};

        concord::Size k_size;
        KarosserieInfo kaross;
        kaross.name = "front";
        k_size = concord::Size(width * 1.26f, height * 0.23f, 0.0f);
        kaross.bound = concord::Bound(concord::Pose(0.0f, (height / 2) + k_size.y / 2, 0.0f), k_size);
        kaross.color = color;
        kaross.controllable = true;
        kaross.has_physics = true;
        robot_info.karos.push_back(kaross);

        kaross.name = "back";
        k_size = concord::Size(width * 0.9f, height * 0.15f, 0.0f);
        kaross.bound = concord::Bound(concord::Pose(0.0f, -(height / 2) - k_size.y / 2, 0.0f), k_size);
        kaross.color = color;
        kaross.controllable = false;
        kaross.has_physics = true;
        robot_info.karos.push_back(kaross);

        // color
        pigment::RGB robot_color = color;
        robot_info.color = robot_color;

        // Tank - harvester has a harvest tank in upper 1/3 with padding
        TankInfo harvest_tank;
        harvest_tank.name = "harvest_bin";
        harvest_tank.capacity = 10000.0f; // 10000 units capacity
        // Tank in upper 1/3 of chassis with padding
        float padding = 0.2f; // Padding from edges
        float tank_width = width * (1.0f - 2 * padding);
        float tank_height = height * (1.0f / 2.0f - padding);    // 1/3 height minus padding
        float y_offset = height / 2 - tank_height / 2 - padding; // Position at top
        harvest_tank.bound =
            concord::Bound(concord::Pose(0.0f, y_offset, 0.0f), concord::Size(tank_width, tank_height, 0.0f));
        robot_info.tank = harvest_tank;

        // Power source - fuel tank
        PowerInfo power_info;
        power_info.name = "fuel_tank";
        power_info.type = PowerType::FUEL;
        power_info.capacity = 200.0f;        // 200 liters
        power_info.consumption_rate = 0.05f; // 0.5 liters per second at full work
        robot_info.power_source = power_info;

        return robot_info;
    }

    inline fs::RobotInfo tractor(concord::Pose pose, std::string name, pigment::RGB color = pigment::RGB(255, 200, 0),
                                 std::string uuid = "") {
        // extract dimensions
        const float width = 1.6f;
        const float height = 2.8f;

        fs::RobotInfo robot_info;
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
        std::vector<float> steerings_max = {utils::deg2rad(35), utils::deg2rad(35), 0.0f, 0.0f};
        std::vector<float> throttles_max = {0.0f, 0.0f, 0.2f, 0.2f};
        std::vector<float> steerings_diff = {-utils::deg2rad(4), utils::deg2rad(4), 0.0f, 0.0f};
        std::vector<bool> left_side = {false, true, false, true};
        robot_info.controls = {steerings_max, throttles_max, steerings_diff, left_side};

        // karosserie
        concord::Size k_size = concord::Size(width * 0.50f, height * 0.05f, 0.0f);
        KarosserieInfo kaross;
        kaross.name = "front";
        kaross.bound = concord::Bound(concord::Pose(0.0f, (height / 2) + k_size.y / 2, 0.0f), k_size);
        kaross.color = color;
        kaross.controllable = true;
        kaross.has_physics = true;
        robot_info.karos.push_back(kaross);

        // Small rear extrusion for hitch connection
        concord::Size hitch_size = concord::Size(width * 0.15f, height * 0.1f, 0.0f); // Small extrusion
        KarosserieInfo hitch_mount;
        hitch_mount.name = "hitch_mount";
        hitch_mount.bound = concord::Bound(concord::Pose(0.0f, -(height / 2) - hitch_size.y / 2, 0.0f), hitch_size);
        hitch_mount.color = color;
        hitch_mount.controllable = false;
        hitch_mount.has_physics = false;
        robot_info.karos.push_back(hitch_mount);

        // rear hitch for pulling - at the middle of the extrusion (which is now physicless)
        concord::Bound hitch_bound =
            concord::Bound(concord::Pose(0, -(height / 2) - hitch_size.y / 2, 0.0f), concord::Size(0.05f, 0.05f, 0.0f));
        robot_info.hitches["rear_hitch"] = hitch_bound;

        // color
        pigment::RGB robot_color = color;
        robot_info.color = robot_color;

        // Power source - fuel tank (no harvest tank for tractor)
        PowerInfo power_info;
        power_info.name = "fuel_tank";
        power_info.type = PowerType::FUEL;
        power_info.capacity = 150.0f;        // 150 liters
        power_info.consumption_rate = 0.03f; // 0.3 liters per second at full work
        robot_info.power_source = power_info;
        // No tank for tractor

        return robot_info;
    }

    inline fs::RobotInfo trailer(concord::Pose pose, std::string name, pigment::RGB color = pigment::RGB(255, 200, 0),
                                 std::string uuid = "") {
        // extract dimensions
        const float width = 1.8f;
        const float height = 2.6f;

        fs::RobotInfo robot_info;
        robot_info.RCI = 3;
        robot_info.name = name;
        robot_info.uuid = uuid.empty() ? name : uuid;
        robot_info.type = "trailer";
        robot_info.works_on = {"food"};
        robot_info.role = RobotRole::SLAVE;

        // overall bound
        robot_info.bound = concord::Bound(pose, concord::Size(width, height, 0.0f));

        // wheels: back
        std::vector<concord::Bound> wheels;
        concord::Size back_w_size{width * 0.22f, height * 0.30f, 0.0f};
        wheels.push_back(concord::Bound(concord::Pose(width / 2, -(height / 2) * 0.6f, 0.0f), back_w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, -(height / 2) * 0.6f, 0.0f), back_w_size));
        robot_info.wheels = wheels;

        // control limits
        std::vector<float> steerings_max = {0.0f, 0.0f};
        std::vector<float> throttles_max = {0.0f, 0.0f};
        std::vector<float> steerings_diff = {0.0f, 0.0f};
        std::vector<bool> left_side = {false, true};
        robot_info.controls = {steerings_max, throttles_max, steerings_diff, left_side};

        // Extended karosserie as towing pole
        concord::Size pole_size = concord::Size(width * 0.1f, height * 0.4f, 0.0f); // Thin pole extending forward
        KarosserieInfo pole;
        pole.name = "towing_pole";
        pole.bound = concord::Bound(concord::Pose(0.0f, (height / 2) + pole_size.y / 2, 0.0f), pole_size);
        pole.color = color;
        pole.controllable = false;
        pole.has_physics = true;
        robot_info.karos.push_back(pole);

        // front hitch for being pulled - beyond the pole to avoid collision during rotation
        concord::Bound hitch_bound = concord::Bound(concord::Pose(0, (height / 2) + pole_size.y + 0.2f, 0.0f),
                                                    concord::Size(0.05f, 0.05f, 0.0f));
        robot_info.hitches["front_hitch"] = hitch_bound;

        // color
        pigment::RGB robot_color = color;
        robot_info.color = robot_color;

        // Biner has a tank but no power (carried by tractor)
        TankInfo tank_info;
        tank_info.name = "storage_bin";
        tank_info.capacity = 2000.0f; // 2000 units capacity (larger than harvester)
        // Tank fills most of the biner
        float tank_width = width * 0.9f;
        float tank_height = height * 0.8f;
        tank_info.bound = concord::Bound(concord::Pose(0.0f, 0.0f, 0.0f), // Centered
                                         concord::Size(tank_width, tank_height, 0.0f));
        robot_info.tank = tank_info;
        // No power source - it's carried

        return robot_info;
    }

    inline fs::RobotInfo big_truck(concord::Pose pose, std::string name,
                                   pigment::RGB color = pigment::RGB(100, 100, 255), std::string uuid = "") {
        // extract dimensions
        const float width = 2.5f;
        const float height = 8.0f; // Big truck length
        const float cabin_height = height * 0.25f;

        fs::RobotInfo robot_info;
        robot_info.RCI = 3;
        robot_info.name = name;
        robot_info.uuid = uuid.empty() ? name : uuid;
        robot_info.type = "big_truck";
        robot_info.works_on = {"cargo"};

        // overall bound
        robot_info.bound = concord::Bound(pose, concord::Size(width, height, 0.0f));

        // 8 wheels configuration
        std::vector<concord::Bound> wheels;
        concord::Size w_size{width * 0.18f, height * 0.12f, 0.0f};

        // First two wheels middle of cabin
        float front_y = height / 2 + cabin_height / 2;
        wheels.push_back(concord::Bound(concord::Pose(width / 2, front_y, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, front_y, 0.0f), w_size));

        // Second pair in the middle but a bit forward
        float middle_y = height * 0.2f; // Slightly forward of center
        wheels.push_back(concord::Bound(concord::Pose(width / 2, middle_y, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, middle_y, 0.0f), w_size));

        // Last four wheels close together at the rear
        float rear_y1 = -height / 2 + height * 0.30f;
        float rear_y2 = -height / 2 + height * 0.15f;
        wheels.push_back(concord::Bound(concord::Pose(width / 2, rear_y1, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, rear_y1, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(width / 2, rear_y2, 0.0f), w_size));
        wheels.push_back(concord::Bound(concord::Pose(-width / 2, rear_y2, 0.0f), w_size));

        robot_info.wheels = wheels;

        // control limits - front wheels steer, all wheels have throttle
        std::vector<float> steerings_max = {utils::deg2rad(30), utils::deg2rad(30), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        std::vector<float> throttles_max = {0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f};
        std::vector<float> steerings_diff = {-utils::deg2rad(3), utils::deg2rad(3), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        std::vector<bool> left_side = {false, true, false, true, false, true, false, true};
        robot_info.controls = {steerings_max, throttles_max, steerings_diff, left_side};

        // Cabin karosserie - 1/5 of total length at front
        concord::Size cabin_size = concord::Size(width * 1.05f, cabin_height, 0.0f); // 1/5 of height
        KarosserieInfo cabin;
        cabin.name = "cabin";
        cabin.bound = concord::Bound(concord::Pose(0.0f, height / 2 + cabin_size.y / 2, 0.0f), cabin_size);
        cabin.color = color;
        cabin.controllable = true;
        cabin.has_physics = true;
        robot_info.karos.push_back(cabin);

        // color
        robot_info.color = color;

        // Cargo tank - large capacity for hauling
        TankInfo cargo_tank;
        cargo_tank.name = "cargo_bed";
        cargo_tank.capacity = 50000.0f; // Large capacity
        // Tank fills the rear 4/5 of the truck
        float tank_width = width * 0.9f;
        float tank_height = height * 0.75f;    // Most of the chassis length
        float tank_y_offset = -height * 0.05f; // Positioned in rear portion
        cargo_tank.bound =
            concord::Bound(concord::Pose(0.0f, tank_y_offset, 0.0f), concord::Size(tank_width, tank_height, 0.0f));
        robot_info.tank = cargo_tank;

        // Power source - large fuel tank
        PowerInfo power_info;
        power_info.name = "fuel_tank";
        power_info.type = PowerType::FUEL;
        power_info.capacity = 500.0f;       // 500 liters - large tank
        power_info.consumption_rate = 0.1f; // Higher consumption for big truck
        robot_info.power_source = power_info;

        return robot_info;
    }

} // namespace fs
