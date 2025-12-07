#include "flatsim/robot/chassis_manager.hpp"
#include "flatsim/robot.hpp"

namespace fs {

    void ChassisManager::init(Robot *r, std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world,
                              muli::CollisionFilter filter, RobotInfo &robo) {
        robot = r;
        this->rec = rec;
        this->world = world;
        this->filter = filter;

        chassis = std::make_unique<Chassis>(world, rec, filter, &robot->info, &robot->state);
        chassis->init(robo);
    }

    void ChassisManager::tick(float dt) {
        if (chassis) {
            chassis->tick(dt);
        }
    }

    void ChassisManager::tock(const std::string &label) {
        if (chassis) {
            chassis->tock(label);
        }
    }

    void ChassisManager::update(const std::vector<float> &steering, const std::vector<float> &throttle, float dt) {
        if (chassis) {
            chassis->update(steering, throttle, dt);
        }
    }

    void ChassisManager::toggle_section_work(const std::string &karosserie_name, int section_id) {
        if (chassis) {
            chassis->toggle_section_work(karosserie_name, section_id);
        }
    }

    void ChassisManager::toggle_all_sections_work(const std::string &karosserie_name) {
        if (chassis) {
            chassis->toggle_all_sections_work(karosserie_name);
        }
    }

    void ChassisManager::toggle_all_except_section_work(const std::string &karosserie_name, int except_section_id) {
        if (chassis) {
            chassis->toggle_all_except_section_work(karosserie_name, except_section_id);
        }
    }

    void ChassisManager::set_wheel_damping(float linear_damping, float angular_damping) {
        if (chassis) {
            chassis->wheel_damping(linear_damping, angular_damping);
        }
    }

    void ChassisManager::teleport(concord::Pose pose) {
        if (chassis) {
            chassis->teleport(pose);
        }
    }

    void ChassisManager::update_color(const pigment::RGB &new_color) {
        if (chassis) {
            chassis->update_color(new_color);
        }
    }

    concord::Pose ChassisManager::get_pose() const {
        if (chassis) {
            return chassis->get_pose();
        }
        return concord::Pose{};
    }

    muli::Transform ChassisManager::get_transform() const {
        if (chassis) {
            return chassis->get_transform();
        }
        return muli::Transform{};
    }

    muli::RigidBody *ChassisManager::get_body() {
        if (chassis) {
            return chassis->get_body();
        }
        return nullptr;
    }

    const muli::RigidBody *ChassisManager::get_body() const {
        if (chassis) {
            return chassis->get_body();
        }
        return nullptr;
    }

    std::vector<Karosserie> *ChassisManager::get_karosseries() {
        if (chassis) {
            return &chassis->karosseries;
        }
        return nullptr;
    }

    std::vector<Hitch> *ChassisManager::get_hitches() {
        if (chassis) {
            return &chassis->hitches;
        }
        return nullptr;
    }

    const concord::Bound &ChassisManager::get_bound() const {
        if (chassis) {
            return chassis->get_bound();
        }
        static concord::Bound empty_bound{};
        return empty_bound;
    }

} // namespace fs
