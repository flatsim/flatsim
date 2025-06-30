#include "flatsim/robot/chassis/karosserie.hpp"

namespace fs {
    Karosserie::Karosserie(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<muli::World> world)
        : rec(rec), world(world) {}

    void Karosserie::init(const pigment::RGB &color, const std::string &parent_name, const std::string &name,
                          concord::Bound parent_bound, concord::Bound bound, muli::CollisionFilter filter,
                          int num_sections, bool has_physics) {
        this->name = name;
        this->parent_name = parent_name;
        this->color = color;
        this->bound = bound;
        this->has_physics = has_physics;

        pose = utils::shift(parent_bound.pose, bound.pose);

        // Create sections if num_sections > 0
        if (num_sections > 0) {
            sections.clear();
            sections.reserve(num_sections);

            // Divide the karosserie width equally among sections
            float section_width = bound.size.x / static_cast<float>(num_sections);
            float start_x = bound.pose.point.x - (bound.size.x / 2.0f) + (section_width / 2.0f);

            for (int i = 0; i < num_sections; ++i) {
                Section section(rec);

                // Calculate section position
                float section_x = start_x + (i * section_width);
                concord::Pose section_pose = bound.pose;
                section_pose.point.x = section_x;

                // Create section bound with divided width
                concord::Bound section_bound(section_pose, concord::Size(section_width, bound.size.y, bound.size.z));

                section.init(color, parent_name, name + "_section", section_bound, i);
                sections.push_back(section);
            }
        }

        // Physics karosseries are now handled as compound shapes in chassis
        // No separate RigidBody needed - they're part of the chassis body
        if (has_physics) {
            // Physics handled by chassis compound shape
            karosserie = nullptr;
        }
    }

    void Karosserie::tick(float dt, concord::Pose trans_pose) {
        auto new_pose = utils::move(bound.pose, trans_pose);

        if (has_physics && karosserie) {
            // Only update transform if it's a separate body (not part of compound shape)
            karosserie->SetTransform(utils::pose_to_transform(new_pose));
        }

        pose.point.x = new_pose.point.x;
        pose.point.y = new_pose.point.y;
        pose.angle.yaw = new_pose.angle.yaw;

        // Update sections
        for (auto &section : sections) {
            section.tick(dt, trans_pose);
        }
    }

    void Karosserie::teleport(concord::Pose trans_pose) {
        pose = trans_pose;
        if (has_physics && karosserie) {
            // Only update if it's a separate body (not part of compound shape)
            karosserie->SetTransform(utils::pose_to_transform(trans_pose));
            karosserie->SetSleeping(true);
        }

        // Teleport sections
        for (auto &section : sections) {
            section.teleport(trans_pose);
        }
    }

    muli::Transform Karosserie::get_transform() const {
        if (karosserie) {
            return karosserie->GetTransform();
        }
        // For compound shape karosseries, return identity transform
        return muli::Transform();
    }

    muli::RigidBody *Karosserie::get_body() const {
        return karosserie; // Will be nullptr for compound shape karosseries
    }

    void Karosserie::tock() {
        // Only show karosserie if it has no sections, otherwise show individual sections
        if (!sections.empty()) {
            for (auto &section : sections) section.tock();
        } else {
            auto k_x = pose.point.x;
            auto k_y = pose.point.y;
            auto k_th = pose.angle.yaw;
            auto k_w = float(bound.size.x);
            auto k_h = float(bound.size.y);
            std::vector<rerun::Color> colors_a;
            colors_a.push_back(rerun::Color(color.r, color.g, color.b));
            std::vector<rerun::components::PoseTranslation3D> centers = {
                rerun::components::PoseTranslation3D(float(k_x), float(k_y), 0.1f)};
            std::vector<rerun::datatypes::Vec3D> sizes = {rerun::datatypes::Vec3D(float(k_w), float(k_h), 0.0f)};
            rec->log_static(
                this->parent_name + "/chassis/karosserie/" + name,
                rerun::Boxes3D::from_centers_and_sizes(centers, sizes)
                    .with_radii({{0.02f}})
                    .with_fill_mode(this->working ? rerun::FillMode::Solid : rerun::FillMode::MajorWireframe)
                    .with_rotation_axis_angles(
                        {rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(k_th))})
                    .with_colors(colors_a));
        }
    }

    void Karosserie::toggle_section_work(int section_id) {
        if (section_id >= 0 && section_id < static_cast<int>(sections.size())) {
            sections[section_id].toggle_work();
        }
    }

    void Karosserie::toggle_all_sections_work() {
        for (auto &section : sections) {
            section.toggle_work();
        }
    }

    void Karosserie::toggle_all_except_section_work(int except_section_id) {
        for (int i = 0; i < static_cast<int>(sections.size()); ++i) {
            if (i != except_section_id) {
                sections[i].toggle_work();
            }
        }
    }
} // namespace fs
