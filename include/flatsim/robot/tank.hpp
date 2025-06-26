#pragma once

#include <algorithm>
#include <string>
#include "concord/concord.hpp"
#include "pigment/pigment.hpp"
#include "rerun.hpp"
#include "flatsim/utils.hpp"

namespace fs {

class Tank {
public:
    enum class Type {
        HARVEST,    // For storing harvested material
        WASTE       // For other types of tanks (future use)
    };

private:
    Type type;
    float capacity;
    float current_amount;
    float fill_rate;      // units per second when filling
    float empty_rate;     // units per second when emptying (fast for harvest)
    std::string name;
    std::string parent_name;
    concord::Bound bound;  // Visual representation bound
    concord::Pose pose;    // Current world position/rotation
    pigment::RGB color;

public:
    Tank(const std::string& name, Type type, float capacity, float fill_rate, float empty_rate)
        : name(name), type(type), capacity(capacity), current_amount(0.0f),
          fill_rate(fill_rate), empty_rate(empty_rate) {}

    void update(float dt, bool filling, bool emptying) {
        // Tank filling is now handled by fill() method based on harvest amount
        // Emptying is instant via empty_all()
    }

    void fill(float amount) {
        current_amount = std::min(current_amount + amount, capacity);
    }

    void empty(float amount) {
        current_amount = std::max(current_amount - amount, 0.0f);
    }

    void empty_all() {
        current_amount = 0.0f;
    }

    float get_percentage() const {
        return capacity > 0.0f ? (current_amount / capacity) * 100.0f : 0.0f;
    }

    bool is_full() const {
        return current_amount >= capacity;
    }

    bool is_empty() const {
        return current_amount <= 0.0f;
    }

    // Getters
    Type get_type() const { return type; }
    float get_capacity() const { return capacity; }
    float get_current_amount() const { return current_amount; }
    float get_fill_rate() const { return fill_rate; }
    float get_empty_rate() const { return empty_rate; }
    const std::string& get_name() const { return name; }
    
    void init(const pigment::RGB& tank_color, const std::string& parent_name, const concord::Bound& tank_bound) {
        this->color = tank_color;
        this->parent_name = parent_name;
        this->bound = tank_bound;
    }
    
    void tick(float dt, concord::Pose trans_pose, std::shared_ptr<rerun::RecordingStream> rec) {
        auto new_pose = utils::move(bound.pose, trans_pose);
        
        pose.point.x = new_pose.point.x;
        pose.point.y = new_pose.point.y;
        pose.angle.yaw = new_pose.angle.yaw;
        
        visualize(rec);
    }
    
    void visualize(std::shared_ptr<rerun::RecordingStream> rec) {
        if (!rec) return;
        
        auto t_x = pose.point.x;
        auto t_y = pose.point.y;
        auto t_th = pose.angle.yaw;
        auto t_w = float(bound.size.x);
        auto t_h = float(bound.size.y);
        
        // Visualize tank fill level
        float fill_percentage = get_percentage() / 100.0f;
        if (fill_percentage > 0.0f) {
            // Calculate filled portion height
            float filled_height = t_h * fill_percentage;
            
            // Position the filled part at the bottom of the tank
            float y_offset = (t_h - filled_height) / 2.0f;
            float filled_x = t_x - y_offset * sin(t_th);
            float filled_y = t_y - y_offset * cos(t_th);
            
            std::vector<rerun::Color> fill_colors;
            fill_colors.push_back(rerun::Color(100, 200, 100));  // Green for harvest
            
            std::vector<rerun::components::PoseTranslation3D> fill_centers = {
                rerun::components::PoseTranslation3D(filled_x, filled_y, 0.12f)};
            std::vector<rerun::datatypes::Vec3D> fill_sizes = {
                rerun::datatypes::Vec3D(t_w, filled_height, 0.0f)};
            
            rec->log_static(
                parent_name + "/chassis/tank/" + name + "/fill",
                rerun::Boxes3D::from_centers_and_sizes(fill_centers, fill_sizes)
                    .with_radii({{0.01f}})
                    .with_fill_mode(rerun::FillMode::Solid)
                    .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(t_th))})
                    .with_colors(fill_colors));
        }
        
        // Tank outline
        std::vector<rerun::Color> outline_colors;
        outline_colors.push_back(rerun::Color(80, 80, 80));  // Dark gray outline
        
        std::vector<rerun::components::PoseTranslation3D> outline_centers = {
            rerun::components::PoseTranslation3D(t_x, t_y, 0.11f)};
        std::vector<rerun::datatypes::Vec3D> outline_sizes = {
            rerun::datatypes::Vec3D(t_w, t_h, 0.0f)};
        
        rec->log_static(
            parent_name + "/chassis/tank/" + name + "/outline",
            rerun::Boxes3D::from_centers_and_sizes(outline_centers, outline_sizes)
                .with_radii({{0.02f}})
                .with_fill_mode(rerun::FillMode::MajorWireframe)
                .with_rotation_axis_angles({rerun::RotationAxisAngle({0.0f, 0.0f, 1.0f}, rerun::Angle::radians(t_th))})
                .with_colors(outline_colors));
    }
};

} // namespace fs