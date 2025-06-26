#pragma once

#include <algorithm>
#include <string>
#include "concord/concord.hpp"
#include "pigment/pigment.hpp"
#include "rerun.hpp"

namespace fs {

class Power {
public:
    enum class Type {
        FUEL,
        BATTERY
    };

private:
    Type type;
    float capacity;           // liters for fuel, kWh for battery
    float current_amount;
    float consumption_rate;   // base consumption rate per second
    float charge_rate;        // charge rate per second (for battery)
    std::string name;

public:
    Power(const std::string& name, Type type, float capacity, float consumption_rate, float charge_rate = 0.0f)
        : name(name), type(type), capacity(capacity), 
          current_amount(capacity), consumption_rate(consumption_rate), charge_rate(charge_rate) {}

    // Update power consumption based on operation mode multiplier
    void update(float dt, float consumption_multiplier) {
        if (current_amount > 0.0f) {
            current_amount = std::max(0.0f, current_amount - consumption_rate * consumption_multiplier * dt);
        }
    }

    void charge(float dt) {
        if (type == Type::BATTERY && current_amount < capacity) {
            current_amount = std::min(current_amount + charge_rate * dt, capacity);
        }
    }

    void refuel(float amount) {
        current_amount = std::min(current_amount + amount, capacity);
    }

    void refuel_full() {
        current_amount = capacity;
    }

    float get_percentage() const {
        return capacity > 0.0f ? (current_amount / capacity) * 100.0f : 0.0f;
    }

    bool is_empty() const {
        return current_amount <= 0.0f;
    }

    bool is_low() const {
        return get_percentage() < 15.0f; // Warning threshold at 15%
    }

    bool is_full() const {
        return current_amount >= capacity;
    }

    // Getters
    Type get_type() const { return type; }
    float get_capacity() const { return capacity; }
    float get_current_amount() const { return current_amount; }
    float get_consumption_rate() const { return consumption_rate; }
    float get_charge_rate() const { return charge_rate; }
    const std::string& get_name() const { return name; }
};

} // namespace fs