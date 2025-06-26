#include "include/flatsim/robot/chassis/chassis.hpp"
#include "include/flatsim/robot/tank.hpp"
#include "include/flatsim/types.hpp"

int main() {
    // Test basic compilation
    fs::Tank tank("test", fs::Tank::Type::HARVEST, 100.0f, 0.0f, 0.0f);
    
    fs::TankInfo tank_info;
    tank_info.name = "test_tank";
    tank_info.capacity = 100.0f;
    
    return 0;
}