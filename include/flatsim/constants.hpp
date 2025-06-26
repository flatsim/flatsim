#pragma once

namespace fs {
    namespace constants {
        // Camera and visualization settings
        inline constexpr bool followCam = true;
        inline constexpr bool rotateCam = false;
        inline constexpr bool drawAxis = false;
        
        // Physics simulation parameters
        inline constexpr float linearDamping = 0.2f;
        inline constexpr float angularDamping = 0.2f;
        inline constexpr float force = 30.0f;        // Base force for typical wheel (0.2m radius)
        inline constexpr float torque = 10.0f;
        inline constexpr float friction = 0.4f;      // Base lateral friction
        inline constexpr float maxImpulse = 0.5f;    // Base max impulse for lateral friction
        inline constexpr float brake = 10.0f;
        inline constexpr float drag = 0.5f;          // Velocity-dependent drag coefficient
    } // namespace constants
} // namespace fs
