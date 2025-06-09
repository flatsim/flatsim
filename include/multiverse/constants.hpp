#pragma once

namespace mvs {
    namespace constants {
        // Camera and visualization settings
        inline constexpr bool followCam = true;
        inline constexpr bool rotateCam = false;
        inline constexpr bool drawAxis = false;
        
        // Physics simulation parameters
        inline constexpr float linearDamping = 0.2f;
        inline constexpr float angularDamping = 0.2f;
        inline constexpr float force = 30.0f;
        inline constexpr float torque = 10.0f;
        inline constexpr float friction = 0.4f;
        inline constexpr float maxImpulse = 0.5f;
        inline constexpr float brake = 10.0f;
        inline constexpr float drag = 0.5f;
    } // namespace constants
} // namespace mvs
