#pragma once

namespace mvs {
    namespace utils {
        inline float mapper(float x, float in_min, float in_max, float out_min, float out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        inline u_int8_t float_to_byte(float v, float min = 0.0f, float max = 255.0f) {
            v = std::clamp(v, 0.0f, 1.0f);
            float scaled = v * 255.0f;
            float clamped = std::clamp(scaled, min, max);
            return static_cast<uint8_t>(std::round(scaled));
        }
    } // namespace utils
} // namespace mvs
