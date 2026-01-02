#pragma once

#include <cstdint>
#include <datapod/serialization/serialize.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

namespace flatsim::wire {

    enum class Kind : uint8_t {
        STATE = 1,
        SENSORS = 2,
        CONTROL = 3,
        HEARTBEAT = 4,
        LIDAR_CFG = 5,
    };

    template <typename T> inline std::vector<uint8_t> pack(Kind kind, T &payload) {
        auto data = datapod::serialize(payload);
        std::vector<uint8_t> out;
        out.reserve(1 + data.size());
        out.push_back(static_cast<uint8_t>(kind));
        out.insert(out.end(), data.begin(), data.end());
        return out;
    }

    struct TaggedBytes {
        Kind kind{};
        std::vector<uint8_t> payload;
    };

    inline TaggedBytes unpack(std::vector<uint8_t> bytes) {
        if (bytes.size() < 2) {
            throw std::runtime_error("tagged_zmq: message too small");
        }
        TaggedBytes t;
        t.kind = static_cast<Kind>(bytes[0]);
        t.payload.assign(bytes.begin() + 1, bytes.end());
        return t;
    }

    template <typename T> inline T deserialize(std::vector<uint8_t> payload) {
        return datapod::deserialize<datapod::Mode::NONE, T>(payload);
    }

} // namespace flatsim::wire
