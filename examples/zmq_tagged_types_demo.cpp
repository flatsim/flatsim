#include <cassert>
#include "flatsim/utils.hpp"
#include <cista/serialization.h>
#include "flatsim/utils.hpp"
#include <cstdint>
#include "flatsim/utils.hpp"
#include <iostream>
#include "flatsim/utils.hpp"
#include <thread>
#include "flatsim/utils.hpp"
#include <vector>
#include "flatsim/utils.hpp"
#include <zmq.hpp>
#include "flatsim/utils.hpp"

#include "flatsim/types.hpp"
#include "flatsim/utils.hpp"

namespace {

enum class MsgKind : uint8_t { State = 1, Sensors = 2 };

template <typename T> std::vector<uint8_t> pack_tagged(MsgKind kind, const T &payload) {
    auto data = cista::serialize(payload);
    std::vector<uint8_t> out;
    out.reserve(1 + data.size());
    out.push_back(static_cast<uint8_t>(kind));
    out.insert(out.end(), data.begin(), data.end());
    return out;
}

template <typename T> T unpack_cista(std::vector<uint8_t> payload) {
    auto *ptr = cista::deserialize<T>(payload);
    assert(ptr != nullptr);
    return *ptr;
}

std::vector<uint8_t> recv_bytes(zmq::socket_t &sock) {
    zmq::message_t msg;
    auto ok = sock.recv(msg, zmq::recv_flags::none);
    assert(ok.has_value());
    const auto *begin = static_cast<const uint8_t *>(msg.data());
    return std::vector<uint8_t>(begin, begin + msg.size());
}

void send_bytes(zmq::socket_t &sock, const std::vector<uint8_t> &bytes) {
    auto ok = sock.send(zmq::buffer(bytes), zmq::send_flags::none);
    assert(ok.has_value());
}

} // namespace

int main() {
    zmq::context_t ctx(1);
    const char *endpoint = "inproc://flatsim_zmq_tagged_types_demo";

    zmq::socket_t pull(ctx, zmq::socket_type::pull);
    pull.bind(endpoint);

    zmq::socket_t push(ctx, zmq::socket_type::push);
    push.connect(endpoint);

    const std::string uuid = "demo-uuid";

    types::ser::MachineState state_out;
    state_out.uuid = uuid;
    state_out.pose.position.x = 1.0f;
    state_out.pose.position.y = 2.0f;
    state_out.pose.angle = 0.5f;
    state_out.velocity.x = 3.0f;
    state_out.velocity.y = 4.0f;
    state_out.angular_vel = 5.0f;

    types::ser::SensorState sensors_out;
    sensors_out.uuid = uuid;
    sensors_out.has_gps = true;
    sensors_out.gps.latitude = 52.0;
    sensors_out.gps.longitude = 13.0;
    sensors_out.gps.altitude = 100.0;
    sensors_out.gps.heading = 1.25f;
    sensors_out.gps.speed = 6.5f;

    send_bytes(push, pack_tagged(MsgKind::State, state_out));
    send_bytes(push, pack_tagged(MsgKind::Sensors, sensors_out));

    bool got_state = false;
    bool got_sensors = false;

    for (int i = 0; i < 2; ++i) {
        const auto bytes = recv_bytes(pull);
        assert(bytes.size() >= 2);

        const auto kind = static_cast<MsgKind>(bytes[0]);
        std::vector<uint8_t> payload(bytes.begin() + 1, bytes.end());

        switch (kind) {
        case MsgKind::State: {
            const auto state_in = unpack_cista<types::ser::MachineState>(std::move(payload));
            assert(std::string(state_in.uuid.view()) == uuid);
            assert(state_in.pose.position.x == state_out.pose.position.x);
            assert(state_in.pose.position.y == state_out.pose.position.y);
            assert(state_in.pose.angle == state_out.pose.angle);
            assert(state_in.velocity.x == state_out.velocity.x);
            assert(state_in.velocity.y == state_out.velocity.y);
            assert(state_in.angular_vel == state_out.angular_vel);
            got_state = true;
            break;
        }
        case MsgKind::Sensors: {
            const auto sensors_in = unpack_cista<types::ser::SensorState>(std::move(payload));
            assert(std::string(sensors_in.uuid.view()) == uuid);
            assert(sensors_in.has_gps == true);
            assert(sensors_in.gps.latitude == sensors_out.gps.latitude);
            assert(sensors_in.gps.longitude == sensors_out.gps.longitude);
            assert(sensors_in.gps.altitude == sensors_out.gps.altitude);
            assert(sensors_in.gps.heading == sensors_out.gps.heading);
            assert(sensors_in.gps.speed == sensors_out.gps.speed);
            got_sensors = true;
            break;
        }
        default:
            assert(false && "unexpected MsgKind");
        }
    }

    assert(got_state && got_sensors);
    std::cout << "OK: sent/received types::ser::MachineState + types::ser::SensorState on one ZMQ socket "
                 "using a 1-byte message kind tag\n";
    return 0;
}
