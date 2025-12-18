#pragma once

#include <map>
#include <memory>
#include <zmq.hpp>

#include "flatsim/types.hpp"
#include "muli/world.h"

namespace simulator {

    enum class Conn { TCP, IPC };

    struct WorldSettings {
        float width = 100.0f;
        float height = 100.0f;
    };

    class Simulator {
      private:
        // ZMQ
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        Conn conn_;
        std::string address_;

        // Physics
        std::unique_ptr<muli::World> world_;
        WorldSettings world_settings_;

        // Bodies: uuid -> rigid body
        std::map<std::string, muli::RigidBody *> bodies_;

      public:
        Simulator(Conn conn, const std::string &address = "", const WorldSettings &settings = {});
        ~Simulator();

        void tick(float dt);
        void tock();

        muli::RigidBody *create_body(const types::Chassis &chassis);
        muli::World &get_world() { return *world_; }
    };

} // namespace simulator
