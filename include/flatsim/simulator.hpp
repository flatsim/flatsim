#pragma once

#include <memory>
#include <zmq.hpp>

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

      public:
        Simulator(Conn conn, const std::string &address = "", const WorldSettings &settings = {});
        ~Simulator();

        void tick(float dt);
        void tock();

        muli::World &get_world() { return *world_; }
    };

} // namespace simulator
