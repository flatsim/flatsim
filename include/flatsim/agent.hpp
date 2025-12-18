#pragma once

#include <memory>
#include <zmq.hpp>

#include "flatsim/types.hpp"

namespace agent {

    class Agent {
      private:
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        std::string address_;
        types::Chassis chassis_;

      public:
        Agent(const std::string &address = "");
        ~Agent();

        void set_chassis(const types::Chassis &chassis);
        bool spawn();

        void tick(float dt);
        void tock();
    };

} // namespace agent
