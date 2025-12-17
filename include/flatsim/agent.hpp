#pragma once

#include <memory>
#include <zmq.hpp>

namespace agent {

    class Agent {
      private:
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        std::string address_;

      public:
        Agent(const std::string &address = "");
        ~Agent();

        void tick(float dt);
        void tock();
    };

} // namespace agent
