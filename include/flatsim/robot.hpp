#pragma once

#include <memory>
#include <zmq.hpp>

namespace fs::robot {

    class Robot {
      private:
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        std::string address_;

      public:
        Robot(const std::string &address = "");
        ~Robot();

        void tick(float dt);
        void tock();
    };

} // namespace fs::robot
