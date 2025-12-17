#pragma once

#include <memory>
#include <zmq.hpp>

namespace fs::simulator {

    enum class Conn { TCP, IPC };

    class Simulator {
      private:
        zmq::context_t ctx_;
        std::unique_ptr<zmq::socket_t> socket_;
        Conn conn_;
        std::string address_;

      public:
        Simulator(Conn conn, const std::string &address = "");
        ~Simulator();

        void tick(float dt);
        void tock();
    };

} // namespace fs::simulator
