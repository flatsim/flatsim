// Test SUB socket receiving from simulator
#include <chrono>
#include <iostream>
#include <thread>
#include <zmq.hpp>

int main() {
    std::cout << "[TEST] Connecting to simulator state socket..." << std::endl;

    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, zmq::socket_type::sub);

    // Connect to the agent_001 state socket
    sub.connect("ipc:///tmp/flatsim_state_agent_001");

    // Subscribe to all messages
    sub.set(zmq::sockopt::subscribe, "");
    std::cout << "[TEST] Subscribed to all messages" << std::endl;

    // Wait a bit for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Try to receive 10 messages
    sub.set(zmq::sockopt::rcvtimeo, 1000);
    for (int i = 0; i < 10; i++) {
        zmq::message_t msg;
        auto result = sub.recv(msg, zmq::recv_flags::none);
        if (result) {
            std::cout << "[TEST] Received message " << i << " (" << msg.size() << " bytes)" << std::endl;
        } else {
            std::cout << "[TEST] Timeout waiting for message " << i << std::endl;
        }
    }

    return 0;
}
