// Test basic netpipe stream functionality
// This validates that netpipe TCP/IPC/SHM streams work before we build the wrapper

#include <chrono>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <thread>

void test_tcp_stream() {
    std::cout << "\n=== Testing TCP Stream ===\n";

    // Server thread
    std::thread server_thread([]() {
        netpipe::TcpStream server;
        netpipe::TcpEndpoint endpoint{"0.0.0.0", 7447};

        auto listen_res = server.listen(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "Server listening on 0.0.0.0:7447\n";

        auto client_res = server.accept();
        if (client_res.is_err()) {
            std::cerr << "Server accept failed: " << client_res.error().message.c_str() << "\n";
            return;
        }
        auto client = std::move(client_res.value());
        std::cout << "Server accepted connection\n";

        // Receive message
        auto recv_res = client->recv();
        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            std::cout << "Server received " << msg.size() << " bytes\n";

            // Echo back
            client->send(msg);
            std::cout << "Server echoed message\n";
        }

        client->close();
    });

    // Give server time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Client
    netpipe::TcpStream client;
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 7447};

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "Client connected to 127.0.0.1:7447\n";

    // Send message
    netpipe::Message msg = {0x48, 0x65, 0x6c, 0x6c, 0x6f}; // "Hello"
    auto send_res = client.send(msg);
    if (send_res.is_ok()) {
        std::cout << "Client sent " << msg.size() << " bytes\n";
    }

    // Receive echo
    auto recv_res = client.recv();
    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        std::cout << "Client received echo: " << echo.size() << " bytes\n";
    }

    client.close();
    server_thread.join();
    std::cout << "TCP test completed\n";
}

void test_ipc_stream() {
    std::cout << "\n=== Testing IPC Stream ===\n";

    // Server thread
    std::thread server_thread([]() {
        netpipe::IpcStream server;
        netpipe::IpcEndpoint endpoint{"/tmp/flatsim_test.sock"};

        auto listen_res = server.listen_ipc(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "IPC Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "IPC Server listening on /tmp/flatsim_test.sock\n";

        auto client_res = server.accept();
        if (client_res.is_err()) {
            std::cerr << "IPC Server accept failed: " << client_res.error().message.c_str() << "\n";
            return;
        }
        auto client = std::move(client_res.value());
        std::cout << "IPC Server accepted connection\n";

        // Receive message
        auto recv_res = client->recv();
        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            std::cout << "IPC Server received " << msg.size() << " bytes\n";

            // Echo back
            client->send(msg);
            std::cout << "IPC Server echoed message\n";
        }

        client->close();
    });

    // Give server time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Client
    netpipe::IpcStream client;
    netpipe::IpcEndpoint endpoint{"/tmp/flatsim_test.sock"};

    auto connect_res = client.connect_ipc(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "IPC Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "IPC Client connected to /tmp/flatsim_test.sock\n";

    // Send message
    netpipe::Message msg = {0x49, 0x50, 0x43}; // "IPC"
    auto send_res = client.send(msg);
    if (send_res.is_ok()) {
        std::cout << "IPC Client sent " << msg.size() << " bytes\n";
    }

    // Receive echo
    auto recv_res = client.recv();
    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        std::cout << "IPC Client received echo: " << echo.size() << " bytes\n";
    }

    client.close();
    server_thread.join();
    std::cout << "IPC test completed\n";
}

void test_shm_stream() {
    std::cout << "\n=== Testing SHM Stream ===\n";

    // Server thread
    std::thread server_thread([]() {
        netpipe::ShmStream server;
        netpipe::ShmEndpoint endpoint{"flatsim_test_shm", 1024 * 1024}; // 1MB

        auto listen_res = server.listen_shm(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "SHM Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "SHM Server created ring buffer: flatsim_test_shm (1MB)\n";

        // SHM doesn't have accept() - it's a direct 1:1 connection
        // Wait for client to connect and send
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Receive message
        auto recv_res = server.recv();
        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            std::cout << "SHM Server received " << msg.size() << " bytes\n";

            // Echo back
            server.send(msg);
            std::cout << "SHM Server echoed message\n";
        }

        server.close();
    });

    // Give server time to create shared memory
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Client
    netpipe::ShmStream client;
    netpipe::ShmEndpoint endpoint{"flatsim_test_shm", 1024 * 1024};

    auto connect_res = client.connect_shm(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "SHM Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "SHM Client connected to flatsim_test_shm\n";

    // Send message
    netpipe::Message msg = {0x53, 0x48, 0x4d}; // "SHM"
    auto send_res = client.send(msg);
    if (send_res.is_ok()) {
        std::cout << "SHM Client sent " << msg.size() << " bytes\n";
    }

    // Receive echo
    auto recv_res = client.recv();
    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        std::cout << "SHM Client received echo: " << echo.size() << " bytes\n";
    }

    client.close();
    server_thread.join();
    std::cout << "SHM test completed\n";
}

int main() {
    std::cout << "Testing Netpipe Streams\n";
    std::cout << "========================\n";

    test_tcp_stream();
    test_ipc_stream();
    test_shm_stream();

    std::cout << "\n✓ All tests completed\n";
    return 0;
}
