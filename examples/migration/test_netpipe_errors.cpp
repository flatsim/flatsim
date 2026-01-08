// Test error handling with netpipe
// Validates that netpipe properly handles error conditions

#include <chrono>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <thread>

void test_tcp_connection_refused() {
    std::cout << "\n=== Testing TCP Connection Refused ===\n";

    netpipe::TcpStream client;
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 9999}; // No server listening

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_err()) {
        std::cout << "✓ Connection refused as expected: " << connect_res.error().message.c_str() << "\n";
    } else {
        std::cerr << "✗ Connection should have failed but succeeded\n";
    }
}

void test_tcp_invalid_host() {
    std::cout << "\n=== Testing TCP Invalid Host ===\n";

    netpipe::TcpStream client;
    netpipe::TcpEndpoint endpoint{"invalid.host.that.does.not.exist", 7447};

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_err()) {
        std::cout << "✓ Invalid host rejected as expected: " << connect_res.error().message.c_str() << "\n";
    } else {
        std::cerr << "✗ Invalid host should have failed but succeeded\n";
    }
}

void test_tcp_port_in_use() {
    std::cout << "\n=== Testing TCP Port Already In Use ===\n";

    netpipe::TcpStream server1;
    netpipe::TcpEndpoint endpoint{"0.0.0.0", 7451};

    // First server binds successfully
    auto listen_res1 = server1.listen(endpoint);
    if (listen_res1.is_err()) {
        std::cerr << "First server listen failed: " << listen_res1.error().message.c_str() << "\n";
        return;
    }
    std::cout << "First server listening on 0.0.0.0:7451\n";

    // Second server should fail to bind to same port
    netpipe::TcpStream server2;
    auto listen_res2 = server2.listen(endpoint);
    if (listen_res2.is_err()) {
        std::cout << "✓ Port in use rejected as expected: " << listen_res2.error().message.c_str() << "\n";
    } else {
        std::cerr << "✗ Port in use should have failed but succeeded\n";
    }

    server1.close();
}

void test_ipc_socket_not_found() {
    std::cout << "\n=== Testing IPC Socket Not Found ===\n";

    netpipe::IpcStream client;
    netpipe::IpcEndpoint endpoint{"/tmp/nonexistent_socket.sock"};

    auto connect_res = client.connect_ipc(endpoint);
    if (connect_res.is_err()) {
        std::cout << "✓ Socket not found rejected as expected: " << connect_res.error().message.c_str() << "\n";
    } else {
        std::cerr << "✗ Nonexistent socket should have failed but succeeded\n";
    }
}

void test_ipc_invalid_path() {
    std::cout << "\n=== Testing IPC Invalid Path ===\n";

    netpipe::IpcStream server;
    netpipe::IpcEndpoint endpoint{"/invalid/path/that/does/not/exist/socket.sock"};

    auto listen_res = server.listen_ipc(endpoint);
    if (listen_res.is_err()) {
        std::cout << "✓ Invalid path rejected as expected: " << listen_res.error().message.c_str() << "\n";
    } else {
        std::cerr << "✗ Invalid path should have failed but succeeded\n";
        server.close();
    }
}

void test_shm_invalid_size() {
    std::cout << "\n=== Testing SHM Invalid Size ===\n";

    netpipe::ShmStream server;
    netpipe::ShmEndpoint endpoint{"test_shm", 0}; // Invalid size

    auto listen_res = server.listen_shm(endpoint);
    if (listen_res.is_err()) {
        std::cout << "✓ Invalid size rejected as expected: " << listen_res.error().message.c_str() << "\n";
    } else {
        std::cerr << "✗ Invalid size should have failed but succeeded\n";
        server.close();
    }
}

void test_shm_not_found() {
    std::cout << "\n=== Testing SHM Not Found ===\n";

    netpipe::ShmStream client;
    netpipe::ShmEndpoint endpoint{"nonexistent_shm", 1024};

    auto connect_res = client.connect_shm(endpoint);
    if (connect_res.is_err()) {
        std::cout << "✓ SHM not found rejected as expected: " << connect_res.error().message.c_str() << "\n";
    } else {
        std::cerr << "✗ Nonexistent SHM should have failed but succeeded\n";
    }
}

void test_tcp_send_after_close() {
    std::cout << "\n=== Testing TCP Send After Close ===\n";

    std::thread server_thread([]() {
        netpipe::TcpStream server;
        netpipe::TcpEndpoint endpoint{"0.0.0.0", 7452};
        server.listen(endpoint);
        auto client_res = server.accept();
        if (client_res.is_ok()) {
            auto client = std::move(client_res.value());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            client->close();
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    netpipe::TcpStream client;
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 7452};

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Try to send after server closed
        netpipe::Message msg = {0x01, 0x02, 0x03};
        auto send_res = client.send(msg);
        if (send_res.is_err()) {
            std::cout << "✓ Send after close failed as expected: " << send_res.error().message.c_str() << "\n";
        } else {
            std::cout << "⚠ Send after close succeeded (may be buffered)\n";
        }
    }

    client.close();
    server_thread.join();
}

void test_tcp_recv_timeout() {
    std::cout << "\n=== Testing TCP Receive Timeout ===\n";

    std::thread server_thread([]() {
        netpipe::TcpStream server;
        netpipe::TcpEndpoint endpoint{"0.0.0.0", 7453};
        server.listen(endpoint);
        auto client_res = server.accept();
        if (client_res.is_ok()) {
            auto client = std::move(client_res.value());
            // Don't send anything, just wait
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            client->close();
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    netpipe::TcpStream client;
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 7453};

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_ok()) {
        std::cout << "Client connected, waiting for data that won't come...\n";

        // This will block until server closes or timeout
        auto start = std::chrono::high_resolution_clock::now();
        auto recv_res = client.recv();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (recv_res.is_err()) {
            std::cout << "✓ Receive failed after " << duration << "ms: " << recv_res.error().message.c_str() << "\n";
        } else {
            std::cout << "⚠ Receive succeeded with " << recv_res.value().size() << " bytes (connection closed)\n";
        }
    }

    client.close();
    server_thread.join();
}

void test_message_size_mismatch() {
    std::cout << "\n=== Testing Message Size Handling ===\n";

    std::thread server_thread([]() {
        netpipe::TcpStream server;
        netpipe::TcpEndpoint endpoint{"0.0.0.0", 7454};
        server.listen(endpoint);
        auto client_res = server.accept();
        if (client_res.is_ok()) {
            auto client = std::move(client_res.value());

            // Receive message
            auto recv_res = client->recv();
            if (recv_res.is_ok()) {
                auto msg = recv_res.value();
                std::cout << "Server received message of size: " << msg.size() << " bytes\n";

                // Send back different size
                netpipe::Message response(msg.size() * 2, 0xFF);
                client->send(response);
            }
            client->close();
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    netpipe::TcpStream client;
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 7454};

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_ok()) {
        // Send small message
        netpipe::Message msg(100, 0xAA);
        client.send(msg);

        // Receive larger response
        auto recv_res = client.recv();
        if (recv_res.is_ok()) {
            auto response = recv_res.value();
            std::cout << "✓ Client received response of size: " << response.size() << " bytes\n";
            std::cout << "✓ Different message sizes handled correctly\n";
        }
    }

    client.close();
    server_thread.join();
}

int main() {
    std::cout << "Testing Netpipe Error Handling\n";
    std::cout << "===============================\n";

    // TCP error tests
    test_tcp_connection_refused();
    test_tcp_invalid_host();
    test_tcp_port_in_use();
    test_tcp_send_after_close();
    test_tcp_recv_timeout();
    test_message_size_mismatch();

    // IPC error tests
    test_ipc_socket_not_found();
    test_ipc_invalid_path();

    // SHM error tests
    test_shm_invalid_size();
    test_shm_not_found();

    std::cout << "\n✓ All error handling tests completed\n";
    return 0;
}
