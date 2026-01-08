// Test concurrent netpipe connections
// Validates multiple simultaneous client connections work correctly

#include <chrono>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <thread>
#include <vector>

void test_tcp_concurrent() {
    std::cout << "\n=== Testing TCP Concurrent Connections ===\n";

    const int num_clients = 5;
    std::atomic<int> clients_connected{0};
    std::atomic<int> messages_received{0};

    // Server thread
    std::thread server_thread([&]() {
        netpipe::TcpStream server;
        netpipe::TcpEndpoint endpoint{"0.0.0.0", 7448};

        auto listen_res = server.listen(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "Server listening on 0.0.0.0:7448\n";

        // Accept multiple clients
        std::vector<std::thread> client_handlers;
        for (int i = 0; i < num_clients; i++) {
            auto client_res = server.accept();
            if (client_res.is_err()) {
                std::cerr << "Server accept failed: " << client_res.error().message.c_str() << "\n";
                continue;
            }
            auto client = std::move(client_res.value());
            clients_connected++;
            std::cout << "Server accepted client " << (i + 1) << "\n";

            // Handle each client in separate thread
            client_handlers.emplace_back([client = std::move(client), &messages_received]() mutable {
                auto recv_res = client->recv();
                if (recv_res.is_ok()) {
                    auto msg = recv_res.value();
                    messages_received++;
                    // Echo back
                    client->send(msg);
                }
                client->close();
            });
        }

        // Wait for all handlers to complete
        for (auto &handler : client_handlers) {
            handler.join();
        }
    });

    // Give server time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Create multiple client threads
    std::vector<std::thread> client_threads;
    for (int i = 0; i < num_clients; i++) {
        client_threads.emplace_back([i]() {
            netpipe::TcpStream client;
            netpipe::TcpEndpoint endpoint{"127.0.0.1", 7448};

            auto connect_res = client.connect(endpoint);
            if (connect_res.is_err()) {
                std::cerr << "Client " << i << " connect failed: " << connect_res.error().message.c_str() << "\n";
                return;
            }

            // Send unique message
            netpipe::Message msg = {static_cast<uint8_t>('A' + i), static_cast<uint8_t>('0' + i)};
            auto send_res = client.send(msg);
            if (send_res.is_ok()) {
                std::cout << "Client " << i << " sent message\n";
            }

            // Receive echo
            auto recv_res = client.recv();
            if (recv_res.is_ok()) {
                std::cout << "Client " << i << " received echo\n";
            }

            client.close();
        });
    }

    // Wait for all clients to complete
    for (auto &thread : client_threads) {
        thread.join();
    }

    server_thread.join();

    std::cout << "Concurrent test completed: " << clients_connected << "/" << num_clients << " clients connected, "
              << messages_received << "/" << num_clients << " messages received\n";
}

void test_ipc_concurrent() {
    std::cout << "\n=== Testing IPC Concurrent Connections ===\n";

    const int num_clients = 3;
    std::atomic<int> clients_connected{0};
    std::atomic<int> messages_received{0};

    // Server thread
    std::thread server_thread([&]() {
        netpipe::IpcStream server;
        netpipe::IpcEndpoint endpoint{"/tmp/flatsim_concurrent.sock"};

        auto listen_res = server.listen_ipc(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "IPC Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "IPC Server listening on /tmp/flatsim_concurrent.sock\n";

        // Accept multiple clients
        std::vector<std::thread> client_handlers;
        for (int i = 0; i < num_clients; i++) {
            auto client_res = server.accept();
            if (client_res.is_err()) {
                std::cerr << "IPC Server accept failed: " << client_res.error().message.c_str() << "\n";
                continue;
            }
            auto client = std::move(client_res.value());
            clients_connected++;
            std::cout << "IPC Server accepted client " << (i + 1) << "\n";

            // Handle each client in separate thread
            client_handlers.emplace_back([client = std::move(client), &messages_received]() mutable {
                auto recv_res = client->recv();
                if (recv_res.is_ok()) {
                    auto msg = recv_res.value();
                    messages_received++;
                    // Echo back
                    client->send(msg);
                }
                client->close();
            });
        }

        // Wait for all handlers to complete
        for (auto &handler : client_handlers) {
            handler.join();
        }
    });

    // Give server time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Create multiple client threads
    std::vector<std::thread> client_threads;
    for (int i = 0; i < num_clients; i++) {
        client_threads.emplace_back([i]() {
            // Stagger client connections slightly
            std::this_thread::sleep_for(std::chrono::milliseconds(50 * i));

            netpipe::IpcStream client;
            netpipe::IpcEndpoint endpoint{"/tmp/flatsim_concurrent.sock"};

            auto connect_res = client.connect_ipc(endpoint);
            if (connect_res.is_err()) {
                std::cerr << "IPC Client " << i << " connect failed: " << connect_res.error().message.c_str() << "\n";
                return;
            }

            // Send unique message
            netpipe::Message msg = {static_cast<uint8_t>('I' + i), static_cast<uint8_t>('P' + i),
                                    static_cast<uint8_t>('C' + i)};
            auto send_res = client.send(msg);
            if (send_res.is_ok()) {
                std::cout << "IPC Client " << i << " sent message\n";
            }

            // Receive echo
            auto recv_res = client.recv();
            if (recv_res.is_ok()) {
                std::cout << "IPC Client " << i << " received echo\n";
            }

            client.close();
        });
    }

    // Wait for all clients to complete
    for (auto &thread : client_threads) {
        thread.join();
    }

    server_thread.join();

    std::cout << "IPC Concurrent test completed: " << clients_connected << "/" << num_clients << " clients connected, "
              << messages_received << "/" << num_clients << " messages received\n";
}

int main() {
    std::cout << "Testing Netpipe Concurrent Connections\n";
    std::cout << "=======================================\n";

    test_tcp_concurrent();
    test_ipc_concurrent();

    std::cout << "\n✓ All concurrent tests completed\n";
    return 0;
}
