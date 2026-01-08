// Test large data transfers with netpipe
// Validates that netpipe can handle MB-sized messages

#include <chrono>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <thread>
#include <vector>

void test_tcp_large() {
    std::cout << "\n=== Testing TCP Large Transfer (1MB) ===\n";

    const size_t data_size = 1024 * 1024; // 1MB

    // Server thread
    std::thread server_thread([data_size]() {
        netpipe::TcpStream server;
        netpipe::TcpEndpoint endpoint{"0.0.0.0", 7449};

        auto listen_res = server.listen(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "Server listening on 0.0.0.0:7449\n";

        auto client_res = server.accept();
        if (client_res.is_err()) {
            std::cerr << "Server accept failed: " << client_res.error().message.c_str() << "\n";
            return;
        }
        auto client = std::move(client_res.value());
        std::cout << "Server accepted connection\n";

        // Receive large message
        auto start = std::chrono::high_resolution_clock::now();
        auto recv_res = client->recv();
        auto end = std::chrono::high_resolution_clock::now();

        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);

            std::cout << "Server received " << msg.size() << " bytes in " << duration << "ms (" << throughput
                      << " MB/s)\n";

            // Verify data integrity (check pattern)
            bool valid = true;
            for (size_t i = 0; i < std::min(msg.size(), size_t(1000)); i++) {
                if (msg[i] != static_cast<uint8_t>(i % 256)) {
                    valid = false;
                    break;
                }
            }
            std::cout << "Data integrity: " << (valid ? "OK" : "FAILED") << "\n";

            // Echo back
            start = std::chrono::high_resolution_clock::now();
            client->send(msg);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
            std::cout << "Server echoed in " << duration << "ms (" << throughput << " MB/s)\n";
        }

        client->close();
    });

    // Give server time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Client
    netpipe::TcpStream client;
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 7449};

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "Client connected to 127.0.0.1:7449\n";

    // Create large message with pattern
    netpipe::Message msg(data_size);
    for (size_t i = 0; i < data_size; i++) {
        msg[i] = static_cast<uint8_t>(i % 256);
    }

    // Send large message
    auto start = std::chrono::high_resolution_clock::now();
    auto send_res = client.send(msg);
    auto end = std::chrono::high_resolution_clock::now();

    if (send_res.is_ok()) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "Client sent " << msg.size() << " bytes in " << duration << "ms (" << throughput << " MB/s)\n";
    }

    // Receive echo
    start = std::chrono::high_resolution_clock::now();
    auto recv_res = client.recv();
    end = std::chrono::high_resolution_clock::now();

    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (echo.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "Client received echo: " << echo.size() << " bytes in " << duration << "ms (" << throughput
                  << " MB/s)\n";
    }

    client.close();
    server_thread.join();
    std::cout << "TCP large transfer test completed\n";
}

void test_ipc_large() {
    std::cout << "\n=== Testing IPC Large Transfer (5MB) ===\n";

    const size_t data_size = 5 * 1024 * 1024; // 5MB

    // Server thread
    std::thread server_thread([data_size]() {
        netpipe::IpcStream server;
        netpipe::IpcEndpoint endpoint{"/tmp/flatsim_large.sock"};

        auto listen_res = server.listen_ipc(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "IPC Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "IPC Server listening on /tmp/flatsim_large.sock\n";

        auto client_res = server.accept();
        if (client_res.is_err()) {
            std::cerr << "IPC Server accept failed: " << client_res.error().message.c_str() << "\n";
            return;
        }
        auto client = std::move(client_res.value());
        std::cout << "IPC Server accepted connection\n";

        // Receive large message
        auto start = std::chrono::high_resolution_clock::now();
        auto recv_res = client->recv();
        auto end = std::chrono::high_resolution_clock::now();

        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);

            std::cout << "IPC Server received " << msg.size() << " bytes in " << duration << "ms (" << throughput
                      << " MB/s)\n";

            // Echo back
            start = std::chrono::high_resolution_clock::now();
            client->send(msg);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
            std::cout << "IPC Server echoed in " << duration << "ms (" << throughput << " MB/s)\n";
        }

        client->close();
    });

    // Give server time to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Client
    netpipe::IpcStream client;
    netpipe::IpcEndpoint endpoint{"/tmp/flatsim_large.sock"};

    auto connect_res = client.connect_ipc(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "IPC Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "IPC Client connected to /tmp/flatsim_large.sock\n";

    // Create large message with pattern
    netpipe::Message msg(data_size);
    for (size_t i = 0; i < data_size; i++) {
        msg[i] = static_cast<uint8_t>(i % 256);
    }

    // Send large message
    auto start = std::chrono::high_resolution_clock::now();
    auto send_res = client.send(msg);
    auto end = std::chrono::high_resolution_clock::now();

    if (send_res.is_ok()) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "IPC Client sent " << msg.size() << " bytes in " << duration << "ms (" << throughput << " MB/s)\n";
    }

    // Receive echo
    start = std::chrono::high_resolution_clock::now();
    auto recv_res = client.recv();
    end = std::chrono::high_resolution_clock::now();

    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (echo.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "IPC Client received echo: " << echo.size() << " bytes in " << duration << "ms (" << throughput
                  << " MB/s)\n";
    }

    client.close();
    server_thread.join();
    std::cout << "IPC large transfer test completed\n";
}

void test_shm_large() {
    std::cout << "\n=== Testing SHM Large Transfer (10MB) ===\n";

    const size_t data_size = 10 * 1024 * 1024; // 10MB
    const size_t shm_size = 16 * 1024 * 1024;  // 16MB buffer

    // Server thread
    std::thread server_thread([data_size, shm_size]() {
        netpipe::ShmStream server;
        netpipe::ShmEndpoint endpoint{"flatsim_large_shm", shm_size};

        auto listen_res = server.listen_shm(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "SHM Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "SHM Server created ring buffer: flatsim_large_shm (" << (shm_size / 1024 / 1024) << "MB)\n";

        // Wait for client to connect and send
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Receive large message
        auto start = std::chrono::high_resolution_clock::now();
        auto recv_res = server.recv();
        auto end = std::chrono::high_resolution_clock::now();

        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);

            std::cout << "SHM Server received " << msg.size() << " bytes in " << duration << "ms (" << throughput
                      << " MB/s)\n";

            // Echo back
            start = std::chrono::high_resolution_clock::now();
            server.send(msg);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
            std::cout << "SHM Server echoed in " << duration << "ms (" << throughput << " MB/s)\n";
        }

        server.close();
    });

    // Give server time to create shared memory
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Client
    netpipe::ShmStream client;
    netpipe::ShmEndpoint endpoint{"flatsim_large_shm", shm_size};

    auto connect_res = client.connect_shm(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "SHM Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "SHM Client connected to flatsim_large_shm\n";

    // Create large message with pattern
    netpipe::Message msg(data_size);
    for (size_t i = 0; i < data_size; i++) {
        msg[i] = static_cast<uint8_t>(i % 256);
    }

    // Send large message
    auto start = std::chrono::high_resolution_clock::now();
    auto send_res = client.send(msg);
    auto end = std::chrono::high_resolution_clock::now();

    if (send_res.is_ok()) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "SHM Client sent " << msg.size() << " bytes in " << duration << "ms (" << throughput << " MB/s)\n";
    }

    // Receive echo
    start = std::chrono::high_resolution_clock::now();
    auto recv_res = client.recv();
    end = std::chrono::high_resolution_clock::now();

    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (echo.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "SHM Client received echo: " << echo.size() << " bytes in " << duration << "ms (" << throughput
                  << " MB/s)\n";
    }

    client.close();
    server_thread.join();
    std::cout << "SHM large transfer test completed\n";
}

int main() {
    std::cout << "Testing Netpipe Large Data Transfers\n";
    std::cout << "=====================================\n";

    test_tcp_large();
    test_ipc_large();
    test_shm_large();

    std::cout << "\n✓ All large transfer tests completed\n";
    return 0;
}
