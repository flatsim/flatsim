// Test extra large data transfers with netpipe
// Validates that netpipe can handle 10s of MB-sized messages

#include <chrono>
#include <iostream>
#include <netpipe/netpipe.hpp>
#include <thread>
#include <vector>

void test_tcp_xlarge() {
    std::cout << "\n=== Testing TCP Extra Large Transfer (50MB) ===\n";

    const size_t data_size = 50 * 1024 * 1024; // 50MB

    // Server thread
    std::thread server_thread([data_size]() {
        netpipe::TcpStream server;
        netpipe::TcpEndpoint endpoint{"0.0.0.0", 7450};

        auto listen_res = server.listen(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "Server listening on 0.0.0.0:7450\n";

        auto client_res = server.accept();
        if (client_res.is_err()) {
            std::cerr << "Server accept failed: " << client_res.error().message.c_str() << "\n";
            return;
        }
        auto client = std::move(client_res.value());
        std::cout << "Server accepted connection\n";

        // Receive extra large message
        auto start = std::chrono::high_resolution_clock::now();
        auto recv_res = client->recv();
        auto end = std::chrono::high_resolution_clock::now();

        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);

            std::cout << "Server received " << (msg.size() / 1024 / 1024) << "MB in " << duration << "ms ("
                      << throughput << " MB/s)\n";

            // Verify data integrity (check pattern at multiple points)
            bool valid = true;
            std::vector<size_t> check_points = {0, msg.size() / 4, msg.size() / 2, 3 * msg.size() / 4,
                                                msg.size() - 1000};
            for (size_t point : check_points) {
                for (size_t i = 0; i < 100 && point + i < msg.size(); i++) {
                    if (msg[point + i] != static_cast<uint8_t>((point + i) % 256)) {
                        valid = false;
                        std::cerr << "Data corruption at offset " << (point + i) << "\n";
                        break;
                    }
                }
                if (!valid) break;
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
    netpipe::TcpEndpoint endpoint{"127.0.0.1", 7450};

    auto connect_res = client.connect(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "Client connected to 127.0.0.1:7450\n";

    // Create extra large message with pattern
    std::cout << "Allocating " << (data_size / 1024 / 1024) << "MB buffer...\n";
    netpipe::Message msg(data_size);
    std::cout << "Filling buffer with test pattern...\n";
    for (size_t i = 0; i < data_size; i++) {
        msg[i] = static_cast<uint8_t>(i % 256);
    }

    // Send extra large message
    std::cout << "Sending " << (data_size / 1024 / 1024) << "MB...\n";
    auto start = std::chrono::high_resolution_clock::now();
    auto send_res = client.send(msg);
    auto end = std::chrono::high_resolution_clock::now();

    if (send_res.is_ok()) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "Client sent " << (msg.size() / 1024 / 1024) << "MB in " << duration << "ms (" << throughput
                  << " MB/s)\n";
    }

    // Receive echo
    std::cout << "Receiving echo...\n";
    start = std::chrono::high_resolution_clock::now();
    auto recv_res = client.recv();
    end = std::chrono::high_resolution_clock::now();

    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (echo.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "Client received echo: " << (echo.size() / 1024 / 1024) << "MB in " << duration << "ms ("
                  << throughput << " MB/s)\n";
    }

    client.close();
    server_thread.join();
    std::cout << "TCP extra large transfer test completed\n";
}

void test_ipc_xlarge() {
    std::cout << "\n=== Testing IPC Extra Large Transfer (100MB) ===\n";

    const size_t data_size = 100 * 1024 * 1024; // 100MB

    // Server thread
    std::thread server_thread([data_size]() {
        netpipe::IpcStream server;
        netpipe::IpcEndpoint endpoint{"/tmp/flatsim_xlarge.sock"};

        auto listen_res = server.listen_ipc(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "IPC Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "IPC Server listening on /tmp/flatsim_xlarge.sock\n";

        auto client_res = server.accept();
        if (client_res.is_err()) {
            std::cerr << "IPC Server accept failed: " << client_res.error().message.c_str() << "\n";
            return;
        }
        auto client = std::move(client_res.value());
        std::cout << "IPC Server accepted connection\n";

        // Receive extra large message
        std::cout << "IPC Server receiving data...\n";
        auto start = std::chrono::high_resolution_clock::now();
        auto recv_res = client->recv();
        auto end = std::chrono::high_resolution_clock::now();

        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);

            std::cout << "IPC Server received " << (msg.size() / 1024 / 1024) << "MB in " << duration << "ms ("
                      << throughput << " MB/s)\n";

            // Verify data integrity at sample points
            bool valid = true;
            for (size_t i = 0; i < 10000; i += 1000) {
                if (msg[i] != static_cast<uint8_t>(i % 256)) {
                    valid = false;
                    break;
                }
            }
            std::cout << "Data integrity: " << (valid ? "OK" : "FAILED") << "\n";

            // Echo back
            std::cout << "IPC Server echoing data...\n";
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
    netpipe::IpcEndpoint endpoint{"/tmp/flatsim_xlarge.sock"};

    auto connect_res = client.connect_ipc(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "IPC Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "IPC Client connected to /tmp/flatsim_xlarge.sock\n";

    // Create extra large message with pattern
    std::cout << "Allocating " << (data_size / 1024 / 1024) << "MB buffer...\n";
    netpipe::Message msg(data_size);
    std::cout << "Filling buffer with test pattern...\n";
    for (size_t i = 0; i < data_size; i++) {
        msg[i] = static_cast<uint8_t>(i % 256);
    }

    // Send extra large message
    std::cout << "Sending " << (data_size / 1024 / 1024) << "MB...\n";
    auto start = std::chrono::high_resolution_clock::now();
    auto send_res = client.send(msg);
    auto end = std::chrono::high_resolution_clock::now();

    if (send_res.is_ok()) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "IPC Client sent " << (msg.size() / 1024 / 1024) << "MB in " << duration << "ms (" << throughput
                  << " MB/s)\n";
    }

    // Receive echo
    std::cout << "Receiving echo...\n";
    start = std::chrono::high_resolution_clock::now();
    auto recv_res = client.recv();
    end = std::chrono::high_resolution_clock::now();

    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (echo.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "IPC Client received echo: " << (echo.size() / 1024 / 1024) << "MB in " << duration << "ms ("
                  << throughput << " MB/s)\n";
    }

    client.close();
    server_thread.join();
    std::cout << "IPC extra large transfer test completed\n";
}

void test_shm_xlarge() {
    std::cout << "\n=== Testing SHM Extra Large Transfer (50MB) ===\n";

    const size_t data_size = 50 * 1024 * 1024; // 50MB
    const size_t shm_size = 64 * 1024 * 1024;  // 64MB buffer

    // Server thread
    std::thread server_thread([data_size, shm_size]() {
        netpipe::ShmStream server;
        netpipe::ShmEndpoint endpoint{"flatsim_xlarge_shm", shm_size};

        auto listen_res = server.listen_shm(endpoint);
        if (listen_res.is_err()) {
            std::cerr << "SHM Server listen failed: " << listen_res.error().message.c_str() << "\n";
            return;
        }
        std::cout << "SHM Server created ring buffer: flatsim_xlarge_shm (" << (shm_size / 1024 / 1024) << "MB)\n";

        // Wait for client to connect and send
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Receive extra large message
        std::cout << "SHM Server receiving data...\n";
        auto start = std::chrono::high_resolution_clock::now();
        auto recv_res = server.recv();
        auto end = std::chrono::high_resolution_clock::now();

        if (recv_res.is_ok()) {
            auto msg = recv_res.value();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);

            std::cout << "SHM Server received " << (msg.size() / 1024 / 1024) << "MB in " << duration << "ms ("
                      << throughput << " MB/s)\n";

            // Echo back
            std::cout << "SHM Server echoing data...\n";
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
    netpipe::ShmEndpoint endpoint{"flatsim_xlarge_shm", shm_size};

    auto connect_res = client.connect_shm(endpoint);
    if (connect_res.is_err()) {
        std::cerr << "SHM Client connect failed: " << connect_res.error().message.c_str() << "\n";
        server_thread.join();
        return;
    }
    std::cout << "SHM Client connected to flatsim_xlarge_shm\n";

    // Create extra large message with pattern
    std::cout << "Allocating " << (data_size / 1024 / 1024) << "MB buffer...\n";
    netpipe::Message msg(data_size);
    std::cout << "Filling buffer with test pattern...\n";
    for (size_t i = 0; i < data_size; i++) {
        msg[i] = static_cast<uint8_t>(i % 256);
    }

    // Send extra large message
    std::cout << "Sending " << (data_size / 1024 / 1024) << "MB via shared memory...\n";
    auto start = std::chrono::high_resolution_clock::now();
    auto send_res = client.send(msg);
    auto end = std::chrono::high_resolution_clock::now();

    if (send_res.is_ok()) {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (msg.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "SHM Client sent " << (msg.size() / 1024 / 1024) << "MB in " << duration << "ms (" << throughput
                  << " MB/s)\n";
    }

    // Receive echo
    std::cout << "Receiving echo...\n";
    start = std::chrono::high_resolution_clock::now();
    auto recv_res = client.recv();
    end = std::chrono::high_resolution_clock::now();

    if (recv_res.is_ok()) {
        auto echo = recv_res.value();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        double throughput = (echo.size() / 1024.0 / 1024.0) / (duration / 1000.0);
        std::cout << "SHM Client received echo: " << (echo.size() / 1024 / 1024) << "MB in " << duration << "ms ("
                  << throughput << " MB/s)\n";
    }

    client.close();
    server_thread.join();
    std::cout << "SHM extra large transfer test completed\n";
}

int main() {
    std::cout << "Testing Netpipe Extra Large Data Transfers\n";
    std::cout << "===========================================\n";
    std::cout << "WARNING: This test allocates large buffers and may take time\n\n";

    test_tcp_xlarge();
    test_ipc_xlarge();
    test_shm_xlarge();

    std::cout << "\n✓ All extra large transfer tests completed\n";
    return 0;
}
