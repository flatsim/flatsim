#pragma once

#include <memory>
#include <netpipe/netpipe.hpp>
#include <string>
#include <vector>

namespace flatsim {

    // Unified endpoint type for all transports
    struct Endpoint {
        enum class Type { TCP, IPC, SHM };

        Type type;
        std::string host;    // TCP: hostname/IP, IPC: socket path, SHM: shared memory name
        uint16_t port;       // TCP: port number, SHM: buffer size (in KB)
        size_t shm_size = 0; // SHM only: buffer size in bytes

        // TCP endpoint
        static Endpoint tcp(const std::string &host, uint16_t port) { return {Type::TCP, host, port, 0}; }

        // IPC endpoint (Unix domain socket)
        static Endpoint ipc(const std::string &path) { return {Type::IPC, path, 0, 0}; }

        // SHM endpoint (shared memory)
        static Endpoint shm(const std::string &name, size_t size_bytes) { return {Type::SHM, name, 0, size_bytes}; }

        std::string to_string() const {
            switch (type) {
            case Type::TCP:
                return "tcp://" + host + ":" + std::to_string(port);
            case Type::IPC:
                return "ipc://" + host;
            case Type::SHM:
                return "shm://" + host + " (" + std::to_string(shm_size) + " bytes)";
            }
            return "unknown";
        }
    };

    // Abstract transport interface
    class Transport {
      public:
        virtual ~Transport() = default;

        // Client side: connect to remote endpoint
        virtual bool connect(const Endpoint &endpoint) = 0;

        // Server side: bind and listen
        virtual bool listen(const Endpoint &endpoint) = 0;

        // Server side: accept incoming connection (returns new Transport for client)
        virtual std::unique_ptr<Transport> accept() = 0;

        // Send message
        virtual bool send(const std::vector<uint8_t> &data) = 0;

        // Receive message (blocking)
        virtual std::vector<uint8_t> recv() = 0;

        // Receive message with timeout (0 = non-blocking)
        virtual std::vector<uint8_t> recv(int timeout_ms) = 0;

        // Close connection
        virtual void close() = 0;

        // Check if connected
        virtual bool is_connected() const = 0;

        // Get last error message
        virtual std::string last_error() const = 0;
    };

    // TCP transport implementation
    class TcpTransport : public Transport {
      private:
        netpipe::TcpStream stream_;
        std::string last_error_;

      public:
        TcpTransport() = default;
        explicit TcpTransport(netpipe::TcpStream &&stream) : stream_(std::move(stream)) {}

        bool connect(const Endpoint &endpoint) override {
            if (endpoint.type != Endpoint::Type::TCP) {
                last_error_ = "Invalid endpoint type for TCP transport";
                return false;
            }

            netpipe::TcpEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.port};
            auto res = stream_.connect(ep);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        bool listen(const Endpoint &endpoint) override {
            if (endpoint.type != Endpoint::Type::TCP) {
                last_error_ = "Invalid endpoint type for TCP transport";
                return false;
            }

            netpipe::TcpEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.port};
            auto res = stream_.listen(ep);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        std::unique_ptr<Transport> accept() override {
            auto res = stream_.accept();
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return nullptr;
            }

            // Downcast unique_ptr<Stream> to TcpStream
            auto tcp_stream = dynamic_cast<netpipe::TcpStream *>(res.value().get());
            if (!tcp_stream) {
                last_error_ = "Failed to cast accepted stream to TcpStream";
                return nullptr;
            }

            // Move the TcpStream out (release ownership from unique_ptr<Stream>)
            netpipe::TcpStream moved_stream = std::move(*tcp_stream);
            res.value().release();

            return std::make_unique<TcpTransport>(std::move(moved_stream));
        }

        bool send(const std::vector<uint8_t> &data) override {
            netpipe::Message msg(data.begin(), data.end());
            auto res = stream_.send(msg);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        std::vector<uint8_t> recv() override {
            auto res = stream_.recv();
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return {};
            }
            return std::vector<uint8_t>(res.value().begin(), res.value().end());
        }

        std::vector<uint8_t> recv(int timeout_ms) override {
            if (timeout_ms > 0) {
                stream_.set_recv_timeout(static_cast<uint32_t>(timeout_ms));
            }
            return recv();
        }

        void close() override { stream_.close(); }

        bool is_connected() const override { return stream_.is_connected(); }

        std::string last_error() const override { return last_error_; }
    };

    // IPC transport implementation
    class IpcTransport : public Transport {
      private:
        netpipe::IpcStream stream_;
        std::string last_error_;

      public:
        IpcTransport() = default;
        explicit IpcTransport(netpipe::IpcStream &&stream) : stream_(std::move(stream)) {}

        bool connect(const Endpoint &endpoint) override {
            if (endpoint.type != Endpoint::Type::IPC) {
                last_error_ = "Invalid endpoint type for IPC transport";
                return false;
            }

            netpipe::IpcEndpoint ep{dp::String(endpoint.host.c_str())};
            auto res = stream_.connect_ipc(ep);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        bool listen(const Endpoint &endpoint) override {
            if (endpoint.type != Endpoint::Type::IPC) {
                last_error_ = "Invalid endpoint type for IPC transport";
                return false;
            }

            netpipe::IpcEndpoint ep{dp::String(endpoint.host.c_str())};
            auto res = stream_.listen_ipc(ep);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        std::unique_ptr<Transport> accept() override {
            auto res = stream_.accept();
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return nullptr;
            }

            auto ipc_stream = dynamic_cast<netpipe::IpcStream *>(res.value().get());
            if (!ipc_stream) {
                last_error_ = "Failed to cast accepted stream to IpcStream";
                return nullptr;
            }

            netpipe::IpcStream moved_stream = std::move(*ipc_stream);
            res.value().release();

            return std::make_unique<IpcTransport>(std::move(moved_stream));
        }

        bool send(const std::vector<uint8_t> &data) override {
            netpipe::Message msg(data.begin(), data.end());
            auto res = stream_.send(msg);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        std::vector<uint8_t> recv() override {
            auto res = stream_.recv();
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return {};
            }
            return std::vector<uint8_t>(res.value().begin(), res.value().end());
        }

        std::vector<uint8_t> recv(int timeout_ms) override {
            if (timeout_ms > 0) {
                stream_.set_recv_timeout(static_cast<uint32_t>(timeout_ms));
            }
            return recv();
        }

        void close() override { stream_.close(); }

        bool is_connected() const override { return stream_.is_connected(); }

        std::string last_error() const override { return last_error_; }
    };

    // SHM transport implementation
    class ShmTransport : public Transport {
      private:
        netpipe::ShmStream stream_;
        std::string last_error_;

      public:
        ShmTransport() = default;
        ShmTransport(const ShmTransport &) = delete;
        ShmTransport &operator=(const ShmTransport &) = delete;
        ShmTransport(ShmTransport &&) = delete;
        ShmTransport &operator=(ShmTransport &&) = delete;

        bool connect(const Endpoint &endpoint) override {
            if (endpoint.type != Endpoint::Type::SHM) {
                last_error_ = "Invalid endpoint type for SHM transport";
                return false;
            }

            netpipe::ShmEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.shm_size};
            auto res = stream_.connect_shm(ep);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        bool listen(const Endpoint &endpoint) override {
            if (endpoint.type != Endpoint::Type::SHM) {
                last_error_ = "Invalid endpoint type for SHM transport";
                return false;
            }

            netpipe::ShmEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.shm_size};
            auto res = stream_.listen_shm(ep);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        std::unique_ptr<Transport> accept() override {
            // SHM doesn't support accept (1:1 connection)
            last_error_ = "SHM transport does not support accept()";
            return nullptr;
        }

        bool send(const std::vector<uint8_t> &data) override {
            netpipe::Message msg(data.begin(), data.end());
            auto res = stream_.send(msg);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return false;
            }
            return true;
        }

        std::vector<uint8_t> recv() override {
            auto res = stream_.recv();
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return {};
            }
            return std::vector<uint8_t>(res.value().begin(), res.value().end());
        }

        std::vector<uint8_t> recv(int timeout_ms) override {
            if (timeout_ms > 0) {
                stream_.set_recv_timeout(static_cast<uint32_t>(timeout_ms));
            }
            return recv();
        }

        void close() override { stream_.close(); }

        bool is_connected() const override { return stream_.is_connected(); }

        std::string last_error() const override { return last_error_; }
    };

    // Factory function to create appropriate transport based on endpoint type
    inline std::unique_ptr<Transport> create_transport(Endpoint::Type type) {
        switch (type) {
        case Endpoint::Type::TCP:
            return std::make_unique<TcpTransport>();
        case Endpoint::Type::IPC:
            return std::make_unique<IpcTransport>();
        case Endpoint::Type::SHM:
            return std::make_unique<ShmTransport>();
        }
        return nullptr;
    }

    // ============================================================================
    // RPC Layer - Remote<Unidirect> wrapper for spawn/despawn/heartbeat
    // ============================================================================

    // RPC method IDs
    enum class RpcMethod : uint32_t {
        // Agent → Simulator
        SPAWN = 1,
        DESPAWN = 2,
        HEARTBEAT = 3,
        CONTROL = 4,
        LIDAR_CFG = 5,

        // Simulator → Agent
        STATE = 10,
        SENSORS = 11,
        TELEPORT = 12,
    };

    // RPC client - calls remote methods
    class RpcClient {
      private:
        std::unique_ptr<netpipe::TcpStream> tcp_stream_; // Owned stream for TCP
        std::unique_ptr<netpipe::IpcStream> ipc_stream_; // Owned stream for IPC
        std::unique_ptr<netpipe::ShmStream> shm_stream_; // Owned stream for SHM
        std::unique_ptr<netpipe::Remote<netpipe::Unidirect>> router_;

      public:
        RpcClient() = default;

        // Connect to server
        bool connect(const Endpoint &endpoint) {
            // Create appropriate stream based on endpoint type
            switch (endpoint.type) {
            case Endpoint::Type::TCP: {
                tcp_stream_ = std::make_unique<netpipe::TcpStream>();
                netpipe::TcpEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.port};
                auto res = tcp_stream_->connect(ep);
                if (res.is_err()) {
                    return false;
                }
                router_ = std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*tcp_stream_);
                return true;
            }
            case Endpoint::Type::IPC: {
                ipc_stream_ = std::make_unique<netpipe::IpcStream>();
                netpipe::IpcEndpoint ep{dp::String(endpoint.host.c_str())};
                auto res = ipc_stream_->connect_ipc(ep);
                if (res.is_err()) {
                    return false;
                }
                router_ = std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*ipc_stream_);
                return true;
            }
            case Endpoint::Type::SHM: {
                shm_stream_ = std::make_unique<netpipe::ShmStream>();
                netpipe::ShmEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.shm_size};
                auto res = shm_stream_->connect_shm(ep);
                if (res.is_err()) {
                    return false;
                }
                router_ = std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*shm_stream_);
                return true;
            }
            }
            return false;
        }

        // Call a remote method
        std::vector<uint8_t> call(RpcMethod method, const std::vector<uint8_t> &request, uint32_t timeout_ms = 5000) {
            if (!router_) {
                return {};
            }
            netpipe::Message req(request.begin(), request.end());
            auto res = router_->call(static_cast<uint32_t>(method), req, timeout_ms);
            if (res.is_err()) {
                return {};
            }
            return std::vector<uint8_t>(res.value().begin(), res.value().end());
        }

        // Close connection
        void close() {
            if (tcp_stream_) tcp_stream_->close();
            if (ipc_stream_) ipc_stream_->close();
            if (shm_stream_) shm_stream_->close();
        }
    };

    // RPC server - handles incoming requests
    class RpcServer {
      private:
        std::unique_ptr<netpipe::TcpStream> tcp_stream_; // Owned stream for TCP
        std::unique_ptr<netpipe::IpcStream> ipc_stream_; // Owned stream for IPC
        std::unique_ptr<netpipe::ShmStream> shm_stream_; // Owned stream for SHM
        std::unique_ptr<netpipe::Remote<netpipe::Unidirect>> router_;

      public:
        RpcServer() = default;

        // Listen on endpoint
        bool listen(const Endpoint &endpoint) {
            switch (endpoint.type) {
            case Endpoint::Type::TCP: {
                tcp_stream_ = std::make_unique<netpipe::TcpStream>();
                netpipe::TcpEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.port};
                auto res = tcp_stream_->listen(ep);
                if (res.is_err()) {
                    return false;
                }
                router_ = std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*tcp_stream_);
                return true;
            }
            case Endpoint::Type::IPC: {
                ipc_stream_ = std::make_unique<netpipe::IpcStream>();
                netpipe::IpcEndpoint ep{dp::String(endpoint.host.c_str())};
                auto res = ipc_stream_->listen_ipc(ep);
                if (res.is_err()) {
                    return false;
                }
                router_ = std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*ipc_stream_);
                return true;
            }
            case Endpoint::Type::SHM: {
                shm_stream_ = std::make_unique<netpipe::ShmStream>();
                netpipe::ShmEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.shm_size};
                auto res = shm_stream_->listen_shm(ep);
                if (res.is_err()) {
                    return false;
                }
                router_ = std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*shm_stream_);
                return true;
            }
            }
            return false;
        }

        // Accept a client connection (returns new RpcServer for the client)
        std::unique_ptr<RpcServer> accept() {
            if (!tcp_stream_ && !ipc_stream_) {
                return nullptr; // SHM doesn't support accept
            }

            netpipe::Stream *listen_stream = tcp_stream_ ? static_cast<netpipe::Stream *>(tcp_stream_.get())
                                                         : static_cast<netpipe::Stream *>(ipc_stream_.get());

            auto res = listen_stream->accept();
            if (res.is_err()) {
                return nullptr;
            }

            auto client_server = std::make_unique<RpcServer>();

            // Determine which type of stream was accepted
            if (tcp_stream_) {
                auto *tcp = dynamic_cast<netpipe::TcpStream *>(res.value().get());
                if (tcp) {
                    client_server->tcp_stream_ = std::unique_ptr<netpipe::TcpStream>(tcp);
                    res.value().release();
                    client_server->router_ =
                        std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*client_server->tcp_stream_);
                }
            } else if (ipc_stream_) {
                auto *ipc = dynamic_cast<netpipe::IpcStream *>(res.value().get());
                if (ipc) {
                    client_server->ipc_stream_ = std::unique_ptr<netpipe::IpcStream>(ipc);
                    res.value().release();
                    client_server->router_ =
                        std::make_unique<netpipe::Remote<netpipe::Unidirect>>(*client_server->ipc_stream_);
                }
            }

            return client_server;
        }

        // Register a handler for a method
        bool register_method(RpcMethod method,
                             std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> handler) {
            if (!router_) {
                return false;
            }

            // Wrap the handler to convert between std::vector and netpipe::Message
            auto wrapped_handler = [handler](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                std::vector<uint8_t> request(req.begin(), req.end());
                auto response = handler(request);
                if (response.empty()) {
                    return dp::result::err(dp::Error::io_error("handler returned empty response"));
                }
                netpipe::Message resp(response.begin(), response.end());
                return dp::result::ok(std::move(resp));
            };

            auto res = router_->register_method(static_cast<uint32_t>(method), wrapped_handler);
            return res.is_ok();
        }

        // Serve requests (blocking loop)
        bool serve() {
            if (!router_) {
                return false;
            }
            auto res = router_->serve();
            return res.is_ok();
        }

        // Close connection
        void close() {
            if (tcp_stream_) tcp_stream_->close();
            if (ipc_stream_) ipc_stream_->close();
            if (shm_stream_) shm_stream_->close();
        }
    };

    // ============================================================================
    // RpcPeer - Bidirectional RPC using Remote<Bidirect> (single channel)
    // ============================================================================

    // Handler type for RPC methods
    using RpcHandler = std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>;

    // RpcPeer - wraps netpipe::Remote<Bidirect> for bidirectional communication
    class RpcPeer {
      private:
        std::unique_ptr<netpipe::TcpStream> tcp_stream_;
        std::unique_ptr<netpipe::IpcStream> ipc_stream_;
        std::unique_ptr<netpipe::ShmStream> shm_stream_;
        std::unique_ptr<netpipe::Remote<netpipe::Bidirect>> peer_;
        Endpoint::Type type_;
        std::string last_error_;

      public:
        RpcPeer() = default;

        // Connect to remote peer (client side)
        bool connect(const Endpoint &endpoint, size_t max_concurrent = 100, bool enable_metrics = true) {
            type_ = endpoint.type;

            switch (endpoint.type) {
            case Endpoint::Type::TCP: {
                tcp_stream_ = std::make_unique<netpipe::TcpStream>();
                netpipe::TcpEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.port};
                auto res = tcp_stream_->connect(ep);
                if (res.is_err()) {
                    last_error_ = std::string(res.error().message.c_str());
                    return false;
                }
                peer_ =
                    std::make_unique<netpipe::Remote<netpipe::Bidirect>>(*tcp_stream_, max_concurrent, enable_metrics);
                return true;
            }
            case Endpoint::Type::IPC: {
                ipc_stream_ = std::make_unique<netpipe::IpcStream>();
                netpipe::IpcEndpoint ep{dp::String(endpoint.host.c_str())};
                auto res = ipc_stream_->connect_ipc(ep);
                if (res.is_err()) {
                    last_error_ = std::string(res.error().message.c_str());
                    return false;
                }
                peer_ =
                    std::make_unique<netpipe::Remote<netpipe::Bidirect>>(*ipc_stream_, max_concurrent, enable_metrics);
                return true;
            }
            case Endpoint::Type::SHM: {
                shm_stream_ = std::make_unique<netpipe::ShmStream>();
                netpipe::ShmEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.shm_size};
                auto res = shm_stream_->connect_shm(ep);
                if (res.is_err()) {
                    last_error_ = std::string(res.error().message.c_str());
                    return false;
                }
                peer_ =
                    std::make_unique<netpipe::Remote<netpipe::Bidirect>>(*shm_stream_, max_concurrent, enable_metrics);
                return true;
            }
            }
            return false;
        }

        // Listen for incoming connection (server side)
        bool listen(const Endpoint &endpoint, size_t max_concurrent = 100, bool enable_metrics = true) {
            type_ = endpoint.type;

            switch (endpoint.type) {
            case Endpoint::Type::TCP: {
                tcp_stream_ = std::make_unique<netpipe::TcpStream>();
                netpipe::TcpEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.port};
                auto res = tcp_stream_->listen(ep);
                if (res.is_err()) {
                    last_error_ = std::string(res.error().message.c_str());
                    return false;
                }
                return true;
            }
            case Endpoint::Type::IPC: {
                ipc_stream_ = std::make_unique<netpipe::IpcStream>();
                netpipe::IpcEndpoint ep{dp::String(endpoint.host.c_str())};
                auto res = ipc_stream_->listen_ipc(ep);
                if (res.is_err()) {
                    last_error_ = std::string(res.error().message.c_str());
                    return false;
                }
                return true;
            }
            case Endpoint::Type::SHM: {
                shm_stream_ = std::make_unique<netpipe::ShmStream>();
                netpipe::ShmEndpoint ep{dp::String(endpoint.host.c_str()), endpoint.shm_size};
                auto res = shm_stream_->listen_shm(ep);
                if (res.is_err()) {
                    last_error_ = std::string(res.error().message.c_str());
                    return false;
                }
                return true;
            }
            }
            return false;
        }

        // Accept incoming connection (returns new RpcPeer for client)
        std::unique_ptr<RpcPeer> accept(size_t max_concurrent = 100, bool enable_metrics = true) {
            if (!tcp_stream_ && !ipc_stream_) {
                last_error_ = "SHM does not support accept()";
                return nullptr;
            }

            netpipe::Stream *listen_stream = tcp_stream_ ? static_cast<netpipe::Stream *>(tcp_stream_.get())
                                                         : static_cast<netpipe::Stream *>(ipc_stream_.get());

            auto res = listen_stream->accept();
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return nullptr;
            }

            auto client_peer = std::make_unique<RpcPeer>();
            client_peer->type_ = type_;

            // Move accepted stream to client peer
            if (tcp_stream_) {
                auto *tcp = dynamic_cast<netpipe::TcpStream *>(res.value().get());
                if (tcp) {
                    client_peer->tcp_stream_ = std::unique_ptr<netpipe::TcpStream>(tcp);
                    res.value().release();
                    client_peer->peer_ = std::make_unique<netpipe::Remote<netpipe::Bidirect>>(
                        *client_peer->tcp_stream_, max_concurrent, enable_metrics);
                }
            } else if (ipc_stream_) {
                auto *ipc = dynamic_cast<netpipe::IpcStream *>(res.value().get());
                if (ipc) {
                    client_peer->ipc_stream_ = std::unique_ptr<netpipe::IpcStream>(ipc);
                    res.value().release();
                    client_peer->peer_ = std::make_unique<netpipe::Remote<netpipe::Bidirect>>(
                        *client_peer->ipc_stream_, max_concurrent, enable_metrics);
                }
            }

            return client_peer;
        }

        // Register a handler for incoming RPC calls
        bool register_method(RpcMethod method, RpcHandler handler) {
            if (!peer_) {
                last_error_ = "Peer not initialized";
                return false;
            }

            // Wrap handler to convert between std::vector and netpipe::Message
            auto wrapped_handler = [handler](const netpipe::Message &req) -> dp::Res<netpipe::Message> {
                std::vector<uint8_t> request(req.begin(), req.end());
                auto response = handler(request);
                netpipe::Message resp(response.begin(), response.end());
                return dp::result::ok(std::move(resp));
            };

            auto res = peer_->register_method(static_cast<uint32_t>(method), wrapped_handler);
            return res.is_ok();
        }

        // Call a remote method (blocks until response or timeout)
        std::vector<uint8_t> call(RpcMethod method, const std::vector<uint8_t> &request, uint32_t timeout_ms = 5000) {
            if (!peer_) {
                last_error_ = "Peer not initialized";
                return {};
            }

            netpipe::Message req(request.begin(), request.end());
            auto res = peer_->call(static_cast<uint32_t>(method), req, timeout_ms);
            if (res.is_err()) {
                last_error_ = std::string(res.error().message.c_str());
                return {};
            }
            return std::vector<uint8_t>(res.value().begin(), res.value().end());
        }

        // Get number of pending outgoing requests
        size_t pending_count() const { return peer_ ? peer_->pending_count() : 0; }

        // Get number of registered methods
        size_t method_count() const { return peer_ ? peer_->method_count() : 0; }

        // Check if metrics are enabled
        bool metrics_enabled() const { return peer_ ? peer_->metrics_enabled() : false; }

        // Get client metrics (outgoing calls)
        const netpipe::remote::RemoteMetrics *get_client_metrics() const {
            return peer_ ? &peer_->get_client_metrics() : nullptr;
        }

        // Get server metrics (incoming requests)
        const netpipe::remote::RemoteMetrics *get_server_metrics() const {
            return peer_ ? &peer_->get_server_metrics() : nullptr;
        }

        // Reset metrics
        void reset_metrics() {
            if (peer_) {
                peer_->reset_metrics();
            }
        }

        // Close connection
        void close() {
            peer_.reset();
            if (tcp_stream_) tcp_stream_->close();
            if (ipc_stream_) ipc_stream_->close();
            if (shm_stream_) shm_stream_->close();
        }

        // Get last error message
        std::string last_error() const { return last_error_; }

        // Check if peer is initialized
        bool is_connected() const { return peer_ != nullptr; }
    };

} // namespace flatsim
