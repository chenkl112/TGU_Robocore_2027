//
// Created by Interweave on 2026/4/14.
//

#include "foxglove_comm.hpp"

#include <optional>
#include <utility>

#include <foxglove/server.hpp>

#include "tools/logger.hpp"

namespace tools {
    struct FoxGloveComm::Impl {
        std::string host;
        uint16_t port = 0;
        bool ready = false;

        std::optional<foxglove::WebSocketServer> server;

        LoggerConfig cfg{
            .level = LogLevel::Debug,
            .enable_console = true,
            .enable_file = false
        };

        static constexpr const char *MODULE = "FOXGLOVE_COMM";

        Impl(const std::string &h, uint16_t p) : host(h), port(p) {
        }

        bool create_server() {
            ready = false;
            server.reset();

            auto result = foxglove::WebSocketServer::create(
                foxglove::WebSocketServerOptions{
                    .host = host,
                    .port = port
                }
            );

            if (!result.has_value()) {
                LOG_ERROR(MODULE, "Failed to create WebSocket server at {}:{}", host, port);
                return false;
            }

            server.emplace(std::move(result.value()));
            ready = true;

            LOG_INFO(MODULE, "Foxglove server created at {}:{}", host, port);
            return true;
        }
    };

    FoxGloveComm::FoxGloveComm(const std::string &host, uint16_t port)
        : impl_(std::make_unique<Impl>(host, port)) {
        Logger::instance().init(impl_->cfg);
        impl_->create_server();
    }

    FoxGloveComm::~FoxGloveComm() = default;

    FoxGloveComm::FoxGloveComm(FoxGloveComm &&) noexcept = default;

    FoxGloveComm &FoxGloveComm::operator=(FoxGloveComm &&) noexcept = default;

    bool FoxGloveComm::ok() const {
        return impl_ && impl_->ready;
    }

    const std::string &FoxGloveComm::host() const {
        return impl_->host;
    }

    uint16_t FoxGloveComm::port() const {
        return impl_->port;
    }

    bool FoxGloveComm::reset(const std::string &host, uint16_t port) {
        if (!impl_) {
            return false;
        }

        impl_->host = host;
        impl_->port = port;
        return impl_->create_server();
    }
} // namespace tools
