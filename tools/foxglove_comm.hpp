//
// Created by Interweave on 2026/4/14.
//


#ifndef TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
#define TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP

#pragma once

#include <memory>
#include <string>

namespace tools {

    class FoxGloveComm {
    public:
        FoxGloveComm(const std::string& host = "0.0.0.0", uint16_t port = 8765);
        ~FoxGloveComm();

        FoxGloveComm(const FoxGloveComm&) = delete;
        FoxGloveComm& operator=(const FoxGloveComm&) = delete;

        FoxGloveComm(FoxGloveComm&&) noexcept;
        FoxGloveComm& operator=(FoxGloveComm&&) noexcept;

        // 是否初始化成功
        bool ok() const;

        // 当前绑定参数
        const std::string& host() const;
        uint16_t port() const;

        // 重新创建 server
        bool reset(const std::string& host, uint16_t port);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}  // namespace tools

#endif //TGU_ROBOCORE_2027_FOXGLOVE_COMM_HPP
