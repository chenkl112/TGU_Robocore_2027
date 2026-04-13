//
// Created by Interweave on 2026/4/10.
//

#include "serial.hpp"
#include <iostream>

namespace io {

    Serial::Serial()
        : serial_(io_), is_open_(false) {}

    Serial::~Serial() {
        close();
    }

    bool Serial::open(const std::string& device, int baudrate) {
        try {
            if (serial_.is_open()) {
                serial_.close();
            }

            serial_.open(device);

            serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));

            is_open_ = true;
            std::cout << "[Serial] open success\n";
            return true;

        } catch (std::exception& e) {
            std::cerr << "[Serial] open failed: " << e.what() << std::endl;
            is_open_ = false;
            return false;
        }
    }

    void Serial::close() {
        if (serial_.is_open()) {
            serial_.close();
        }
        is_open_ = false;
    }

    bool Serial::isOpen() const {
        return is_open_;
    }

    size_t Serial::write(const uint8_t* data, size_t size) {
        if (!is_open_) return 0;

        try {
            return boost::asio::write(serial_, boost::asio::buffer(data, size));
        } catch (...) {
            is_open_ = false;
            return 0;
        }
    }

    void Serial::spinOnce() {
        if (!is_open_) return;

        try {
            size_t len = serial_.read_some(boost::asio::buffer(rx_buf_));

            for (auto& cb : callbacks_) {
                cb(rx_buf_, len);
            }

        } catch (...) {
            is_open_ = false;
        }
    }

} // namespace io