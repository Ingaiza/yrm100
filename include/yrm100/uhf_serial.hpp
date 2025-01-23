#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <functional>
#include <thread>
#include "/home/aimbot/yrm100/src/yrm100/include/yrm100/uhf_buffer.hpp"

class UHFSerial {
public:
    static constexpr size_t RX_BUFFER_SIZE = 250;
    static constexpr uint32_t DEFAULT_BAUDRATE = 115200;
    static constexpr uint8_t FRAME_START = 0xBB;
    static constexpr uint8_t FRAME_END = 0x7E;
    static constexpr int WAIT_TICK = 1000;

    using receive_callback_t = std::function<void(uint8_t*, void*)>;

    UHFSerial(const std::string& port_name);
    ~UHFSerial();

    // Non-copyable
    UHFSerial(const UHFSerial&) = delete;
    UHFSerial& operator=(const UHFSerial&) = delete;

    void send(const uint8_t* data, size_t size);
    size_t send_wait(const uint8_t* data, size_t size, uint8_t* response_framed_);
    void set_receive_callback(receive_callback_t callback, void* ctx);
    void send_wait_multi(Buffer** buffer_return_,const uint8_t* data, size_t size);
    void set_baudrate(uint32_t baudrate);
    bool tick();
    void tick_reset();
    void start();
    void stop();
    void pop_from_buffer(uint8_t* buffer, size_t& buffer_size);
    Buffer* buffer_;
    M_Buffer multi_buffer_[5];
    uint32_t baudrate_;


private:
    void rx_worker();
    void start_receive();
    void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);

    boost::asio::io_context io_context_;
    std::shared_ptr<boost::asio::serial_port> serial_port_;
    std::unique_ptr<std::thread> worker_thread_;
    receive_callback_t callback_;
    void* callback_ctx_;
    std::atomic<int> tick_;
    uint8_t read_buffer_[1];
    bool running_;
};