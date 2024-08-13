#ifndef BASE_BOARD_HANDLER_H
#define BASE_BOARD_HANDLER_H

#include <string>
#include <thread>
#include <atomic>
#include "CircularBuffer.h"

class BaseBoardHandler {
public:
    BaseBoardHandler();
    BaseBoardHandler(const std::string& port, uint16_t start_seq, double publish_hz);
    ~BaseBoardHandler();
    void start();
    void stop();
    void sendPacket(uint32_t data1, uint32_t data2);
    uint32_t getActualAccel() {return actual_accel;};
    uint32_t getActualSteer() {return actual_steer;};

private:
    int fd;
    std::string serial_port;
    uint16_t start_seq;
    double publish_hz;
    std::atomic<bool> stop_flag;
    CircularBuffer rx_buffer;
    std::thread send_thread;
    std::thread receive_thread;
    uint32_t counter;

    uint32_t actual_accel;
    uint32_t actual_steer;

    void process_received_data();
    void send_loop();
    void receive_loop();
};

#endif // BASE_BOARD_HANDLER_H
