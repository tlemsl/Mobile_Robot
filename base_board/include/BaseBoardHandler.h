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
    void sendPacket(int accel, int steer);
    int getActualAccel() {return actual_accel;};
    int getActualSteer() {return actual_steer;};

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

    int actual_accel;
    int actual_steer;

    void process_received_data();
    void send_loop();
    void receive_loop();
};

#endif // BASE_BOARD_HANDLER_H
