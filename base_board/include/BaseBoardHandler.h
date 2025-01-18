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
    uint32_t getTransimitterThrottle() {return transimitter_throttle;};
    uint32_t getTransimitterSteer() {return transimitter_steer;};
    uint32_t getTransimitterAux() {return transimitter_aux;};
    uint32_t getBaseBoardMotorCmd() {return base_board_motor_cmd;};
    uint32_t getBaseBoardServoCmd() {return base_board_servo_cmd;};

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

    uint32_t transimitter_throttle;
    uint32_t transimitter_steer;
    uint32_t transimitter_aux;
    uint32_t base_board_motor_cmd;
    uint32_t base_board_servo_cmd;

    void process_received_data();
    void send_loop();
    void receive_loop();
};

#endif // BASE_BOARD_HANDLER_H
