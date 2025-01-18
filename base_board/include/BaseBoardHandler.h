#ifndef BASE_BOARD_HANDLER_H
#define BASE_BOARD_HANDLER_H

#include <atomic>
#include <string>
#include <thread>

#include "CircularBuffer.h"
enum class AuxState {
  UP,
  MIDDLE,
  DOWN,
};

class BaseBoardHandler {
 public:
  BaseBoardHandler();
    BaseBoardHandler(const std::string& base_board_port = "/dev/ttyACM0",
                   const uint16_t start_seq = 0xAA55,
                   const double publish_hz = 200.0);
  BaseBoardHandler(const std::string& transimitter_config_path,
                   const std::string& base_board_port = "/dev/ttyACM0",
                   const uint16_t start_seq = 0xAA55,
                   const double publish_hz = 200.0);
  ~BaseBoardHandler();
  void start();
  void stop();
  void sendPacket(uint32_t motor_cmd, uint32_t servo_cmd);
  uint32_t getTransimitterThrottleRaw() { return transimitter_throttle; };
  uint32_t getTransimitterSteerRaw() { return transimitter_steer; };
  uint32_t getTransimitterAuxRaw() { return transimitter_aux; };
  int getTransimitterThrottle() {
    return transimitter_throttle - transimitter_throttle_middle;
  };
  int getTransimitterSteer() {
    return transimitter_steer - transimitter_steer_middle;
  };
  int getBaseBoardMotorCmd() {
    return base_board_motor_cmd - transimitter_throttle_middle;
  };
  int getBaseBoardServoCmd() {
    return base_board_servo_cmd - transimitter_steer_middle;
  };
  AuxState getTransimitterAux();

  void setMotorCmd(int motor_cmd) { motor_cmd_ = motor_cmd; };
  void setServoCmd(int servo_cmd) { servo_cmd_ = servo_cmd; };

 private:
  int fd;
  std::string base_board_port;
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

  uint32_t transimitter_throttle_up;
  uint32_t transimitter_throttle_middle;
  uint32_t transimitter_throttle_down;

  uint32_t transimitter_steer_left;
  uint32_t transimitter_steer_middle;
  uint32_t transimitter_steer_right;

  uint32_t transimitter_aux_up;
  uint32_t transimitter_aux_middle;
  uint32_t transimitter_aux_down;

  int motor_cmd_;
  int servo_cmd_;

  void process_received_data();
  void send_loop();
  void receive_loop();
  void loadTransimitterConfig(const std::string& config_path);
};

#endif  // BASE_BOARD_HANDLER_H
