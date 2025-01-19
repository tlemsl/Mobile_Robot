#ifndef BASE_BOARD_INCLUDE_BASE_BOARD_HANDLER_H_
#define BASE_BOARD_INCLUDE_BASE_BOARD_HANDLER_H_

// C++ system headers
#include <atomic>
#include <string>
#include <thread>

// Project headers
#include "CircularBuffer.h"

enum class AuxState {
  kUp,
  kMiddle,
  kDown,
};

class BaseBoardHandler {
 public:
  BaseBoardHandler();
  BaseBoardHandler(const std::string& base_board_port = "/dev/ttyACM0",
                   const uint16_t start_seq = 0xAA55,
                   const double publish_hz = 200.0);
  BaseBoardHandler(const std::string& transmitter_config_path,
                   const std::string& base_board_port = "/dev/ttyACM0",
                   const uint16_t start_seq = 0xAA55,
                   const double publish_hz = 200.0);
  ~BaseBoardHandler();

  // Public methods
  void Start();
  void StartReceiveThread();
  void StartSendThread();
  void Stop();
  void SendPacket(uint32_t motor_cmd, uint32_t servo_cmd);
  uint32_t GetTransmitterThrottleRaw() const { return transmitter_throttle_; }
  uint32_t GetTransmitterSteerRaw() const { return transmitter_steer_; }
  uint32_t GetTransmitterAuxRaw() const { return transmitter_aux_; }
  int GetTransmitterThrottle() const;
  int GetTransmitterSteer() const;
  double GetTransmitterThrottleRatio() const;
  int GetBaseBoardMotorCmd() const;
  int GetBaseBoardServoCmd() const;
  AuxState GetTransmitterAux() const;
  uint32_t ToRawThrottle(int cmd) const;
  uint32_t ToRawSteer(int cmd) const;
  void SetMotorCmd(uint32_t motor_cmd);
  void SetServoCmd(uint32_t servo_cmd);
 private:
  // Private methods
  void ProcessReceivedData();
  void SendLoop();
  void ReceiveLoop();
  void LoadTransmitterConfig(const std::string& config_path);
  void InitializeSerialPort();

  // Configuration (order matches constructor initialization)
  std::string base_board_port_;
  uint16_t start_seq_;
  double publish_hz_;
  std::atomic<bool> receive_stop_flag_;
  std::atomic<bool> send_stop_flag_;
  uint32_t counter_;
  CircularBuffer rx_buffer_;

  // Threading
  std::thread send_thread_;
  std::thread receive_thread_;

  // File descriptor
  int fd_;

  // Transmitter state
  uint32_t transmitter_throttle_{0};
  uint32_t transmitter_steer_{0};
  uint32_t transmitter_aux_{0};
  uint32_t base_board_motor_cmd_{0};
  uint32_t base_board_servo_cmd_{0};

  // Calibration values
  uint32_t transmitter_throttle_up_{0};
  uint32_t transmitter_throttle_middle_{0};
  uint32_t transmitter_throttle_down_{0};
  uint32_t transmitter_steer_left_{0};
  uint32_t transmitter_steer_middle_{0};
  uint32_t transmitter_steer_right_{0};
  uint32_t transmitter_aux_up_{0};
  uint32_t transmitter_aux_middle_{0};
  uint32_t transmitter_aux_down_{0};

  // Command values
  int motor_cmd_{0};
  int servo_cmd_{0};
};

#endif  // BASE_BOARD_INCLUDE_BASE_BOARD_HANDLER_H_
