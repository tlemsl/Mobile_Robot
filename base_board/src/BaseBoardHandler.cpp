#include "BaseBoardHandler.h"

// C system headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// C++ system headers
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

// Other libraries' headers
#include <yaml-cpp/yaml.h>

namespace {
constexpr int kBufferSize = 1024;
constexpr int kRxPacketSize = 25;
constexpr int kTxPacketSize = 13;
constexpr int kDefaultMotorCmd = 1500;
constexpr int kDefaultServoCmd = 1500;

uint8_t CalculateChecksum(const uint8_t* data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}
}  // namespace

BaseBoardHandler::BaseBoardHandler(const std::string& base_board_port,
                                   const uint16_t start_seq,
                                   const double publish_hz)
    : base_board_port_(base_board_port),
      start_seq_(start_seq),
      publish_hz_(publish_hz),
      receive_stop_flag_(false),
      send_stop_flag_(false),
      counter_(0),
      rx_buffer_(kBufferSize) {
  InitializeSerialPort();
  motor_cmd_ = kDefaultMotorCmd;
  servo_cmd_ = kDefaultServoCmd;
}

BaseBoardHandler::BaseBoardHandler(const std::string& transmitter_config_path,
                                   const std::string& base_board_port,
                                   const uint16_t start_seq,
                                   const double publish_hz)
    : base_board_port_(base_board_port),
      start_seq_(start_seq),
      publish_hz_(publish_hz),
      receive_stop_flag_(false),
      send_stop_flag_(false),
      counter_(0),
      rx_buffer_(kBufferSize) {
  InitializeSerialPort();
  LoadTransmitterConfig(transmitter_config_path);
  transmitter_throttle_ = transmitter_throttle_middle_;
  transmitter_steer_ = transmitter_steer_middle_;
  transmitter_aux_ = transmitter_aux_middle_;
  motor_cmd_ = transmitter_throttle_middle_;
  servo_cmd_ = transmitter_steer_middle_;
}

BaseBoardHandler::~BaseBoardHandler() {
  Stop();
  close(fd_);
}

void BaseBoardHandler::InitializeSerialPort() {
  fd_ = open(base_board_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    throw std::runtime_error("Error opening serial port");
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd_, &tty) != 0) {
    close(fd_);
    throw std::runtime_error("Error from tcgetattr");
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 10;

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close(fd_);
    throw std::runtime_error("Error from tcsetattr");
  }
}

int BaseBoardHandler::GetTransmitterThrottle() const {
  return static_cast<int>(transmitter_throttle_) -
         static_cast<int>(transmitter_throttle_middle_);
}

int BaseBoardHandler::GetTransmitterSteer() const {
  return static_cast<int>(transmitter_steer_) -
         static_cast<int>(transmitter_steer_middle_);
}

double BaseBoardHandler::GetTransmitterThrottleRatio() const {
  double diff = static_cast<double>(transmitter_throttle_) -
               static_cast<double>(transmitter_throttle_middle_);
  double range = static_cast<double>(transmitter_throttle_up_) -
                 static_cast<double>(transmitter_throttle_down_);
  return diff / range * 2;
}

int BaseBoardHandler::GetBaseBoardMotorCmd() const {
  return static_cast<int>(base_board_motor_cmd_) -
         static_cast<int>(transmitter_throttle_middle_);
}

int BaseBoardHandler::GetBaseBoardServoCmd() const {
  return static_cast<int>(base_board_servo_cmd_) -
         static_cast<int>(transmitter_steer_middle_);
}

uint32_t BaseBoardHandler::ToRawThrottle(int cmd) const {
  int sign = transmitter_throttle_up_ > transmitter_throttle_down_ ? 1 : -1;
  return static_cast<uint32_t>(cmd * sign + transmitter_throttle_middle_);
}

uint32_t BaseBoardHandler::ToRawSteer(int cmd) const {
  int sign = transmitter_steer_right_ > transmitter_steer_left_ ? 1 : -1;
  return static_cast<uint32_t>(cmd * sign + transmitter_steer_middle_);
}

void BaseBoardHandler::SetMotorCmd(uint32_t motor_cmd) {
  uint32_t max = transmitter_throttle_up_ > transmitter_throttle_down_
                     ? transmitter_throttle_up_
                     : transmitter_throttle_down_;
  uint32_t min = transmitter_throttle_up_ < transmitter_throttle_down_
                     ? transmitter_throttle_up_
                     : transmitter_throttle_down_;
  motor_cmd_ = std::clamp(motor_cmd, min, max);
}

void BaseBoardHandler::SetServoCmd(uint32_t servo_cmd) {
  uint32_t max = transmitter_steer_right_ > transmitter_steer_left_
                     ? transmitter_steer_right_
                     : transmitter_steer_left_;
  uint32_t min = transmitter_steer_right_ < transmitter_steer_left_
                     ? transmitter_steer_right_
                     : transmitter_steer_left_;
  servo_cmd_ = std::clamp(servo_cmd, min, max);
}

void BaseBoardHandler::Start() {
  StartReceiveThread();
  StartSendThread();
}

void BaseBoardHandler::StartReceiveThread() {
  receive_stop_flag_ = false;
  receive_thread_ = std::thread(&BaseBoardHandler::ReceiveLoop, this);
}

void BaseBoardHandler::StartSendThread() {
  send_stop_flag_ = false;
  send_thread_ = std::thread(&BaseBoardHandler::SendLoop, this);
}

void BaseBoardHandler::Stop() {
  receive_stop_flag_ = true;
  send_stop_flag_ = true;
  if (send_thread_.joinable()) send_thread_.join();
  if (receive_thread_.joinable()) receive_thread_.join();
}

void BaseBoardHandler::SendPacket(uint32_t motor_cmd, uint32_t servo_cmd) {
  uint8_t packet[kTxPacketSize];
  uint64_t combined_data = ((uint64_t)(motor_cmd) << 32) | (servo_cmd);

  packet[0] = (start_seq_ >> 8) & 0xFF;
  packet[1] = start_seq_ & 0xFF;
  packet[2] = 8;  // Length of data

  memcpy(&packet[3], &combined_data, sizeof(combined_data));

  packet[11] = CalculateChecksum(&packet[3], 8);
  packet[12] = 0xEF;  // End byte

  for (int i = 0; i < kTxPacketSize; i++) {
    ssize_t bytes_written = write(fd_, &packet[i], 1);
    if (bytes_written != 1) {
      throw std::runtime_error("Failed to write packet data");
    }
  }
}

void BaseBoardHandler::ProcessReceivedData() {
  while (rx_buffer_.size() >= kRxPacketSize) {
    if (rx_buffer_.peek(0) == (start_seq_ >> 8) &&
        rx_buffer_.peek(1) == (start_seq_ & 0xFF) && rx_buffer_.peek(2) == 8) {
      uint8_t data[20];
      for (int i = 0; i < 20; i++) {
        data[i] = rx_buffer_.peek(3 + i);
      }

      uint8_t checksum = rx_buffer_.peek(23);
      uint8_t end_byte = rx_buffer_.peek(24);

      if (checksum == CalculateChecksum(data, 20) && end_byte == 0xEF) {
        uint32_t received_data[5];
        memcpy(&received_data[0], data, sizeof(uint32_t));       // throttle
        memcpy(&received_data[1], data + 4, sizeof(uint32_t));   // steer
        memcpy(&received_data[2], data + 8, sizeof(uint32_t));   // aux
        memcpy(&received_data[3], data + 12, sizeof(uint32_t));  // motor_cmd
        memcpy(&received_data[4], data + 16, sizeof(uint32_t));  // servo_cmd

        transmitter_throttle_ = received_data[0];
        transmitter_steer_ = received_data[1];
        transmitter_aux_ = received_data[2];
        base_board_motor_cmd_ = received_data[3];
        base_board_servo_cmd_ = received_data[4];

        for (int i = 0; i < kRxPacketSize; i++) {
          rx_buffer_.get();
        }
      } else {
        rx_buffer_.get();
      }
    } else {
      rx_buffer_.get();
    }
  }
}

void BaseBoardHandler::SendLoop() {
  while (!send_stop_flag_) {
    SendPacket(motor_cmd_, servo_cmd_);
    usleep(static_cast<int>(1e6 / publish_hz_));
  }
}

void BaseBoardHandler::ReceiveLoop() {
  while (!receive_stop_flag_) {
    uint8_t byte;
    ssize_t n = read(fd_, &byte, 1);
    if (n > 0) {
      rx_buffer_.put(byte);
      ProcessReceivedData();
    } else if (n < 0) {
      std::cerr << "Error reading data" << std::endl;
    }
  }
}

AuxState BaseBoardHandler::GetTransmitterAux() const {
  const int kThreshold = 100;

  int aux_diff_middle = std::abs(static_cast<int>(transmitter_aux_) -
                                 static_cast<int>(transmitter_aux_middle_));
  int aux_diff_down = std::abs(static_cast<int>(transmitter_aux_) -
                               static_cast<int>(transmitter_aux_down_));
  int aux_diff_up = std::abs(static_cast<int>(transmitter_aux_) -
                             static_cast<int>(transmitter_aux_up_));

  if (aux_diff_middle < kThreshold) {
    return AuxState::kMiddle;
  } else if (aux_diff_down < kThreshold) {
    return AuxState::kDown;
  } else if (aux_diff_up < kThreshold) {
    return AuxState::kUp;
  }

  std::cerr << "Invalid aux state: " << transmitter_aux_ << std::endl;
  return AuxState::kMiddle;  // Default to middle as fallback
}

void BaseBoardHandler::LoadTransmitterConfig(const std::string& config_path) {
  try {
    YAML::Node config = YAML::LoadFile(config_path);

    // Load throttle calibration
    transmitter_throttle_up_ = config["Throttle"]["up"].as<uint32_t>();
    transmitter_throttle_middle_ = config["Throttle"]["idle"].as<uint32_t>();
    transmitter_throttle_down_ = config["Throttle"]["down"].as<uint32_t>();

    // Load steering calibration
    transmitter_steer_left_ = config["Steer"]["left"].as<uint32_t>();
    transmitter_steer_middle_ = config["Steer"]["idle"].as<uint32_t>();
    transmitter_steer_right_ = config["Steer"]["right"].as<uint32_t>();

    // Load aux calibration
    transmitter_aux_up_ = config["Aux"]["up"].as<uint32_t>();
    transmitter_aux_middle_ = config["Aux"]["idle"].as<uint32_t>();
    transmitter_aux_down_ = config["Aux"]["down"].as<uint32_t>();

    std::cout << "Transmitter config loaded successfully from: " << config_path
              << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error loading transmitter config from " << config_path << ": "
              << e.what() << std::endl;
    throw;
  }
}
