#include "BaseBoardHandler.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

#define BUFFER_SIZE 1024
#define RX_PACKET_SIZE 25
#define TX_PACKET_SIZE 13
uint8_t calculate_checksum(uint8_t* data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

BaseBoardHandler::BaseBoardHandler(const std::string& base_board_port,
                                   const uint16_t start_seq,
                                   const double publish_hz)
    : base_board_port(base_board_port),
      start_seq(start_seq),
      publish_hz(publish_hz),
      stop_flag(false),
      counter(0),
      rx_buffer(BUFFER_SIZE) {
  fd = open(base_board_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    throw std::runtime_error("Error opening serial port");
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
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

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    throw std::runtime_error("Error from tcsetattr");
  }
  motor_cmd_ = 1500;
  servo_cmd_ = 1500;
}
BaseBoardHandler::BaseBoardHandler(const std::string& transimitter_config_path,
                                   const std::string& base_board_port,
                                   const uint16_t start_seq,
                                   const double publish_hz)
    : base_board_port(base_board_port),
      start_seq(start_seq),
      publish_hz(publish_hz),
      stop_flag(false),
      counter(0),
      rx_buffer(BUFFER_SIZE) {
  fd = open(base_board_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    throw std::runtime_error("Error opening serial port");
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
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

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    throw std::runtime_error("Error from tcsetattr");
  }
  loadTransimitterConfig(transimitter_config_path);
  transimitter_throttle = transimitter_throttle_middle;
  transimitter_steer = transimitter_steer_middle;
  transimitter_aux = transimitter_aux_middle;
  motor_cmd_ = transimitter_throttle_middle;
  servo_cmd_ = transimitter_steer_middle;
}

BaseBoardHandler::~BaseBoardHandler() {
  stop();
  close(fd);
}

void BaseBoardHandler::start() {
  stop_flag = false;
  // For debug
  send_thread = std::thread(&BaseBoardHandler::send_loop, this);
  receive_thread = std::thread(&BaseBoardHandler::receive_loop, this);
}

void BaseBoardHandler::stop() {
  stop_flag = true;
  if (send_thread.joinable()) send_thread.join();
  if (receive_thread.joinable()) receive_thread.join();
}

void BaseBoardHandler::sendPacket(uint32_t motor_cmd, uint32_t servo_cmd) {
  uint8_t packet[TX_PACKET_SIZE];
  uint64_t combined_data = ((uint64_t)(motor_cmd) << 32) | (servo_cmd);

  packet[0] = (start_seq >> 8) & 0xFF;
  packet[1] = start_seq & 0xFF;
  packet[2] = 8;  // Length of data

  memcpy(&packet[3], &combined_data, sizeof(combined_data));

  packet[11] = calculate_checksum(&packet[3], 8);
  packet[12] = 0xEF;  // End byte

  for (int i = 0; i < TX_PACKET_SIZE; i++) {
    write(fd, &packet[i], 1);
  }
}

void BaseBoardHandler::process_received_data() {
  while (rx_buffer.size() >= RX_PACKET_SIZE) {
    if (rx_buffer.peek(0) == (start_seq >> 8) &&
        rx_buffer.peek(1) == (start_seq & 0xFF) && rx_buffer.peek(2) == 8) {
      uint8_t data[20];
      for (int i = 0; i < 20; i++) {
        data[i] = rx_buffer.peek(3 + i);
        // std::cout << "Data[" << i << "]: " << data[i] << std::endl;
      }

      uint8_t checksum = rx_buffer.peek(23);
      uint8_t end_byte = rx_buffer.peek(24);

      if (checksum == calculate_checksum(data, 20) && end_byte == 0xEF) {
        uint32_t received_data_accel;
        uint32_t received_data_steer;
        uint32_t received_data_aux;
        uint32_t received_data_motor_cmd;
        uint32_t received_data_servo_cmd;

        memcpy(&received_data_accel, data, sizeof(received_data_accel));
        memcpy(&received_data_steer, data + 4, sizeof(received_data_steer));
        memcpy(&received_data_aux, data + 8, sizeof(received_data_aux));
        memcpy(&received_data_motor_cmd, data + 12,
               sizeof(received_data_motor_cmd));
        memcpy(&received_data_servo_cmd, data + 16,
               sizeof(received_data_servo_cmd));

        transimitter_throttle = received_data_accel;
        transimitter_steer = received_data_steer;
        base_board_motor_cmd = received_data_motor_cmd;
        base_board_servo_cmd = received_data_servo_cmd;
        transimitter_aux = received_data_aux;

        // Debugging
        // std::cout << "Received data1: " << received_data_accel << std::endl;
        // std::cout << "Received data2: " << received_data_steer << std::endl;
        // std::cout << "Received data3: " << received_data_aux << std::endl;
        // std::cout << "Received data4: " << received_data_motor_cmd <<
        // std::endl; std::cout << "Received data5: " << received_data_servo_cmd
        // << std::endl; std::cout << std::endl;
        for (int i = 0; i < RX_PACKET_SIZE; i++) {
          rx_buffer.get();
        }
      } else {
        rx_buffer.get();
      }
    } else {
      rx_buffer.get();
    }
  }
}

void BaseBoardHandler::send_loop() {
  while (!stop_flag) {
    AuxState aux_state = getTransimitterAux();
    // std::cout << "aux_state: " << static_cast<int>(aux_state) << std::endl;
    switch (aux_state) {
      case AuxState::DOWN:
        sendPacket(transimitter_throttle, transimitter_steer);
        break;
      case AuxState::MIDDLE:
        sendPacket(transimitter_throttle, transimitter_steer);
        break;
      case AuxState::UP:
        sendPacket(motor_cmd_, servo_cmd_);
        break;
    }
    usleep(static_cast<int>(1e6 /
                            publish_hz));  // Send data according to publish_hz
  }
}

void BaseBoardHandler::receive_loop() {
  while (!stop_flag) {
    uint8_t byte;
    ssize_t n = read(fd, &byte, 1);
    if (n > 0) {
      // std::cout << "Received byte: " << byte << std::endl;
      rx_buffer.put(byte);
      process_received_data();
    } else if (n < 0) {
      std::cerr << "Error reading data" << std::endl;
    }
  }
}

AuxState BaseBoardHandler::getTransimitterAux() {
  if (std::abs(static_cast<int>(transimitter_aux) -
               static_cast<int>(transimitter_aux_middle)) < 100) {
    return AuxState::MIDDLE;
  } else if (std::abs(static_cast<int>(transimitter_aux) -
                      static_cast<int>(transimitter_aux_down)) < 100) {
    return AuxState::DOWN;
  } else if (std::abs(static_cast<int>(transimitter_aux) -
                      static_cast<int>(transimitter_aux_up)) < 100) {
    return AuxState::UP;
  } else {
    std::cout << "Invalid aux state: " << transimitter_aux << std::endl;
    std::cout << "Just return middle" << std::endl;
    return AuxState::MIDDLE;
  }
}

void BaseBoardHandler::loadTransimitterConfig(const std::string& config_path) {
  try {
    YAML::Node config = YAML::LoadFile(config_path);
    transimitter_throttle_up = config["Throttle"]["up"].as<uint32_t>();
    transimitter_throttle_middle = config["Throttle"]["idle"].as<uint32_t>();
    transimitter_throttle_down = config["Throttle"]["down"].as<uint32_t>();
    transimitter_steer_left = config["Steer"]["left"].as<uint32_t>();
    transimitter_steer_middle = config["Steer"]["idle"].as<uint32_t>();
    transimitter_steer_right = config["Steer"]["right"].as<uint32_t>();
    transimitter_aux_up = config["Aux"]["up"].as<uint32_t>();
    transimitter_aux_middle = config["Aux"]["idle"].as<uint32_t>();
    transimitter_aux_down = config["Aux"]["down"].as<uint32_t>();
    std::cout << "Transimitter config loaded successfully!" << std::endl;
    std::cout << "file: " << config_path << std::endl;
    std::cout << "throttle_up: " << transimitter_throttle_up << std::endl;
    std::cout << "throttle_middle: " << transimitter_throttle_middle
              << std::endl;
    std::cout << "throttle_down: " << transimitter_throttle_down << std::endl;
    std::cout << "steer_left: " << transimitter_steer_left << std::endl;
    std::cout << "steer_middle: " << transimitter_steer_middle << std::endl;
    std::cout << "steer_right: " << transimitter_steer_right << std::endl;
    std::cout << "aux_up: " << transimitter_aux_up << std::endl;
    std::cout << "aux_middle: " << transimitter_aux_middle << std::endl;
    std::cout << "aux_down: " << transimitter_aux_down << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error loading transimitter config: " << e.what() << std::endl;
    std::cerr << "file : " << config_path << std::endl;
  }
}
