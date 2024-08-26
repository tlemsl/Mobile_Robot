#include "BaseBoardHandler.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdexcept>

#define BUFFER_SIZE 1024
#define PACKET_SIZE 13

uint8_t calculate_checksum(uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

BaseBoardHandler::BaseBoardHandler(const std::string& port, uint16_t start_seq, double publish_hz) 
    : serial_port(port), start_seq(start_seq), publish_hz(publish_hz), stop_flag(false), counter(0), rx_buffer(BUFFER_SIZE) {
    fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
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
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 10;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Error from tcsetattr");
    }
}

BaseBoardHandler::~BaseBoardHandler() {
    stop();
    close(fd);
}

void BaseBoardHandler::start() {
    stop_flag = false;
    // For debug
    // send_thread = std::thread(&BaseBoardHandler::send_loop, this);
    receive_thread = std::thread(&BaseBoardHandler::receive_loop, this);
}

void BaseBoardHandler::stop() {
    stop_flag = true;
    if (send_thread.joinable()) send_thread.join();
    if (receive_thread.joinable()) receive_thread.join();
}

void BaseBoardHandler::sendPacket(int accel, int steer) {
    uint8_t packet[PACKET_SIZE];
    uint64_t combined_data = ((uint64_t)(accel + 1000) << 32) | (steer + 1000);

    packet[0] = (start_seq >> 8) & 0xFF;
    packet[1] = start_seq & 0xFF;
    packet[2] = 8;  // Length of data

    memcpy(&packet[3], &combined_data, sizeof(combined_data));

    packet[11] = calculate_checksum(&packet[3], 8);
    packet[12] = 0xEF;  // End byte

    for (int i = 0; i < PACKET_SIZE; i++) {
        write(fd, &packet[i], 1);
    }
}

void BaseBoardHandler::process_received_data() {
    while (rx_buffer.size() >= PACKET_SIZE) {
        if (rx_buffer.peek(0) == (start_seq >> 8) && rx_buffer.peek(1) == (start_seq & 0xFF) && rx_buffer.peek(2) == 8) {
            uint8_t data[8];
            for (int i = 0; i < 8; i++) {
                data[i] = rx_buffer.peek(3 + i);
            }

            uint8_t checksum = rx_buffer.peek(11);
            uint8_t end_byte = rx_buffer.peek(12);

            if (checksum == calculate_checksum(data, 8) && end_byte == 0xEF) {
                uint64_t received_data;
                memcpy(&received_data, data, sizeof(received_data));

                int received_data1 = ((received_data >> 32) & 0xFFFFFFFF) - 1000;
                int received_data2 = (received_data & 0xFFFFFFFF) - 1000;
                
                // Debugging
                // std::cout << "Received data1: " << received_data1 << std::endl;
                // std::cout << "Received data2: " << received_data2 << std::endl;

                actual_accel = received_data1;
                actual_steer = received_data2;

                for (int i = 0; i < PACKET_SIZE; i++) {
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
        sendPacket(counter, ~counter);
        counter++;
        usleep(static_cast<int>(1e6 / publish_hz));  // Send data according to publish_hz
    }
}

void BaseBoardHandler::receive_loop() {
    while (!stop_flag) {
        uint8_t byte;
        ssize_t n = read(fd, &byte, 1);
        if (n > 0) {
            rx_buffer.put(byte);
            process_received_data();
        } else if (n < 0) {
            std::cerr << "Error reading data" << std::endl;
        }
    }
}

