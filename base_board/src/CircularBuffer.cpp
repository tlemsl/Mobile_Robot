#include "CircularBuffer.h"

#include <stdexcept>

CircularBuffer::CircularBuffer(size_t size) : buffer_(size) {}

void CircularBuffer::put(uint8_t value) {
  if (full()) {
    throw std::runtime_error("Buffer is full");
  }

  buffer_[tail_] = value;
  tail_ = (tail_ + 1) % buffer_.size();
  ++size_;
}

uint8_t CircularBuffer::get() {
  if (empty()) {
    throw std::runtime_error("Buffer is empty");
  }

  uint8_t value = buffer_[head_];
  head_ = (head_ + 1) % buffer_.size();
  --size_;
  return value;
}

uint8_t CircularBuffer::peek(size_t offset) const {
  if (empty() || offset >= size_) {
    throw std::runtime_error("Invalid peek operation");
  }

  return buffer_[(head_ + offset) % buffer_.size()];
}
