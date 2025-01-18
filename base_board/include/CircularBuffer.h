#ifndef BASE_BOARD_INCLUDE_CIRCULAR_BUFFER_H_
#define BASE_BOARD_INCLUDE_CIRCULAR_BUFFER_H_

// C++ system headers
#include <cstdint>
#include <vector>

class CircularBuffer {
 public:
  explicit CircularBuffer(size_t size);

  // Buffer operations
  void put(uint8_t value);
  uint8_t get();
  uint8_t peek(size_t offset) const;

  // Buffer state
  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == buffer_.size(); }

 private:
  std::vector<uint8_t> buffer_;
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t size_ = 0;
};

#endif  // BASE_BOARD_INCLUDE_CIRCULAR_BUFFER_H_
