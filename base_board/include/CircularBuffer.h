#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <vector>
#include <mutex>

class CircularBuffer {
public:
    CircularBuffer(size_t size);
    bool is_full() const;
    bool is_empty() const;
    size_t capacity() const;
    size_t size() const;
    void put(uint8_t data);
    uint8_t get();
    uint8_t peek(size_t index) const;

private:
    std::vector<uint8_t> buffer;
    size_t head;
    size_t tail;
    bool full;
    mutable std::mutex mutex;
};

#endif // CIRCULAR_BUFFER_H
