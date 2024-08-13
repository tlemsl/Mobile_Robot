#include "CircularBuffer.h"
#include <stdexcept>

CircularBuffer::CircularBuffer(size_t size) 
    : buffer(size), head(0), tail(0), full(false) {}

bool CircularBuffer::is_full() const {
    return full;
}

bool CircularBuffer::is_empty() const {
    return (!full && (head == tail));
}

size_t CircularBuffer::capacity() const {
    return buffer.size();
}

size_t CircularBuffer::size() const {
    size_t size = buffer.capacity();
    if (!full) {
        if (head >= tail) {
            size = head - tail;
        } else {
            size = buffer.capacity() + head - tail;
        }
    }
    return size;
}

void CircularBuffer::put(uint8_t data) {
    std::lock_guard<std::mutex> lock(mutex);
    buffer[head] = data;
    head = (head + 1) % buffer.capacity();
    if (full) {
        tail = (tail + 1) % buffer.capacity();
    }
    full = (head == tail);
}

uint8_t CircularBuffer::get() {
    if (is_empty()) {
        throw std::runtime_error("Buffer Underflow");
    }
    std::lock_guard<std::mutex> lock(mutex);
    uint8_t val = buffer[tail];
    full = false;
    tail = (tail + 1) % buffer.capacity();
    return val;
}

uint8_t CircularBuffer::peek(size_t index) const {
    if (is_empty()) {
        throw std::runtime_error("Buffer Underflow");
    }
    return buffer[(tail + index) % buffer.capacity()];
}
