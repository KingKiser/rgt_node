#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <iterator>

template <typename T>
class CircularBuffer {
private:
    T* buffer;
    size_t head = 0;
    size_t tail = 0;
    size_t _size = 0;
    size_t _capacity;

public:
    explicit CircularBuffer(size_t capacity) : _capacity(capacity) {
        buffer = new T[_capacity]; 
    }

    ~CircularBuffer() { delete[] buffer; } 

    void push_back(const T& item) {
        buffer[tail] = item;
        if (_size < _capacity) {
            _size++;
        } else {
            head = (head + 1) % _capacity; 
        }
        tail = (tail + 1) % _capacity;
    }

    void pop_front() {
        if (_size > 0) {
            head = (head + 1) % _capacity; 
            _size--;
        }
    }

    size_t size() const { return _size; } 
    size_t capacity() const { return _capacity; } 
    bool empty() const { return _size == 0; } 

    T& front() { return buffer[head]; }
    const T& front() const { return buffer[head]; }
    T& back() { return buffer[(tail + _capacity - 1) % _capacity]; }
    const T& back() const { return buffer[(tail + _capacity - 1) % _capacity]; }

    template <typename ValueType>
    class Iterator {
    public:
        // STL 알고리즘 호환을 위한 필수 정의들
        using iterator_category = std::forward_iterator_tag;
        using value_type = ValueType;
        using pointer = ValueType*;
        using reference = ValueType&;
        using difference_type = std::ptrdiff_t;

        Iterator(ValueType* buf, size_t cap, size_t h, size_t idx) 
            : buffer(buf), capacity(cap), head(h), logical_idx(idx) {}

        reference operator*() const { return buffer[(head + logical_idx) % capacity]; }
        pointer operator->() const { return &buffer[(head + logical_idx) % capacity]; }

        Iterator& operator++() {
            logical_idx++;
            return *this;
        }

        Iterator operator++(int) {
            Iterator tmp = *this;
            logical_idx++;
            return tmp;
        }

        // std::max_element 등에서 사용하는 비교 연산자
        bool operator==(const Iterator& other) const {
            return logical_idx == other.logical_idx;
        }

        bool operator!=(const Iterator& other) const {
            return !(*this == other);
        }

    private:
        ValueType* buffer;
        size_t capacity;
        size_t head;
        size_t logical_idx;
    };

    using iterator = Iterator<T>;
    using const_iterator = Iterator<const T>;

    iterator begin() { return iterator(buffer, _capacity, head, 0); }
    iterator end() { return iterator(buffer, _capacity, head, _size); }
    const_iterator begin() const { return const_iterator(buffer, _capacity, head, 0); }
    const_iterator end() const { return const_iterator(buffer, _capacity, head, _size); }
};

#endif