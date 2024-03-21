#pragma once

#include <array>
#include <cstddef>

template<typename T, size_t S>
class CircularBuffer {
public:
    void put(T item) {
        buffer_[tail_++] = item;
        am = std::min(am++, S);
        if (tail_ >= S) {
            tail_ = 0;
        }
        if (tail_ == head_) {
            if (++head_ >= S) {
                head_ = 0;
            }
        }
    }

    std::array<T, S> get_all() {
        //UE_LOG(LogTemp, Warning, TEXT("Getting all"));
        std::array<T, S> items;
        size_t index = head_;
        //UE_LOG(LogTemp, Warning, TEXT("Head: %d"), head_);
        size_t i = 0;
        do {
            items[i++] = buffer_[index++];
            //UE_LOG(LogTemp, Warning, TEXT("Index: %d currend output Index %d"), index, i);
            index = index % S;
        } while (index != tail_);
        return items;
    }

    bool empty() const {
        return head_ == tail_;
    }

    size_t size() const {
        if (head_ <= tail_) {
            return tail_ - head_ + 1;
        } else {
            return (size_t)((int)tail_ - (int)head_ + 1 + S);
        }
    }

private:
    std::array<T, S> buffer_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t am = 0;
};
