#pragma once

#include <array>
#include <cstddef>
#include "CoreMinimal.h"


template<typename T, size_t S>
class CircularBuffer {
public:
    void put(T item) {
        int prev_tail = tail_;
        tail_ = (tail_ + 1) % S;

        if (tail_ == head_) {
            head_ = (head_ + 1) % S;
        }

        buffer_[tail_] = item;

    }

    std::array<T, S> get_all(bool print = false) {
        //UE_LOG(LogTemp, Warning, TEXT("Getting all"));
        std::array<T, S> items;
        size_t index = head_;
        if(print){
            UE_LOG(LogTemp, Warning, TEXT("Tail: %d"), tail_);
            UE_LOG(LogTemp, Warning, TEXT("Head: %d"), head_);
        }
        size_t i = 0;
        do {
            if(print)
                UE_LOG(LogTemp, Warning, TEXT("Index: %d currend output Index %d"), index, i);
            items[i++] = buffer_[index++];

            index = index % S;
        } while (i < S);
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
};
