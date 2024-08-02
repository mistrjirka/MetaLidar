#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "Containers/Array.h"

template<typename T>
class TThreadSafeArray
{
public:
    // Constructor
    TThreadSafeArray() {}
    ~TThreadSafeArray() {}

    // Thread-safe Add
    void Add(const T& Item)
    {
        FScopeLock Lock(&CriticalSection);
        Array.Add(Item);
    }

    // Thread-safe Remove
    bool Remove(const T& Item)
    {
        FScopeLock Lock(&CriticalSection);
        return Array.Remove(Item) > 0;
    }

    // Thread-safe Get
    T Get(int32 Index) const
    {
        FScopeLock Lock(&CriticalSection);
        return Array.IsValidIndex(Index) ? Array[Index] : T();
    }

    // Thread-safe Num
    int32 Num() const
    {
        FScopeLock Lock(&CriticalSection);
        return Array.Num();
    }

    void Reset()
    {
        FScopeLock Lock(&CriticalSection);
        Array.Reset();
    }

    void Empty()
    {
        FScopeLock Lock(&CriticalSection);
        Array.Empty();
    }

    // Other thread-safe operations as needed...

private:
    mutable FCriticalSection CriticalSection;
    TArray<T> Array;
};
