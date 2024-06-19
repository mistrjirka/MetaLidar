#pragma once

#include "CoreMinimal.h"

class MathToolkit
{
public:
    template <typename T, size_t S>
    static void calculateLinearFit(CircularBuffer<T, S> circBuffer, size_t size, FVector &vector_fit_a, FVector &vector_fit_b, bool print = false);
};