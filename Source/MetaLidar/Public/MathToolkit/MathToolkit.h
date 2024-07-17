#pragma once

#include "CoreMinimal.h"
#include "SharedStructure.h"


class MathToolkit
{
public:
    static FVector ConvertUEToROS(const FVector& UEVector);

    static PointXYZI ConvertUEToROS(const PointXYZI& UEPoint);

    static FVector ConvertUEToROSAngle(const FVector& rotation);

    template <typename T, size_t S>
    static void calculateLinearFit(CircularBuffer<T, S> circBuffer, size_t size, FVector &vector_fit_a, FVector &vector_fit_b, bool print = false);
};