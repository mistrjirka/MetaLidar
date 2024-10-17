#pragma once

#include "CoreMinimal.h"
#include "SharedStructure.h"
#include "CircularBuffer/CircularBuffer.h"
#include "MathToolkit.generated.h"

UCLASS()
class METALIDAR_API UMathToolkit : public UObject
{
    GENERATED_BODY()
public:
    static FVector ConvertUEToROS(const FVector& UEVector);

    static PointXYZI ConvertUEToROS(const PointXYZI& UEPoint);

    static FVector ConvertUEToROSAngleDegree(const FVector& rotation);

    static FRotator ConvertUEToROSAngleDegree(const FRotator& rotation);

    static std::pair<FVector,FVector> CalculateSphericalFromDepth(
        float distance, 
        float x, 
        float y, 
        float FOVH, 
        uint32 width,
        uint32 height
    );
    
    template <typename T, size_t S>
    static void calculateLinearFit(CircularBuffer<T, S> circBuffer, size_t size, FVector &vector_fit_a, FVector &vector_fit_b, bool print = false);

    static std::pair<float, float> CalculateNDCCoordinates(
    float alpha,
    float beta,
    float FOVH,
    uint32 width,
    uint32 height);
    
    static float calculateHorizontalFOV(float senzorWidth, float focalLength);
};
