#pragma once

#include "CoreMinimal.h"
#include "Kismet/KismetMathLibrary.h"

class METALIDAR_API UAngleToPixelUtility
{
public:
    // Convert degrees to radians
    static constexpr float DegreesToRadians(float degrees)
    {
        return degrees * PI / 180.0f;
    }

    // Calculate pixel coordinates from angles and FOV
    static FIntPoint GetPixelCoordinates(float HorizontalAngle, float VerticalAngle, float FOVH, int32 Width, int32 Height)
    {
        // Convert angles to radians
        float HorizontalAngleRad = FMath::DegreesToRadians(HorizontalAngle);
        float VerticalAngleRad = FMath::DegreesToRadians(VerticalAngle);

        // Calculate FOV in radians
        float FOVHRad = FMath::DegreesToRadians(FOVH);
        float AspectRatio = static_cast<float>(Width) / Height;
        float FOVVRad = 2 * FMath::Atan(FMath::Tan(FOVHRad / 2) / AspectRatio);

        // Calculate normalized device coordinates (NDC)
        float NDC_X = FMath::Tan(HorizontalAngleRad) / FMath::Tan(FOVHRad / 2);
        float NDC_Y = FMath::Tan(VerticalAngleRad) / FMath::Tan(FOVVRad / 2);

        // Convert NDC to pixel coordinates
        int32 PixelX = static_cast<int32>((NDC_X + 1.0f) / 2.0f * Width);
        int32 PixelY = static_cast<int32>((1.0f - NDC_Y) / 2.0f * Height);

        return FIntPoint(PixelX, PixelY);
    }
};
