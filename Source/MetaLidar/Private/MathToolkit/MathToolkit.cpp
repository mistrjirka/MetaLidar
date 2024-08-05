#include "MathToolkit/MathToolkit.h"

template <typename T, size_t S>
void MathToolkit::calculateLinearFit(CircularBuffer<T, S> circBuffer, size_t size, FVector &vector_fit_a,
                                     FVector &vector_fit_b, bool print)
{
  if (circBuffer.size() < S)
  {
    return;
  }
  std::array<std::pair<FVector, uint32>, S> buffer = circBuffer.get_all();
  double sumX = 0.0, sumY[3], sumXY[3], sumX2 = 0.0;
  for (int i = 0; i < 3; i++)
  {
    sumY[i] = 0.0;
    sumXY[i] = 0.0;
  }

  for (int i = 0; i < S; i++)
  {
    sumX += buffer[i].second;
    sumY[0] += buffer[i].first.X;
    sumY[1] += buffer[i].first.Y;
    sumY[2] += buffer[i].first.Z;
    if (print)
    {
      UE_LOG(LogTemp, Warning, TEXT("Point %d: %f, %f, %f"), i, buffer[i].first.X, buffer[i].first.Y, buffer[i].first.Z);
    }
    // check(std::isnan(buffer[i].first.Z) == false);

    sumXY[0] += buffer[i].second * buffer[i].first.X;
    sumXY[1] += buffer[i].second * buffer[i].first.Y;
    sumXY[2] += buffer[i].second * buffer[i].first.Z;

    sumX2 += buffer[i].second * buffer[i].second;
  }

  double denominator = S * sumX2 - sumX * sumX;
  ////check(std::isnan(denominator) == false);
  ////check(std::isnan(sumXY[2]) == false);
  ////check(std::isnan(sumX) == false);
  ////check(std::isnan(sumY[2]) == false);
  if (denominator == 0.0)
  {
    // Handle singular matrix case
    vector_fit_a[0] = vector_fit_a[1] = vector_fit_a[2] = 0.0;
    vector_fit_b[0] = vector_fit_b[1] = vector_fit_b[2] = 0.0;
    return;
  }

  vector_fit_a[0] = (S * sumXY[0] - sumX * sumY[0]) / denominator;
  vector_fit_a[1] = (S * sumXY[1] - sumX * sumY[1]) / denominator;
  vector_fit_a[2] = (S * sumXY[2] - sumX * sumY[2]) / denominator;
  ////check(std::isnan(vector_fit_a[2]) == false);

  vector_fit_b[0] = (sumY[0] - vector_fit_a[0] * sumX) / S;
  vector_fit_b[1] = (sumY[1] - vector_fit_a[1] * sumX) / S;
  vector_fit_b[2] = (sumY[2] - vector_fit_a[2] * sumX) / S;
  if (print)
  {
    UE_LOG(LogTemp, Warning, TEXT("Resulting Linear fit: %f, %f, %f, %f, %f, %f"), vector_fit_a[0], vector_fit_a[1], vector_fit_a[2], vector_fit_b[0], vector_fit_b[1], vector_fit_b[2]);
  }
}

FVector MathToolkit::ConvertUEToROS(const FVector &UEVector)
{
  // Convert Unreal Engine coordinates (left-handed) to ROS coordinates (right-handed)
  FVector ROSVector;
  ROSVector.X = UEVector.X;
  ROSVector.Y = -UEVector.Y;
  ROSVector.Z = UEVector.Z;

  return ROSVector;
}

FVector MathToolkit::ConvertUEToROSAngleDegree(const FVector &rotation)
{
  // Convert Unreal Engine angles (left-handed) to ROS angles (right-handed)
  FVector ROSRotation;
  ROSRotation.X = -rotation.X;
  ROSRotation.Y = rotation.Y;
  ROSRotation.Z = -rotation.Z;
  if (ROSRotation.Z < 0)
  {
    ROSRotation.Z += 360;
  }
  if (ROSRotation.X < 0)
  {
    ROSRotation.X += 360;
  }
  if (ROSRotation.Y < 0)
  {
    ROSRotation.Y += 360;
  }

  return ROSRotation;
}

FRotator MathToolkit::ConvertUEToROSAngleDegree(const FRotator &rotation)
{
  // Convert Unreal Engine angles (left-handed) to ROS angles (right-handed)
  FRotator ROSRotation;
  ROSRotation.Pitch = -rotation.Pitch;
  ROSRotation.Roll = rotation.Roll;
  ROSRotation.Yaw = -rotation.Yaw;
  if (ROSRotation.Yaw < 0)
  {
    ROSRotation.Yaw += 360;
  }
  if (ROSRotation.Pitch < 0)
  {
    ROSRotation.Pitch += 360;
  }
  if (ROSRotation.Roll < 0)
  {
    ROSRotation.Roll += 360;
  }

  return ROSRotation;
}

PointXYZI MathToolkit::ConvertUEToROS(const PointXYZI &UEPoint)
{
  // Convert Unreal Engine coordinates (left-handed) to ROS coordinates (right-handed)
  PointXYZI ROSPoint;
  ROSPoint.x = UEPoint.x;
  ROSPoint.y = -UEPoint.y;
  ROSPoint.z = UEPoint.z;
  ROSPoint.intensity = UEPoint.intensity;

  return ROSPoint;
}

std::pair<FVector, FVector> MathToolkit::CalculateSphericalFromDepth(
    float Depth,
    float x,
    float y,
    float FOVH,
    uint32 width,
    uint32 height)
{
    Depth *= 0.01f;

    float NDC_X = (2.0f * x / width) - 1.0f;
    float NDC_Y = 1.0f - (2.0f * y / height);

    float AspectRatio = static_cast<float>(width) / height;
    float FOVHRad = FMath::DegreesToRadians(FOVH);
    float FOVVRad = 2.0f * FMath::Atan(FMath::Tan(FOVHRad / 2.0f) / AspectRatio);

    float halfFOVHRad = FOVHRad / 2.0f;
    float halfFOVVRad = FOVVRad / 2.0f;
    float tanHalfFOVHRad = FMath::Tan(halfFOVHRad);
    float tanHalfFOVVRad = FMath::Tan(halfFOVVRad);

    float CameraX = Depth;
    float CameraY = NDC_X * Depth * tanHalfFOVHRad;
    float CameraZ = NDC_Y * Depth * tanHalfFOVVRad;

    FVector point(CameraX * 100.0f, CameraY * 100.0f, CameraZ * 100.0f);

    float r = FMath::Sqrt(FMath::Square(point.X) + FMath::Square(point.Y) + FMath::Square(point.Z));
    float vCoord = FMath::Acos(point.Z / r);
    float hCoord = FMath::Atan2(point.Y, point.X);

    vCoord = (PI / 2.0f) - vCoord;
    FVector spherical(r, hCoord, vCoord);

    return std::pair<FVector, FVector>(spherical, point);
}
