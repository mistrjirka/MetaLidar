#include "MathToolkit/MathToolkit.h"

template <typename T, size_t S>
void MathToolkit::calculateLinearFit(CircularBuffer<T, S> circBuffer, size_t size, FVector& vector_fit_a,
                                                  FVector& vector_fit_b, bool print)
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
    if(print)
    {
      UE_LOG(LogTemp, Warning, TEXT("Point %d: %f, %f, %f"),i, buffer[i].first.X, buffer[i].first.Y, buffer[i].first.Z);
    }
    //check(std::isnan(buffer[i].first.Z) == false);
    
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
  if (denominator == 0.0 )
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
  if(print)
  {
    UE_LOG(LogTemp, Warning, TEXT("Resulting Linear fit: %f, %f, %f, %f, %f, %f"), vector_fit_a[0], vector_fit_a[1], vector_fit_a[2], vector_fit_b[0], vector_fit_b[1], vector_fit_b[2]);
  }
}