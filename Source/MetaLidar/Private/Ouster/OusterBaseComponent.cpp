// Fill out your copyright notice in the Description page of Project Settings.
#include "Ouster/OusterBaseComponent.h"

#include "Logging/LogMacros.h"
#include <cstdint>
#include <utility>
#include "DrawDebugHelpers.h"
// Sets default values for this component's properties
UOusterBaseComponent::UOusterBaseComponent()
{
  // Set this component to be initialized when the game starts, and to be ticked
  // every frame.  You can turn these features off to improve performance if you
  // don't need them.
  PrimaryComponentTick.bCanEverTick = false;

  // Check Platform supports multithreading
  SupportMultithread = FPlatformProcess::SupportsMultithreading();

  // Set-up initial values
  SensorModel = EOusterModelName::OS1;
  SamplingRate = EOusterFrequency::SRO10;
  ReturnMode = EOusterLaserReturnMode::StrongestO;
  SensorIP = FString(TEXT("127.0.0.1"));
  DestinationIP = FString(TEXT("0.0.0.0"));
  ScanPort = 2368;
  PositionPort = 8308;
  PacketSeq = 0;
  UE_LOG(LogTemp, Warning, TEXT("Recommended number of workers: %d"), FPlatformMisc::NumberOfWorkerThreadsToSpawn());
  ThreadNumScan = FMath::Max(FMath::RoundToInt(FPlatformMisc::NumberOfWorkerThreadsToSpawn() * 0.1588 + 0.7978),
                             1);  // calculated based on the number of cores
  ThreadNumPackage = FMath::Max(FMath::RoundToInt(FPlatformMisc::NumberOfWorkerThreadsToSpawn() * 0.4127 + 0.5611), 1);
  UE_LOG(LogTemp, Warning, TEXT("ThreadNum scan: %d"), ThreadNumScan);
  UE_LOG(LogTemp, Warning, TEXT("ThreadNum package: %d"), ThreadNumPackage);
}

// Called when the game starts
void UOusterBaseComponent::BeginPlay()
{
  Super::BeginPlay();
  this->baseTime = 0;

  ConfigureOusterSensor();
}

// Function to get the size of the data type in bytes
uint32_t UOusterBaseComponent::GetDataTypeSize(uint8 type)
{
  switch (type)
  {
    case PointField::INT8:
      return 1;
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
      return 2;
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
      return 4;
    case PointField::UINT32:
      return 4;
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return 0;  // Should never be reached
  }
}

uint32_t UOusterBaseComponent::CalculatePointStep(const TArray<PointField>& fields)
{
  uint32_t pointStep = 0;
  for (int i = 0; i < fields.Num(); i++)
  {
    // UE_LOG(LogTemp, Warning, TEXT("Field: %d"), fields[i].datatype);
    // UE_LOG(LogTemp, Warning, TEXT("Field: %d"), GetDataTypeSize(fields[i].datatype));
    pointStep += GetDataTypeSize(fields[i].datatype) * fields[i].count;
  }
  return pointStep;
}

// Called every frame
void UOusterBaseComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                         FActorComponentTickFunction* ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UOusterBaseComponent::EndPlay(EEndPlayReason::Type Reason)
{
  Super::EndPlay(Reason);
}

void UOusterBaseComponent::ConfigureOusterSensor()
{
  switch (SensorModel.GetValue())
  {
    case 0: {  // OS1
      float Elevation[] = { -22.5f, -22.1f, -21.8f, -21.4f, -21.1f, -20.7f, -20.4f, -20.0f, -19.7f, -19.3f, -19.0f,
                            -18.6f, -18.2f, -17.9f, -17.5f, -17.2f, -16.8f, -16.5f, -16.1f, -15.8f, -15.4f, -15.1f,
                            -14.7f, -14.4f, -14.0f, -13.6f, -13.3f, -12.9f, -12.6f, -12.2f, -11.9f, -11.5f, -11.2f,
                            -10.8f, -10.5f, -10.1f, -9.7f,  -9.4f,  -9.0f,  -8.7f,  -8.3f,  -8.0f,  -7.6f,  -7.3f,
                            -6.9f,  -6.6f,  -6.2f,  -5.8f,  -5.5f,  -5.1f,  -4.8f,  -4.4f,  -4.1f,  -3.7f,  -3.4f,
                            -3.0f,  -2.7f,  -2.3f,  -1.9f,  -1.6f,  -1.2f,  -0.9f,  -0.5f,  -0.2f,  0.2f,   0.5f,
                            0.9f,   1.2f,   1.6f,   1.9f,   2.3f,   2.7f,   3.0f,   3.4f,   3.7f,   4.1f,   4.4f,
                            4.8f,   5.1f,   5.5f,   5.8f,   6.2f,   6.6f,   6.9f,   7.3f,   7.6f,   8.0f,   8.3f,
                            8.7f,   9.0f,   9.4f,   9.7f,   10.1f,  10.5f,  10.8f,  11.2f,  11.5f,  11.9f,  12.2f,
                            12.6f,  12.9f,  13.3f,  13.6f,  14.0f,  14.4f,  14.7f,  15.1f,  15.4f,  15.8f,  16.1f,
                            16.5f,  16.8f,  17.2f,  17.5f,  17.9f,  18.2f,  18.6f,  19.0f,  19.3f,  19.7f,  20.0f,
                            20.4f,  20.7f,  21.1f,  21.4f,  21.8f,  22.1f,  22.5f };


      std::pair<double,double> NoisePairs[] = {
        {9.682262910120329e-20, 0.05099413638162513},
        {-2.312701453815963e-20, 0.0859635118502373},
        {-2.013560328110606e-19, 0.033198821663645635},
        {3.624554967070408e-19, 0.060609963660023775},
        {1.8178738863901138e-18, 0.04584644732453229},
        {4.685580310922071e-21, 0.01502530303019364},
        {-1.2050801052803727e-21, 0.014204441894820475},
        {1.3253493061719702e-21, 0.013665059576404898},
        {1.1590112878709345e-18, 0.04998633334781023},
        {-1.0966304168193426e-20, 0.011943805862212574},
        {-1.4105504116093882e-20, 0.011300632463552422},
        {1.3538424630412126e-21, 0.010506545507936546},
        {9.822412591695363e-21, 0.00957612081762527},
        {4.5088753293487835e-21, 0.008920589051154872},
        {4.9770906662813354e-21, 0.008459166324749341},
        {6.978618346804423e-21, 0.008896720273648719}
      };
      Sensor.Noise = TArray<std::pair<double, double>>(NoisePairs, UE_ARRAY_COUNT(NoisePairs));
          
      Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
      Sensor.VerticalResolution = 128;
      Sensor.SamplingRate = EOusterFrequency::SRO05;
      Sensor.HorizontalResolution = 1024;
      Sensor.fields.Add(PointField(FString(TEXT("x")), 0, PointField::FLOAT32, 1));
      Sensor.fields.Add(PointField(FString(TEXT("y")), 4, PointField::FLOAT32, 1));
      Sensor.fields.Add(PointField(FString(TEXT("z")), 8, PointField::FLOAT32, 1));
      Sensor.fields.Add(PointField(FString(TEXT("intensity")), 12, PointField::UINT8, 1));
      ;
      Sensor.PointStep = this->CalculatePointStep(Sensor.fields);
      // UE_LOG(LogTemp, Warning, TEXT("PointStep: %d"), Sensor.PointStep);
      Sensor.RowStep = Sensor.HorizontalResolution * Sensor.PointStep;
      Sensor.MinRange = 80.0f;
      Sensor.MaxRange = 12000.0f;
      Sensor.IntensityNoise = 7.0;
      Sensor.ToFNoise = 5.0;

      Sensor.NoiseFrequency = 233.7f;
      Sensor.NoiseAmplitude = 0.01f;

      Sensor.MemoryLabel = "/t07ySQdKFH_meta_lidar";
      Sensor.MemorySize = 40000000;
      Sensor.PacketSize =
          FMath::FloorToInt((float)(1.5f * (sizeof(PointCloud2) + Sensor.HorizontalResolution *
                                                                      Sensor.VerticalResolution * Sensor.PointStep)));
      break;
    }
    case 1: {  // OS2
      float Elevation[] = { -11.2f, -11.1f, -10.9f, -10.7f, -10.5f, -10.4f, -10.2f, -10.0f, -9.8f, -9.7f, -9.5f, -9.3f,
                            -9.1f,  -8.9f,  -8.8f,  -8.6f,  -8.4f,  -8.2f,  -8.1f,  -7.9f,  -7.7f, -7.5f, -7.4f, -7.2f,
                            -7.0f,  -6.8f,  -6.6f,  -6.5f,  -6.3f,  -6.1f,  -5.9f,  -5.8f,  -5.6f, -5.4f, -5.2f, -5.0f,
                            -4.9f,  -4.7f,  -4.5f,  -4.3f,  -4.2f,  -4.0f,  -3.8f,  -3.6f,  -3.5f, -3.3f, -3.1f, -2.9f,
                            -2.7f,  -2.6f,  -2.4f,  -2.2f,  -2.0f,  -1.9f,  -1.7f,  -1.5f,  -1.3f, -1.2f, -1.0f, -0.8f,
                            -0.6f,  -0.4f,  -0.3f,  -0.1f,  0.1f,   0.3f,   0.4f,   0.6f,   0.8f,  1.0f,  1.2f,  1.3f,
                            1.5f,   1.7f,   1.9f,   2.0f,   2.2f,   2.4f,   2.6f,   2.7f,   2.9f,  3.1f,  3.3f,  3.5f,
                            3.6f,   3.8f,   4.0f,   4.2f,   4.3f,   4.5f,   4.7f,   4.9f,   5.0f,  5.2f,  5.4f,  5.6f,
                            5.8f,   5.9f,   6.1f,   6.3f,   6.5f,   6.6f,   6.8f,   7.0f,   7.2f,  7.4f,  7.5f,  7.7f,
                            7.9f,   8.1f,   8.2f,   8.4f,   8.6f,   8.8f,   8.9f,   9.1f,   9.3f,  9.5f,  9.7f,  9.8f,
                            10.0f,  10.2f,  10.4f,  10.5f,  10.7f,  10.9f,  11.1f,  11.2f };

      std::pair<double,double> NoisePairs[] = {
        {9.682262910120329e-20, 0.05099413638162513},
        {-2.312701453815963e-20, 0.0859635118502373},
        {-2.013560328110606e-19, 0.033198821663645635},
        {3.624554967070408e-19, 0.060609963660023775},
        {1.8178738863901138e-18, 0.04584644732453229},
        {4.685580310922071e-21, 0.01502530303019364},
        {-1.2050801052803727e-21, 0.014204441894820475},
        {1.3253493061719702e-21, 0.013665059576404898},
        {1.1590112878709345e-18, 0.04998633334781023},
        {-1.0966304168193426e-20, 0.011943805862212574},
        {-1.4105504116093882e-20, 0.011300632463552422},
        {1.3538424630412126e-21, 0.010506545507936546},
        {9.822412591695363e-21, 0.00957612081762527},
        {4.5088753293487835e-21, 0.008920589051154872},
        {4.9770906662813354e-21, 0.008459166324749341},
        {6.978618346804423e-21, 0.008896720273648719}
      };
      Sensor.Noise = TArray<std::pair<double, double>>(NoisePairs, UE_ARRAY_COUNT(NoisePairs));

      Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
      Sensor.VerticalResolution = 128;
      Sensor.SamplingRate = EOusterFrequency::SRO05;
      Sensor.HorizontalResolution = 2048;
      Sensor.fields.Add(PointField(FString(TEXT("x")), 0, PointField::FLOAT32, 1));
      Sensor.fields.Add(PointField(FString(TEXT("y")), 4, PointField::FLOAT32, 1));
      Sensor.fields.Add(PointField(FString(TEXT("z")), 8, PointField::FLOAT32, 1));
      Sensor.fields.Add(PointField(FString(TEXT("Intensity")), 12, PointField::UINT8, 1));

      Sensor.PointStep = this->CalculatePointStep(Sensor.fields);
      UE_LOG(LogTemp, Warning, TEXT("PointStep: %d"), Sensor.PointStep);
      Sensor.RowStep = Sensor.HorizontalResolution * Sensor.PointStep;
      Sensor.MinRange = 80.0f;
      Sensor.MaxRange = 4000.0f;
      Sensor.MemoryLabel = "/t07ySQdKFH_meta_lidar";
      Sensor.MemorySize = 40000000;
      Sensor.IntensityNoise = 7.0;
      Sensor.ToFNoise = 5.0;
      Sensor.NoiseFrequency = 233.7f;
      Sensor.NoiseAmplitude = 0.01f;
      Sensor.PacketSize =
          FMath::FloorToInt((float)(1.5f * (sizeof(PointCloud2) + Sensor.HorizontalResolution *
                                                                      Sensor.VerticalResolution * Sensor.PointStep)));
      break;
    }
  }

  switch (SamplingRate.GetValue())
  {
    case 0:
      Sensor.SamplingRate = 5;
      break;
    case 1:
      Sensor.SamplingRate = 10;
      break;
    case 2:
      Sensor.SamplingRate = 15;
      break;
    case 3:
      Sensor.SamplingRate = 20;
      break;
    default:
      Sensor.SamplingRate = 10;
      break;
  }

  switch (ReturnMode.GetValue())
  {
    case 0:
      Sensor.ReturnMode = 55;
      break;
    case 1:  // Last Return : Not implemented
      Sensor.ReturnMode = 56;
      break;
    case 2:  // Dual Return : Not implemented
      Sensor.ReturnMode = 57;
      break;
  }

  UE_LOG(LogTemp, Warning, TEXT("Horizontal Resolution: %d"), Sensor.HorizontalResolution);
  UE_LOG(LogTemp, Warning, TEXT("Sampling Rate: %d"), Sensor.SamplingRate);

  // Initialize raycast vector and azimuth vector
  Sensor.AzimuthAngle.Init(90.f, this->Sensor.VerticalResolution * this->Sensor.HorizontalResolution);

  // Sensor.DataPacket.AddUninitialized(DATA_PACKET_PAYLOAD);
}

// The VLP-16 measures reflectivity of an object independent of laser power and
// distances involved. Reflectivity values returned are based on laser
// calibration against NIST-calibrated reflectivity reference targets at the
// factory.
//
// For each laser measurement, a reflectivity byte is returned in addition to
// distance. Reflectivity byte values are segmented into two ranges, allowing
// software to distinguish diffuse reflectors (e.g. tree trunks, clothing) in
// the low range from retroreflectors (e.g. road signs, license plates) in the
// high range.
//
// A retroreflector reflects light back to its source with a minimum of
// scattering. The VLP-16 provides its own light, with negligible separation
// between transmitting laser and receiving detector, so retroreflecting
// surfaces pop with reflected IR light compared to diffuse reflectors that tend
// to scatter reflected energy.
//
//   - Diffuse reflectors report values from 0 to 100 for reflectivities from 0%
//   to 100%.
//   - Retroreflectors report values from 101 to 255, where 255 represents an
//   idealreflection.
//
// Note: When a laser pulse doesn't result in a measurement, such as when a
// laser is shot skyward, both distance and
//        reflectivity values will be 0. The key is a distance of 0, because 0
//        is a valid reflectivity value (i.e. one step above noise).

float UOusterBaseComponent::GenerateGaussianNoise(float mean, float stdDev)
{
  float u1 = FMath::RandRange(0.f, 1.f);
  float u2 = FMath::RandRange(0.f, 1.f);
  float randStdNormal =
      FMath::Sqrt(-2.0f * FMath::Loge(u1)) * FMath::Sin(2.0f * PI * u2);  // applying Box-Muller transform
  return mean + stdDev * randStdNormal;
}

// Function to generate 3D noise based on a normal distribution
FVector UOusterBaseComponent::Generate3DNoise(float StandardDeviation)
{
  // Static is used here for efficiency, so the engine is only constructed once.
  static std::default_random_engine Generator;

  // Normal distribution centered at 0 with the given standard deviation
  std::normal_distribution<float> Distribution(0.0f, StandardDeviation);

  // Generate the noise components
  float NoiseX = Distribution(Generator);
  float NoiseY = Distribution(Generator);
  float NoiseZ = Distribution(Generator);

  // Return the noise vector
  return FVector(NoiseX, NoiseY, NoiseZ);
}

// Function to calculate rotation noise based on azimuth, frequency, amplitude, and current time
FRotator CalculateRotationNoise(float Azimuth, float Frequency, float Amplitude, float CurrentTime)
{
  // Convert azimuth to a rotational component (here, we'll apply it to yaw for demonstration)
  float YawNoise = 0.0f;

  // Assuming you want the noise to affect the yaw based on the azimuth
  // If you want to include pitch and roll variations, you could apply similar calculations
  float PitchNoise = FMath::Sin(CurrentTime * Frequency) * Amplitude;  // For simplicity, not applying noise here
  float RollNoise = 0.0f;                                              // For simplicity, not applying noise here

  // Create the noise rotation
  FRotator NoiseRotation = FRotator(PitchNoise, YawNoise, RollNoise);

  return NoiseRotation;
}
// Function to calculate noise vector based on azimuth, frequency, and amplitude
FVector CalculateNoise(float Azimuth, float Frequency, float Amplitude, float CurrentTime)
{
  // Convert azimuth to radians
  float AzimuthInRadians = FMath::DegreesToRadians(Azimuth);

  // Calculate horizontal direction based on azimuth
  FVector HorizontalDirection = FVector(FMath::Cos(AzimuthInRadians), FMath::Sin(AzimuthInRadians), 0.0f);

  // Use a sinusoidal function to simulate vertical vibration
  float VerticalMovement = FMath::Sin(CurrentTime * Frequency) * Amplitude;

  // Apply amplitude to horizontal direction to scale the effect
  FVector ScaledHorizontalDirection = HorizontalDirection * Amplitude;

  // Add the vertical movement to the Z component of the horizontal direction
  FVector NoiseVector = ScaledHorizontalDirection + FVector(0.0f, 0.0f, VerticalMovement);

  return NoiseVector;
}

template <typename T>
void ShuffleArray(TArray<T>& ArrayToShuffle)
{
  if (ArrayToShuffle.Num() > 0)
  {
    int32 LastIndex = ArrayToShuffle.Num() - 1;
    for (int32 i = 0; i <= LastIndex; ++i)
    {
      int32 Index = FMath::RandRange(i, LastIndex);
      if (i != Index)
      {
        ArrayToShuffle.Swap(i, Index);
      }
    }
  }
}

FRotator UOusterBaseComponent::GetLidarRotation(float Azimuth, float Elevation, FRotator lidarRotation)
{
  FRotator Rotation(0.f, 0.f, 0.f);
  Rotation.Add(Elevation, Azimuth, 0.f);
  Rotation = UKismetMathLibrary::ComposeRotators(Rotation, lidarRotation);
  return Rotation;
}

FRotator UOusterBaseComponent::AddRotationNoise(FRotator Rotation, float frequency, float amplitude, float azimuth,
                                                float time)
{
  FRotator noise = CalculateRotationNoise(azimuth, frequency, amplitude, time);
  FQuat Quat = FQuat(noise);
  FQuat Quat2 = FQuat(Rotation);
  FQuat NoiseCombined = Quat * Quat2;
  Rotation = NoiseCombined.Rotator();
  return Rotation;
}

void UOusterBaseComponent::GetScanData()
{
  // complex collisions: true
  FCollisionQueryParams TraceParams = FCollisionQueryParams(TEXT("LaserTrace"), true, GetOwner());

  TraceParams.bReturnPhysicalMaterial = true;
  TraceParams.bTraceComplex = true;

  // Get owner's location and rotation
  FVector LidarPosition = this->GetActorLocation();
  LidarRotation = this->GetActorRotation();

  AActor* Owner = GetOwner();

  Sensor.Transform = Owner->GetTransform();

  // Initialize array for raycast result
  Sensor.RecordedHits.Init(std::make_pair(FHitResult(ForceInit), FRotator(0.f, 0.f, 0.f)),
                           Sensor.VerticalResolution * Sensor.HorizontalResolution);

  // Calculate batch size for 'ParallelFor' based on workable thread

  // Divide work across threads, each thread processes a portion of all hits
  // (vertically and horizontally)
  /*FMath::Max(FPlatformMisc::NumberOfWorkerThreadsToSpawn() / 2, 1)*/;

  /*int timebetweencores = 15;
  int TimeFromStart = FMath::FloorToInt(Owner->GetGameTimeSinceCreation()/timebetweencores);
  int prevThreadNumScan = ThreadNumScan;
  ThreadNumScan = numberOfCoresInSecond[TimeFromStart % 6];
  if(ThreadNumScan != prevThreadNumScan)
  UE_LOG(LogTemp, Warning, TEXT("ThreadNum: %d"), ThreadNumScan);*/

  const int DivideEnd = FMath::FloorToInt((float)(Sensor.RecordedHits.Num() / ThreadNumScan));
  int count = 0;

  float horizontalStepAngle = (float)(360.f / (float)Sensor.HorizontalResolution);
  // UE_LOG(LogTemp, Warning, TEXT("Horizontal Step Angle: %f"), horizontalStepAngle);
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ParallelFor loop inside GetScanData()")
    ParallelFor(
        ThreadNumScan,
        [&](int32 PFIndex) {
          // divide work acrosss threads. Each thread will process a portion of
          // the hits.
          int StartAt = PFIndex * DivideEnd;
          if (StartAt >= Sensor.RecordedHits.Num())
            return;

          int EndAt = StartAt + DivideEnd;
          if (PFIndex == (ThreadNumScan - 1))
            EndAt = Sensor.RecordedHits.Num();

          for (int32 Index = StartAt; Index < EndAt; ++Index)
          {
            float Azimuth, Elevation;

            Azimuth = horizontalStepAngle * FMath::FloorToInt((float)(Index / Sensor.VerticalResolution));
            Elevation = Sensor.ElevationAngle[Index % Sensor.VerticalResolution];
            FRotator Rotation = GetLidarRotation(Azimuth, Elevation, LidarRotation);

            float virtualTime =
                baseTime + ((float)Index / (float)(Sensor.VerticalResolution * Sensor.HorizontalResolution)) *
                               (1.f / Sensor.SamplingRate);
            if (Index % 2048 * 48 == 0)
            {
              UE_LOG(LogTemp, Warning, TEXT("Virtual Time: %f index: %d result first %f the time for the part"),
                     virtualTime, Index,
                     (float)Index / (float)(Sensor.VerticalResolution * Sensor.HorizontalResolution),
                     (1.f / Sensor.SamplingRate));
            }

            //Rotation = AddRotationNoise(Rotation, Sensor.NoiseFrequency, Sensor.NoiseAmplitude, Azimuth, virtualTime);

            FVector EndPoint, BeginPoint;
            {
              TRACE_CPUPROFILER_EVENT_SCOPE_STR("Noise calculation inside loop")

              BeginPoint = LidarPosition + Sensor.MinRange * UKismetMathLibrary::GetForwardVector(Rotation);
              EndPoint = LidarPosition + Sensor.MaxRange * UKismetMathLibrary::GetForwardVector(Rotation);
            }
            FHitResult result;
            {
              TRACE_CPUPROFILER_EVENT_SCOPE_STR("LineTraceSingleByChannel inside loop")
              GetWorld()->LineTraceSingleByChannel(result, BeginPoint, EndPoint, ECC_Visibility, TraceParams,
                                                   FCollisionResponseParams::DefaultResponseParam);
            };
            Sensor.RecordedHits[Index] = std::make_pair(result, Rotation);
          };
        },
        !SupportMultithread);

    baseTime += 1.f / Sensor.SamplingRate;
    /*  for(int i = 0; i < Sensor.RecordedHits.Num(); i++)
      {
        if(Sensor.RecordedHits[i].first.IsValidBlockingHit())
        {
          FVector Location = Sensor.RecordedHits[i].first.Location;
          FVector Normal = Sensor.RecordedHits[i].first.ImpactNormal;
          FVector Start = Sensor.RecordedHits[i].first.TraceStart;
          FVector End = Sensor.RecordedHits[i].first.TraceEnd;

        }
      }
    }*/
    // DrawDebugLine(GetWorld(), BeginPoint, EndPoint, FColor::Red, false, -1, 0, 10.0f);
  }
}

FVector UOusterBaseComponent::GetActorLocation()
{
  return GetOwner()->GetActorLocation();
}

FRotator UOusterBaseComponent::GetActorRotation()
{
  return GetOwner()->GetActorRotation();
}

uint32 UOusterBaseComponent::GetTimestampMicroseconds()
{
  return (uint32)(fmod(GetWorld()->GetTimeSeconds(), 3600.f) * 1000000);  // sec -> microsec
}

void AddToTArray(TArray<uint8>& arr, uint64_t num, int numBytes)
{
  for (int i = 0; i < numBytes; ++i)
  {
    arr.Add(num & 0xFF);
    num >>= 8;
  }
}

void AddStringToTArray(TArray<uint8>& arr, const FString& str)
{
  for (TCHAR c : str)
  {
    arr.Add(c);
  }
}

FVector UOusterBaseComponent::CreateLocationNoise(const FHitResult hit, uint8_t intensity)
{
  // Now you have the start and end points of the trace
  // You can use these to calculate the trace vector
  FVector TraceStart = hit.TraceStart;
  FVector TraceEnd = hit.TraceEnd;

  FVector TraceVector = (TraceEnd - TraceStart).GetSafeNormal();
  
  // get parameters for the standard distribution
  float mean = 0.0f;
  float stdDev = 1.0f;
  int index = FMath::Min(FMath::FloorToInt(intensity / ((255.f)/(float)Sensor.Noise.Num())), Sensor.Noise.Num() - 1);  
  float noiseMultiplier = GenerateGaussianNoise(Sensor.Noise[index].first*100, Sensor.Noise[index].second*100);

  // noiseVector *= noiseScalar;
  FVector ToFNoise = TraceVector * noiseMultiplier;
  return hit.Location + ToFNoise;
}

uint8_t UOusterBaseComponent::GetNoiseForIntensity(uint8_t intensity)
{
  return (uint8_t)GenerateGaussianNoise((float)intensity, Sensor.IntensityNoise);
}

uint8 UOusterBaseComponent::GetIntensity(std::pair<FHitResult, FRotator> hitPair) const
{
  // Get the hit result from the pair
  FHitResult Hit = hitPair.first;

  // Get the distance from the hit result
  float Distance = Hit.Distance;

  FRotator Rotation = hitPair.second;
  FVector StartingVector = UKismetMathLibrary::GetForwardVector(Rotation);
  FVector NormalVector = Hit.ImpactNormal;
  float DotProduct = FVector::DotProduct(StartingVector, NormalVector);
  float lengths = StartingVector.Size() * NormalVector.Size();

  float cos = UKismetMathLibrary::Abs(DotProduct / lengths);

  // float sin = FMath::Sqrt(1 - cos * cos);

  float maxReflectivity = 255 * (Sensor.MaxRange - Distance) / (Sensor.MaxRange - Sensor.MinRange);

  // Calculate the intensity based on the distance
  return (uint8)(cos * maxReflectivity);
}

void UOusterBaseComponent::GenerateDataPacket(uint32 TimeStamp)
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR("Fancy work 1")
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Fancy work 2")

    uint32_t timestamp_sec = TimeStamp / 1000000;
    uint32_t timestamp_nsec = TimeStamp * 1000;
    Sensor.DataPacket.Empty();
    PointCloud2 ScanData;
    ScanData.header.seq = PacketSeq++;
    ScanData.header.stamp.sec = timestamp_sec;
    ScanData.header.stamp.nsec = timestamp_nsec;
    ScanData.header.frame_id = FString(TEXT("ouster"));
    ScanData.height = 1;
    ScanData.fields = Sensor.fields;
    ScanData.is_bigendian = false;
    ScanData.point_step = Sensor.PointStep;
    ScanData.is_dense = true;
    ScanData.numOfFields = Sensor.fields.Num();

    // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket: %d"),
    // Sensor.RecordedHits.Num());
    uint32_t numOfPoints = 0;

    // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket: %d"),
    //      Sensor.RecordedHits.Num());
    AActor* Owner = GetOwner();

    const int DivideEnd = FMath::FloorToInt((float)(Sensor.RecordedHits.Num() / ThreadNumPackage));
    int count = 0;
    TArray<uint8> DataToCopyThread[ThreadNumPackage];
    TArray<uint32> DataToCopySize;
    DataToCopySize.Init(0, ThreadNumPackage);

    ParallelFor(ThreadNumPackage, [&](int32 PFIndex) {
      int StartAt = PFIndex * DivideEnd;
      DataToCopySize[PFIndex] = 0;

      if (StartAt >= Sensor.RecordedHits.Num())
      {
        return;
      }

      int EndAt = StartAt + DivideEnd;

      if (PFIndex == (ThreadNumPackage - 1))
      {
        EndAt = Sensor.RecordedHits.Num();
      }

      for (uint32_t i = StartAt; i < EndAt; i++)
      {
        if (!Sensor.RecordedHits[i].first.IsValidBlockingHit())
        {
          continue;
        }
        PointXYZI Point;

        // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket in forloop: %d"), i);
        FHitResult hit = Sensor.RecordedHits[i].first;
        auto PhysMat = hit.PhysMaterial;
        if (PhysMat != nullptr)
        {
          Point.intensity = GetNoiseForIntensity(GetIntensity(Sensor.RecordedHits[i]));
        }
        else
        {
          Point.intensity = 0.f;
        }
        FVector Location = CreateLocationNoise(hit, Point.intensity);
        FRotator Rotation = Sensor.RecordedHits[i].second;
        FVector HitNormal = hit.ImpactNormal;

        // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket before location: %f"), Location.X);
        FVector RelativeLocation = Sensor.Transform.InverseTransformPosition(Location);
        // RelativeLocation = LidarRotation.UnrotateVector(RelativeLocation);
        //   Rotate the point around the origin
        Point.x = RelativeLocation.X / 100.f;
        Point.y = -RelativeLocation.Y / 100.f;
        Point.z = RelativeLocation.Z / 100.f;

        // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket after location: %f"), Point.x);

        

        DataToCopySize[PFIndex] += 1;

        // FMemory::Memcpy(data, &Point, Sensor.PointStep);
        unsigned char const* data = reinterpret_cast<unsigned char*>(&Point);
        // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket after memcpy: %d"), i);
        // uint64_t whereToCopy = DataToCopy.Num();
        // DataToCopy.SetNum(DataToCopy.Num() + Sensor.PointStep);

        for (int j = 0; j < Sensor.PointStep; j++)
        {
          DataToCopyThread[PFIndex].Add(data[j]);
        }
      }
      // FMemory::Memcpy(DataToCopy.GetData() + whereToCopy, &Point, Sensor.PointStep);
    });

    for (int i = 0; i < ThreadNumPackage; i++)
    {
      numOfPoints += DataToCopySize[i];
    }

    // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket: %d numOfPoints %d"), DataToCopy.Num(), numOfPoints);

    ScanData.width = numOfPoints;
    ScanData.row_step = ScanData.width;

    uint32 DataPacketSize = sizeof(PointCloud2) + numOfPoints * Sensor.PointStep * sizeof(uint8);
    Sensor.DataPacket.Reserve(DataPacketSize);
    // UE_LOG(LogTemp, Warning, TEXT("Seq_number: %d"), ScanData.header.seq);
    // add header
    AddToTArray(Sensor.DataPacket, ScanData.header.seq, 4);
    AddToTArray(Sensor.DataPacket, ScanData.header.stamp.sec, 4);
    AddToTArray(Sensor.DataPacket, ScanData.header.stamp.nsec, 4);
    AddToTArray(Sensor.DataPacket, ScanData.header.frame_id.Len(), 4);
    AddStringToTArray(Sensor.DataPacket, ScanData.header.frame_id);
    AddToTArray(Sensor.DataPacket, ScanData.height, 4);
    AddToTArray(Sensor.DataPacket, ScanData.width, 4);
    AddToTArray(Sensor.DataPacket, ScanData.numOfFields, 4);
    // UE_LOG(LogTemp, Warning, TEXT("GenerateDataPacket: %d"), ScanData.fields.Num());

    for (uint32 i = 0; i < ScanData.numOfFields; i++)
    {
      // UE_LOG(LogTemp, Warning, TEXT("Adding fields: %d"), i);
      // AddToTArray(Sensor.DataPacket, i, 4);
      AddToTArray(Sensor.DataPacket, ScanData.fields[i].name.Len(), 1);
      // UE_LOG(LogTemp, Warning, TEXT("Adding fields: %d"), ScanData.fields[i].name.Len());
      AddStringToTArray(Sensor.DataPacket, ScanData.fields[i].name);
      // UE_LOG(LogTemp, Warning, TEXT("Adding fields: %s"), *ScanData.fields[i].name);
      AddToTArray(Sensor.DataPacket, ScanData.fields[i].offset, 4);
      AddToTArray(Sensor.DataPacket, ScanData.fields[i].datatype, 1);
      AddToTArray(Sensor.DataPacket, ScanData.fields[i].count, 4);
    }
    AddToTArray(Sensor.DataPacket, ScanData.is_bigendian, 1);
    AddToTArray(Sensor.DataPacket, ScanData.point_step, 4);
    AddToTArray(Sensor.DataPacket, ScanData.row_step, 4);
    // add data
    // uint64_t whereToCopy = Sensor.DataPacket.Num();
    // Sensor.DataPacket.SetNum(DataToCopy.Num() + Sensor.DataPacket.Num());
    // FMemory::Memcpy(Sensor.DataPacket.GetData() + whereToCopy, DataToCopy.GetData(), DataToCopy.Num());

    for (uint32_t i = 0; i < ThreadNumPackage; i++)
    {
      for (uint32_t j = 0; j < DataToCopyThread[i].Num(); j++)
      {
        Sensor.DataPacket.Add(DataToCopyThread[i][j]);
      }
    }

    AddToTArray(Sensor.DataPacket, ScanData.is_dense, 1);
  }

  // UE_LOG(LogTemp, Warning, TEXT("Rotation: %f"), LidarRotation.Yaw);
  // UE_LOG(LogTemp, Warning, TEXT("Location: %f"), LidarPosition.X);
}

FString UOusterBaseComponent::DecToHex(int DecimalNumber)
{
  // char array to store hexadecimal number
  char HexaDeciNum[100];

  // counter for hexadecimal number array
  int i = 0;
  while (DecimalNumber != 0)
  {
    // temporary variable to store remainder
    int Temp = 0;

    // storing remainder in temp variable.
    Temp = DecimalNumber % 16;

    // check if Temp < 10
    if (Temp < 10)
    {
      HexaDeciNum[i] = Temp + 48;
      i++;
    }
    else
    {
      HexaDeciNum[i] = Temp + 55;
      i++;
    }

    DecimalNumber = DecimalNumber / 16;
  }

  FString Answer;

  // printing hexadecimal number array in reverse order
  for (int j = i - 1; j >= 0; j--)
  {
    Answer += HexaDeciNum[j];
  }

  return Answer;
}

void UOusterBaseComponent::ASCIItoHEX(FString Ascii, uint8 Hex[])
{
  // Initialize final String
  FString StrHex = "";

  // Make a loop to iterate through
  // every character of ascii string
  for (int i = 0; i < Ascii.Len(); i++)
  {
    // Take a char from
    // position i of string
    // char ch = Ascii[i];

    // Cast char to integer and
    // find its ascii value
    // int tmp = (int)ch;

    // Change this ascii value
    // integer to hexadecimal value
    FString Part = DecToHex((int)Ascii[i]);

    // Add this hexadecimal value
    // to final string.
    StrHex += Part;
  }

  // Return the final string hex
  strcpy(reinterpret_cast<char*>(Hex), TCHAR_TO_ANSI(*StrHex));
}
