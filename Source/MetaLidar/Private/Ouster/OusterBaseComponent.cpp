// Fill out your copyright notice in the Description page of Project Settings.
#include "Ouster/OusterBaseComponent.h"

#include "Logging/LogMacros.h"

// Sets default values for this component's properties
UOusterBaseComponent::UOusterBaseComponent() {
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
  NoiseStd = 1.0f;
}

// Called when the game starts
void UOusterBaseComponent::BeginPlay() {
  Super::BeginPlay();
  FString TheFloatStr = FString::SanitizeFloat(NoiseStd);
  UE_LOG(LogTemp, Warning, TEXT("%s"), *TheFloatStr);

  ConfigureOusterSensor();
}

// Function to get the size of the data type in bytes
uint32_t UOusterBaseComponent::GetDataTypeSize(uint8 type) {
  switch (type) {
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
    return 0; // Should never be reached
  }
}

uint32_t UOusterBaseComponent::CalculatePointStep(
    const std::vector<PointField> &fields) {
  uint32_t pointStep = 0;
  for (const auto &field : fields) {
    pointStep += GetDataTypeSize(field.datatype) * field.count;
  }
  return pointStep;
}

// Called every frame
void UOusterBaseComponent::TickComponent(
    float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction *ThisTickFunction) {
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UOusterBaseComponent::EndPlay(EEndPlayReason::Type Reason) {

  FString TheFloatStr = FString::SanitizeFloat(NoiseStd);
  UE_LOG(LogTemp, Warning, TEXT("%s"), *TheFloatStr);
  Super::EndPlay(Reason);
}

void UOusterBaseComponent::ConfigureOusterSensor() {
  switch (SensorModel.GetValue()) {
  case 0: { // OS1
    float Elevation[] = {
        -22.5f, -22.1f, -21.8f, -21.4f, -21.1f, -20.7f, -20.4f, -20.0f, -19.7f,
        -19.3f, -19.0f, -18.6f, -18.2f, -17.9f, -17.5f, -17.2f, -16.8f, -16.5f,
        -16.1f, -15.8f, -15.4f, -15.1f, -14.7f, -14.4f, -14.0f, -13.6f, -13.3f,
        -12.9f, -12.6f, -12.2f, -11.9f, -11.5f, -11.2f, -10.8f, -10.5f, -10.1f,
        -9.7f,  -9.4f,  -9.0f,  -8.7f,  -8.3f,  -8.0f,  -7.6f,  -7.3f,  -6.9f,
        -6.6f,  -6.2f,  -5.8f,  -5.5f,  -5.1f,  -4.8f,  -4.4f,  -4.1f,  -3.7f,
        -3.4f,  -3.0f,  -2.7f,  -2.3f,  -1.9f,  -1.6f,  -1.2f,  -0.9f,  -0.5f,
        -0.2f,  0.2f,   0.5f,   0.9f,   1.2f,   1.6f,   1.9f,   2.3f,   2.7f,
        3.0f,   3.4f,   3.7f,   4.1f,   4.4f,   4.8f,   5.1f,   5.5f,   5.8f,
        6.2f,   6.6f,   6.9f,   7.3f,   7.6f,   8.0f,   8.3f,   8.7f,   9.0f,
        9.4f,   9.7f,   10.1f,  10.5f,  10.8f,  11.2f,  11.5f,  11.9f,  12.2f,
        12.6f,  12.9f,  13.3f,  13.6f,  14.0f,  14.4f,  14.7f,  15.1f,  15.4f,
        15.8f,  16.1f,  16.5f,  16.8f,  17.2f,  17.5f,  17.9f,  18.2f,  18.6f,
        19.0f,  19.3f,  19.7f,  20.0f,  20.4f,  20.7f,  21.1f,  21.4f,  21.8f,
        22.1f,  22.5f};
    Sensor.ElevationAngle.Append(Elevation, UE_ARRAY_COUNT(Elevation));
    Sensor.VerticalResolution = 128;
    Sensor.HorizontalResolution = 2048;
    Sensor.fields = {PointField("x", PointField::FLOAT32, 1),
                     PointField("y", PointField::FLOAT32, 1),
                     PointField("z", PointField::FLOAT32, 1),
                     PointField("intensity", PointField::FLOAT32, 1)};
    Sensor.PointStep = this->CalculatePointStep(Sensor.fields);
    Sensor.RowStep = Sensor.HorizontalResolution * Sensor.PointStep;
    Sensor.MinRange = 80.0f;
    Sensor.MaxRange = 12000.0f;
    Sensor.PacketSize = sizeof(ScanData) + Sensor.HorizontalResolution * Sensor.VerticalResolution * Sensor.PointStep;
    break;
  }
  }

  switch (SamplingRate.GetValue()) {
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

  switch (ReturnMode.GetValue()) {
  case 0:
    Sensor.ReturnMode = 55;
    break;
  case 1: // Last Return : Not implemented
    Sensor.ReturnMode = 56;
    break;
  case 2: // Dual Return : Not implemented
    Sensor.ReturnMode = 57;
    break;
  }

  // Initialize raycast vector and azimuth vector
  Sensor.AzimuthAngle.Init(90.f, this->Sensor.VerticalResolution *
                                     this->Sensor.HorizontalResolution);

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
uint8 UOusterBaseComponent::GetIntensity(const FString Surface,
                                         const float Distance) const {
  uint8 MaxReflectivity = 0;
  uint8 MinReflectivity = 0;

  if (Surface.Contains(TEXT("PM_Reflectivity_"), ESearchCase::CaseSensitive)) {
    // https://docs.unrealengine.com/5.0/en-US/API/Runtime/Core/Containers/FString/RightChop/1/
    MaxReflectivity = (uint8)FCString::Atoi(*Surface.RightChop(16));
    if (MaxReflectivity > 100) {
      MinReflectivity = 101;
    }
  } else { // Default PhysicalMaterial value, in case of the PhysicalMaterial is
           // not applied
    MaxReflectivity = 20;
  }

  return (uint8)((MinReflectivity - MaxReflectivity) /
                     (Sensor.MaxRange - Sensor.MinRange) * Distance +
                 MaxReflectivity);
}

float UOusterBaseComponent::GenerateGaussianNoise(float mean, float stdDev) {
  float u1 = FMath::RandRange(0.f, 1.f);
  float u2 = FMath::RandRange(0.f, 1.f);
  float randStdNormal =
      FMath::Sqrt(-2.0f * FMath::Loge(u1)) *
      FMath::Sin(2.0f * PI * u2); // applying Box-Muller transform
  return mean + stdDev * randStdNormal;
}

float UOusterBaseComponent::GetNoiseValue(FHitResult result) {
  uint8 intensity = 255;

  auto PhysMat = result.PhysMaterial;

  if (PhysMat != nullptr) {
    intensity = GetIntensity(*PhysMat->GetName(), (result.Distance * 2) / 10);
  }

  // normalize values
  float quality = (intensity) / 255;

  float randomNoise = GenerateGaussianNoise(0, NoiseStd);

  // the noise is dependent on distance and reflectivity

  float Noise = randomNoise * (1 - quality);

  return Noise;
}

void UOusterBaseComponent::GetScanData() {
  // complex collisions: true
  FCollisionQueryParams TraceParams =
      FCollisionQueryParams(TEXT("LaserTrace"), true, GetOwner());
  TraceParams.bReturnPhysicalMaterial = true;
  TraceParams.bTraceComplex = true;

  // Get owner's location and rotation
  FVector LidarPosition = this->GetActorLocation();
  FRotator LidarRotation = this->GetActorRotation();

  // Initialize array for raycast result
  Sensor.RecordedHits.Init(FHitResult(ForceInit),
                           Sensor.VerticalResolution *
                               Sensor.HorizontalResolution);

  // Calculate batch size for 'ParallelFor' based on workable thread
  const int ThreadNum = 1 /*FPlatformMisc::NumberOfWorkerThreadsToSpawn()*/;

  // Divide work across threads, each thread processes a portion of all hits
  // (vertically and horizontally)
  const int DivideEnd =
      FMath::FloorToInt((float)(Sensor.RecordedHits.Num() / ThreadNum));
  ParallelFor(
      ThreadNum,
      [&](int32 PFIndex) {
        // divide work acrosss threads. Each thread will process a portion of
        // the hits.
        int StartAt = PFIndex * DivideEnd;
        if (StartAt >= Sensor.RecordedHits.Num()) {
          return;
        }

        int EndAt = StartAt + DivideEnd;
        if (PFIndex == (ThreadNum - 1)) {
          EndAt = Sensor.RecordedHits.Num();
        }

        for (int32 Index = StartAt; Index < EndAt; ++Index) {
          float horizontalStepAngle = (360 / HorizontalResolution);
          float Azimuth = horizontalStepAngle *
                          FMath::FloorToInt(Index / VerticalResolution);
          float Elevation = Sensor.ElevationAngle[Index % VerticalResolution];

          FRotator LaserRotation(0.f, 0.f, 0.f);
          LaserRotation.Add(VAngle, Azimuth, 0.f);
          FRotator Rotation =
              UKismetMathLibrary::ComposeRotators(LaserRotation, LidarRotation);

          FVector BeginPoint =
              LidarPosition +
              Sensor.MinRange * UKismetMathLibrary::GetForwardVector(Rotation);
          FVector EndPoint =
              LidarPosition +
              Sensor.MaxRange * UKismetMathLibrary::GetForwardVector(Rotation);

          FHitResult result;
          GetWorld()->LineTraceSingleByChannel(result, BeginPoint, EndPoint, ECC_Visibility, TraceParams, FCollisionResponseParams::DefaultResponseParam);

          if (result.IsValidBlockingHit())
          {
            result.Distance += GetNoiseValue(result);
          }

          Sensor.RecordedHits[Index] = result;
        };
      },
      !SupportMultithread);
}

FVector UOusterBaseComponent::GetActorLocation() {
  return GetOwner()->GetActorLocation();
}

FRotator UOusterBaseComponent::GetActorRotation() {
  return GetOwner()->GetActorRotation();
}

uint32 UOusterBaseComponent::GetTimestampMicroseconds() {
  return (uint32)(fmod(GetWorld()->GetTimeSeconds(), 3600.f) *
                  1000000); // sec -> microsec
}

void UOusterBaseComponent::GenerateDataPacket(uint32 TimeStamp) 
{
  PointCloud2 ScanData;
  ScanData.header.stamp = TimeStamp;
  ScanData.header.frame_id = "Ouster";
  ScanData.height = 1;
  ScanData.width = Sensor.RecordedHits.num();
  ScanData.fields = Sensor.fields;
  ScanData.is_bigendian = false;
  ScanData.point_step = Sensor.PointStep;
  ScanData.row_step = Sensor.RecordedHits.num();
  ScanData.is_dense = true;
  ScanData.data.AddUninitialized(Sensor.RecordedHits.num()*Sensor.PointStep)
  for(int i = 0; i < Sensor.RecordedHits.num(); i++)
  {
    FVector Location = Sensor.RecordedHits[i].Location;
    PointXYZI Point;
    Point.x = Location.X;
    Point.y = Location.Y;
    Point.z = Location.Z;


    auto PhysMat = Sensor.RecordedHits[i].PhysMaterial;
    if (PhysMat != nullptr)
    {
      Point.intensity = GetIntensity(*PhysMat->GetName(), (Distance * 2) / 10);
    }
    else
    {
      Point.intensity = 0x00;
    }
    char *data[Sensor.PointStep];
    Memcpy(data, &Point, Sensor.PointStep);

    for(int j = 0; j < Sensor.PointStep; j++)
    {
      ScanData.data[i * Sensor.PointStep + j] = data[j];
    }
  }

  uint32 DataPacketSize = sizeof(ScanData) + Sensor.RecordedHits.num()*Sensor.PointStep;
  Sensor.DataPacket.AddUninitialized(DataPacketSize);
  char *dataToCopy = reinterpret_cast<char *>(&ScanData);
  Memcpy(Sensor.DataPacket, dataToCopy, DataPacketSize);
}

FString UOusterBaseComponent::DecToHex(int DecimalNumber) {
  // char array to store hexadecimal number
  char HexaDeciNum[100];

  // counter for hexadecimal number array
  int i = 0;
  while (DecimalNumber != 0) {
    // temporary variable to store remainder
    int Temp = 0;

    // storing remainder in temp variable.
    Temp = DecimalNumber % 16;

    // check if Temp < 10
    if (Temp < 10) {
      HexaDeciNum[i] = Temp + 48;
      i++;
    } else {
      HexaDeciNum[i] = Temp + 55;
      i++;
    }

    DecimalNumber = DecimalNumber / 16;
  }

  FString Answer;

  // printing hexadecimal number array in reverse order
  for (int j = i - 1; j >= 0; j--) {
    Answer += HexaDeciNum[j];
  }

  return Answer;
}

void UOusterBaseComponent::ASCIItoHEX(FString Ascii, uint8 Hex[]) {
  // Initialize final String
  FString StrHex = "";

  // Make a loop to iterate through
  // every character of ascii string
  for (int i = 0; i < Ascii.Len(); i++) {
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
  strcpy(reinterpret_cast<char *>(Hex), TCHAR_TO_ANSI(*StrHex));
}
