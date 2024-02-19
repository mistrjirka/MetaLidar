// Fill out your copyright notice in the Description page of Project Settings.

#include "Logging/LogMacros.h"
#include "Ouster/OusterBaseComponent.h"


// Sets default values for this component's properties
UOusterBaseComponent::UOusterBaseComponent() {
  // Set this component to be initialized when the game starts, and to be ticked
  // every frame.  You can turn these features off to improve performance if you
  // don't need them.
  PrimaryComponentTick.bCanEverTick = false;

  // Check Platform supports multithreading
  SupportMultithread = FPlatformProcess::SupportsMultithreading();

  // Set-up initial values
  SensorModel = EModelName::VLP16;
  SamplingRate = EFrequency::SR10;
  ReturnMode = ELaserReturnMode::Strongest;
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
uint32_t UOusterBaseComponent::GetDataTypeSize(DataType type) {
    switch (type) {
        case INT8: return 1;
        case UINT8: return 1;
        case INT16: return 2;
        case UINT16: return 2;
        case INT32: return 4;
        case UINT32: return 4;
        case FLOAT32: return 4;
        case FLOAT64: return 8;
        default: return 0; // Should never be reached
    }
}

uint32_t UOusterBaseComponent::CalculatePointStep(const std::vector<PointField>& fields) {
    uint32_t pointStep = 0;
    for (const auto& field : fields) {
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
    Sensor.fields = {
        PointField("x", FLOAT32, 1),
        PointField("y", FLOAT32, 1),
        PointField("z", FLOAT32, 1),
        PointField("intensity", FLOAT32, 1)
    };
    Sensor.PointStep = this->CalculatePointStep(Sensor.fields);
    Sensor.RowStep = Sensor.HorizontalResolution * Sensor.PointStep; 

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
  Sensor.AzimuthAngle.Init(90.f,
                           Sensor.NumberDataBlock * Sensor.NumberDataChannel);

  // Initialize packet vector
  Sensor.DataPacket.AddUninitialized(DATA_PACKET_PAYLOAD);
  Sensor.PositionPacket.AddUninitialized(POSITION_PACKET_PAYLOAD);
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
                           Sensor.NumberDataBlock * Sensor.NumberDataChannel);
  Sensor.AzimuthAngle.Init(Sensor.AzimuthAngle[Sensor.AzimuthAngle.Num() - 1] +
                               Sensor.AzimuthResolution,
                           Sensor.NumberDataBlock * Sensor.NumberDataChannel);

  // Calculate batch size for 'ParallelFor' based on workable thread
  const int ThreadNum = FPlatformMisc::NumberOfWorkerThreadsToSpawn();
  const int DivideEnd =
      FMath::FloorToInt((float)(Sensor.RecordedHits.Num() / ThreadNum));

  ParallelFor(
      ThreadNum,
      [&](int32 PFIndex) {
        int StartAt = PFIndex * DivideEnd;
        if (StartAt >= Sensor.RecordedHits.Num()) {
          return;
        }

        int EndAt = StartAt + DivideEnd;
        if (PFIndex == (ThreadNum - 1)) {
          EndAt = Sensor.RecordedHits.Num();
        }

        for (int32 Index = StartAt; Index < EndAt; ++Index) {
          const float HAngle =
              (float)((int32)(Index / Sensor.NumberLaserEmitter) *
                      Sensor.AzimuthResolution);
          const float VAngle =
              (float)Sensor.ElevationAngle[Index % Sensor.NumberLaserEmitter];

          Sensor.AzimuthAngle[Index] += HAngle;
          Sensor.AzimuthAngle[Index] = (Sensor.AzimuthAngle[Index] > 360.0
                                            ? Sensor.AzimuthAngle[Index] - 360.f
                                            : Sensor.AzimuthAngle[Index]);

          FRotator LaserRotation(0.f, 0.f, 0.f);
          switch (Sensor.ModelNumber) {
          case 40: // VLP-32C
            LaserRotation.Add(
                VAngle,
                Sensor.AzimuthAngle[Index] +
                    Sensor.AzimuthOffset[Index % Sensor.NumberLaserEmitter],
                0.f);
            break;
          default:
            LaserRotation.Add(VAngle, Sensor.AzimuthAngle[Index], 0.f);
            break;
          }

          FRotator Rotation =
              UKismetMathLibrary::ComposeRotators(LaserRotation, LidarRotation);

          FVector BeginPoint =
              LidarPosition +
              Sensor.MinRange * UKismetMathLibrary::GetForwardVector(Rotation);
          FVector EndPoint =
              LidarPosition +
              Sensor.MaxRange * UKismetMathLibrary::GetForwardVector(Rotation);

          FHitResult result;
          GetWorld()->LineTraceSingleByChannel(
              result, BeginPoint, EndPoint, ECC_Visibility, TraceParams,
              FCollisionResponseParams::DefaultResponseParam);

          if (result.IsValidBlockingHit()) {
            result.Distance += GetNoiseValue(result);
          }

          Sensor.RecordedHits[Index] = result;
        }
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

void UOusterBaseComponent::GenerateDataPacket(uint32 TimeStamp) {
  // Packet should be encoded based on Sensor Model & Scanning Mode
  uint8 DataFlag[2];
  uint8 AzimuthData[2];
  uint8 DistanceData[2];
  uint8 IntensityData[1];
  uint8 TailData[6];

  int32 PacketIndex = 0;
  for (int32 Index = 0; Index < Sensor.RecordedHits.Num(); Index++) {

    if (Index % (Sensor.NumberDataChannel) == 0) {
      // Add data flag
      DataFlag[0] = 0xFF;
      DataFlag[1] = 0xEE;
      FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, DataFlag,
                      UE_ARRAY_COUNT(DataFlag));
      PacketIndex += UE_ARRAY_COUNT(DataFlag);

      // Azimuth data
      uint16 Azimuth = (uint16)(Sensor.AzimuthAngle[Index] * 100);
      AzimuthData[0] = Azimuth & 0x00FF;
      AzimuthData[1] = (Azimuth & 0xFF00) >> 8;

      FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, AzimuthData,
                      UE_ARRAY_COUNT(AzimuthData));
      PacketIndex += UE_ARRAY_COUNT(AzimuthData);
    }

    // Range data : converting from cm to mm
    // We should add minimum range to make distance from sensor origin
    uint16 Distance = 0;
    if (Sensor.RecordedHits[Index].bBlockingHit) {
      Distance =
          ((Sensor.RecordedHits[Index].Distance + Sensor.MinRange) * 10) /
          2; // 2mm resolution
    }
    DistanceData[0] = Distance & 0x00FF;
    DistanceData[1] = (Distance & 0xFF00) >> 8;
    FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, DistanceData,
                    UE_ARRAY_COUNT(DistanceData));
    PacketIndex += UE_ARRAY_COUNT(DistanceData);

    // Intensity data
    auto PhysMat = Sensor.RecordedHits[Index].PhysMaterial;
    if (PhysMat != nullptr) {
      IntensityData[0] = GetIntensity(*PhysMat->GetName(), (Distance * 2) / 10);
    } else {
      IntensityData[0] = 0x00;
    }
    FMemory::Memcpy(Sensor.DataPacket.GetData() + PacketIndex, IntensityData,
                    UE_ARRAY_COUNT(IntensityData));
    PacketIndex += UE_ARRAY_COUNT(IntensityData);
  }

  // Add timestamp (0 ~ 35999) and factory bytes
  TailData[0] = TimeStamp & 0x000000FF;
  TailData[1] = (TimeStamp & 0x0000FF00) >> 8;
  TailData[2] = (TimeStamp & 0x00FF0000) >> 16;
  TailData[3] = (TimeStamp & 0xFF000000) >> 24;
  TailData[4] = Sensor.ReturnMode;
  TailData[5] = Sensor.ModelNumber;
  FMemory::Memcpy(Sensor.DataPacket.GetData() + DATA_PACKET_PAYLOAD - 6,
                  TailData, UE_ARRAY_COUNT(TailData));
  PacketIndex += UE_ARRAY_COUNT(TailData);
}

void UOusterBaseComponent::GeneratePositionPacket(uint32 TimeStamp) {
  // Packet should be encoded based on Sensor Model & Scanning Mode
  uint8 UnusedPacket[187] = {0};
  uint8 ReservedPacket[178] = {0};
  uint8 NMEAPacket[128] = {0};
  uint8 TimeStampPacket[4] = {0};
  uint8 OneBytePacket[1] = {0};
  uint8 TwoBytePacket[2] = {0};
  uint8 FourBytePacket[4] = {0};

  int32 PacketIndex = 0;

  // reserved (unused)
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, UnusedPacket,
                  UE_ARRAY_COUNT(UnusedPacket));
  PacketIndex += UE_ARRAY_COUNT(UnusedPacket);

  // Temperature of top board (0 to 150°C)
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // Temperature of bottom board (0 to 150°C)
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // Temperature when ADC calibration last ran (0 to 150°C)
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // Change in temperature since last ADC calibration (-150 to 150°C)
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, TwoBytePacket,
                  UE_ARRAY_COUNT(TwoBytePacket));
  PacketIndex += UE_ARRAY_COUNT(TwoBytePacket);

  // Elapsed seconds since last ADC calibration (0 to 2^32-1)
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, FourBytePacket,
                  UE_ARRAY_COUNT(FourBytePacket));
  PacketIndex += UE_ARRAY_COUNT(FourBytePacket);

  // Reason for the last ADC calibration
  // 0: No calibration
  // 1: Power-on calibration performed
  // 2: Manual calibration performed
  // 3: Delta temperature calibration performed
  // 4: Periodic calibration performed
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // Bitmask indicating current status of ADC calibration
  // These bit fields are not mutually exclusive. The condition is True if the
  // bit field value is 1. b0: Calibration in progress b1: Delta temperature
  // limit has been met b2: Periodic time elapsed limit has been met Each status
  // bit reflects the current sensor condition but does not imply any
  // calibration occurred.
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // µsec since top of the hour
  TimeStampPacket[0] = TimeStamp & 0x000000FF;
  TimeStampPacket[1] = (TimeStamp & 0x0000FF00) >> 8;
  TimeStampPacket[2] = (TimeStamp & 0x00FF0000) >> 16;
  TimeStampPacket[3] = (TimeStamp & 0xFF000000) >> 24;
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex,
                  TimeStampPacket, UE_ARRAY_COUNT(TimeStampPacket));
  PacketIndex += UE_ARRAY_COUNT(TimeStampPacket);

  // Pulse Per Second (PPS) status
  // 0: Absent
  // 1: Synchronizing
  // 2: Locked
  // 3: Error
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // Thermal status
  // 0: Ok
  // 1: Thermal shutdown
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // Last shutdown temperature
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // Temperature of unit at power up
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, OneBytePacket,
                  UE_ARRAY_COUNT(OneBytePacket));
  PacketIndex += UE_ARRAY_COUNT(OneBytePacket);

  // NMEA sentence (GPRMC or GPGGA)
  // [To-Do] Should be implemented
  // if (EnablePositionSensor)
  // {
  //   ASCIItoHEX("$GPRMC,205948,A,3716.6694,N,12153.4550,W,000.0,078.4,260715,013.9,E,D*07",
  //   NMEAPacket);
  //   //FString Fs = FString(UTF8_TO_TCHAR(NMEAPacket));
  //   //UE_LOG(LogTemp, Warning, TEXT("%s"), *Fs);
  //   FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex,
  //   NMEAPacket, UE_ARRAY_COUNT(NMEAPacket));
  // }
  // else
  {
    FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, NMEAPacket,
                    UE_ARRAY_COUNT(NMEAPacket));
  }

  PacketIndex += UE_ARRAY_COUNT(NMEAPacket);

  // reserved
  FMemory::Memcpy(Sensor.PositionPacket.GetData() + PacketIndex, ReservedPacket,
                  UE_ARRAY_COUNT(ReservedPacket));
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
