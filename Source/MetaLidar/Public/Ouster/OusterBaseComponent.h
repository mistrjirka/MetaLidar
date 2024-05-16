// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "SharedStructure.h"
#include <cstdint>
#include <random>

#include "OusterBaseComponent.generated.h"

//IMU is/will be inspired by the following link https://bitbucket.org/frostlab/holoocean-engine/src/master/Source/Holodeck/Sensors/Private/IMUSensor.cpp

class UPhysicalMaterial;
class PacketGenerationTask;

UENUM(BlueprintType)
enum EOusterModelName {
  OS1 UMETA(DisplayName = "OS-1"),
  OS2 UMETA(DisplayName = "OS-2"),
};

UENUM(BlueprintType)
enum EOusterFrequency {
  SRO05 UMETA(DisplayName = "5 Hz"),
  SRO10 UMETA(DisplayName = "10 Hz"),
  SRO15 UMETA(DisplayName = "15 Hz"),
  SRO20 UMETA(DisplayName = "20 Hz")
};

UENUM(BlueprintType)
enum EOusterLaserReturnMode {
  StrongestO UMETA(DisplayName = "Strongest"),
  // LastReturn UMETA(DisplayName = "Last Return"),
  // DualReturn UMETA(DisplayName = "Dual Return")
};

USTRUCT()
struct FOusterLidar {
  GENERATED_BODY()

public:
  uint16 VerticalResolution;
  uint16 HorizontalResolution;
  TArray<PointField> fields;
  uint32 PointStep;
  uint32 RowStep;
  TArray<float> ElevationAngle;
  TArray<uint8> DataPacket;
  TArray<std::pair<FHitResult, FRotator>> RecordedHits;
  FTransform Transform;
  TArray<float> AzimuthAngle;
  uint8 SamplingRate;
  uint8 ReturnMode;
  uint8 ModelNumber;
  uint32 PacketSize;
  float MinRange;
  float MaxRange;
  FString MemoryLabel;
  uint32 MemorySize;
  float ToFNoise;
  float IntensityNoise;
  float NoiseFrequency;
  float NoiseAmplitude;
  TArray<std::pair<double, double>> Noise;

};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class METALIDAR_API UOusterBaseComponent : public UActorComponent {
  GENERATED_BODY()

public:
  // Sets default values for this component's properties
  UOusterBaseComponent();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  TEnumAsByte<EOusterModelName> SensorModel;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  TEnumAsByte<EOusterFrequency> SamplingRate;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  TEnumAsByte<EOusterLaserReturnMode> ReturnMode;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  FString SensorIP;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  FString DestinationIP;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  int32 ScanPort;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  int32 PositionPort;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  float NoiseStd;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  float NoiseFrequency;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  float NoiseAmplitude;


  uint32 PacketSeq;
  uint32 MAX_PACKET_SIZE;
  float baseTime;

  FOusterLidar Sensor;

  /**
   * Get scanning data using trace.
   */
  void GetScanData();

  /**
   * Generate Ouster packet data using raycast results.
   *
   * @param TimeStamp current time of game in microseconds
   */
  void GenerateDataPacket(uint32 TimeStamp);

  uint8 GetIntensity(std::pair<FHitResult, FRotator>) const;

  /**
   * Get current location of Actor.
   */
  FVector GetActorLocation();

  /**
   * Get current rotation of Actor.
   */
  FRotator GetActorRotation();

  FVector CreateLocationNoise(const FHitResult point, uint8_t intensity);

  FVector Generate3DNoise(float stdDev);

  /**
   * Get current time of game.
   */
  uint32 GetTimestampMicroseconds();

  /**
   * Convert decimal to hexadecimal.
   */
  FString DecToHex(int DecimalNumber);

  /**
   * Convert ASCII to HEX.
   */
  void ASCIItoHEX(FString Ascii, uint8 Hex[]);

protected:
  // Called when the game starts
  virtual void BeginPlay() override;

  // Called when the game end
  virtual void EndPlay(EEndPlayReason::Type Reason) override;

public:
  // Called every frame
  virtual void
  TickComponent(float DeltaTime, ELevelTick TickType,
                FActorComponentTickFunction *ThisTickFunction) override;

private:
  bool SupportMultithread = true;
  int ThreadNumScan = 1;
  int ThreadNumPackage = 1;

  FRotator LidarRotation;
  
  float GenerateGaussianNoise(float mean, float stdDev);

  void ConfigureOusterSensor();

  uint32 CalculatePointStep(const TArray<PointField> &fields);
  // Function to get the size of the data type in bytes
  uint32 GetDataTypeSize(uint8 type);

  FRotator GetLidarRotation(float Azimuth, float Elevation, FRotator LidarRotation);

  FRotator AddRotationNoise(FRotator Rotation, float frequency, float amplitude, float azimuth, float time);

  uint8_t GetNoiseForIntensity(uint8_t intensity);
};
