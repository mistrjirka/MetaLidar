// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "Kismet/KismetMathLibrary.h"
#include "OusterBaseComponent.generated.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Physics/PhysicsInterfaceCore.h"
#include <cstdint>
#include <string>
#include <vector>

class UPhysicalMaterial;
class PacketGenerationTask;

// ROS standard header file
typedef struct Header {
  uint32_t seq;         // sequence ID
  double stamp;         // timestamp in seconds
  std::string frame_id; // frame ID in which the data is observed
} Header;

// Data structure for a single field in PointCloud2
typedef struct PointField {
  std::string name; // Name of the field (e.g., x, y, z, intensity)
  uint32_t offset;  // Offset from the start of the point struct
  uint8_t datatype; // Data type of the elements (e.g., FLOAT32, UINT8)
  uint32_t
      count; // Number of elements in the field (e.g., 1 for x, y, z; 3 for RGB)

  // Constants for data types
  static const uint8_t INT8 = 1;
  static const uint8_t UINT8 = 2;
  static const uint8_t INT16 = 3;
  static const uint8_t UINT16 = 4;
  static const uint8_t INT32 = 5;
  static const uint8_t UINT32 = 6;
  static const uint8_t FLOAT32 = 7;
  static const uint8_t FLOAT64 = 8;
} PointField;

// The PointCloud2 message structure
typedef struct PointCloud2 {
  Header header;   // Standard ROS message header
  uint32_t height; // Height of the point cloud dataset
  uint32_t width;  // Width of the point cloud dataset
  std::vector<PointField>
      fields; // Describes the channels and their layout in the binary data blob
  bool is_bigendian;   // Is the data big-endian
  uint32_t point_step; // Length of a point in bytes
  uint32_t row_step;   // Length of a row in bytes
  std::vector<uint8_t>
      data;      // Actual point cloud data, size is (row_step*height)
  bool is_dense; // True if there are no invalid points (NaN or Inf)
} PointCloud2;

UENUM(BlueprintType)
enum EModelName {
  OS1 UMETA(DisplayName = "OS-1"),
};

UENUM(BlueprintType)
enum EFrequency {
  SR05 UMETA(DisplayName = "5 Hz"),
  SR10 UMETA(DisplayName = "10 Hz"),
  SR15 UMETA(DisplayName = "15 Hz"),
  SR20 UMETA(DisplayName = "20 Hz")
};

UENUM(BlueprintType)
enum ELaserReturnMode {
  Strongest UMETA(DisplayName = "Strongest"),
  // LastReturn UMETA(DisplayName = "Last Return"),
  // DualReturn UMETA(DisplayName = "Dual Return")
};

USTRUCT()
struct FOusterLidar {
  GENERATED_BODY()

public:
  uint16 VerticalResolution;
  uint16 HorizontalResolution;
  std::vector<PointField> fields;
  uint32 PointStep;
  uint32 RowStep;
  TArray<float> ElevationAngle;
  TArray<uint8> DataPacket;
  TArray<FHitResult> RecordedHits;
  TArray<float> AzimuthAngle;
  uint8 SamplingRate;
  uint8 ReturnMode;
  uint8 ModelNumber;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class METALIDAR_API UOusterBaseComponent : public UActorComponent {
  GENERATED_BODY()

public:
  // Sets default values for this component's properties
  UOusterBaseComponent();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  TEnumAsByte<EModelName> SensorModel;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  TEnumAsByte<EFrequency> SamplingRate;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ouster")
  TEnumAsByte<ELaserReturnMode> ReturnMode;

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

  uint8 GetIntensity(const FString Surface, const float Distance) const;

  /**
   * Get current location of Actor.
   */
  FVector GetActorLocation();

  /**
   * Get current rotation of Actor.
   */
  FRotator GetActorRotation();

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

  float GenerateGaussianNoise(float mean, float stdDev);

  void ConfigureOusterSensor();

  float GetNoiseValue(FHitResult result);

  uint32_t CalculatePointStep(const std::vector<PointField> &fields);
  // Function to get the size of the data type in bytes
  uint32_t GetDataTypeSize(DataType type);
};
