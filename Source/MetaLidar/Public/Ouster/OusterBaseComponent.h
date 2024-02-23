// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "OusterBaseComponent.generated.h"


class UPhysicalMaterial;
class PacketGenerationTask;

// ROS standard header file
typedef struct Header {
  uint32 seq;         // sequence ID
  double stamp;         // timestamp in seconds
  FString frame_id; // frame ID in which the data is observed
} Header;

// Data structure for a single field in PointCloud2
typedef struct PointField {
  FString name; // Name of the field (e.g., x, y, z, intensity)
  uint32 offset;  // Offset from the start of the point struct
  uint8 datatype; // Data type of the elements (e.g., FLOAT32, UINT8)
  uint32
      count; // Number of elements in the field (e.g., 1 for x, y, z; 3 for RGB)

  // Constants for data types
  static const uint8 INT8 = 1;
  static const uint8 UINT8 = 2;
  static const uint8 INT16 = 3;
  static const uint8 UINT16 = 4;
  static const uint8 INT32 = 5;
  static const uint8 UINT32 = 6;
  static const uint8 FLOAT32 = 7;
  static const uint8 FLOAT64 = 8;

  PointField(FString name, uint32 offset, uint8 datatype, uint32 count)
    : name(name), offset(offset), datatype(datatype), count(count) {}
} PointField;

// The PointCloud2 message structure
typedef struct PointCloud2 {
  Header header;   // Standard ROS message header
  uint32 height; // Height of the point cloud dataset
  uint32 width;  // Width of the point cloud dataset
  uint32 numOfFields;
  TArray<PointField>
      fields; // Describes the channels and their layout in the binary data blob
  bool is_bigendian;   // Is the data big-endian
  uint32 point_step; // Length of a point in bytes
  uint32 row_step;   // Length of a row in bytes
  TArray<uint8>
      data;      // Actual point cloud data, size is (row_step*height)
  bool is_dense; // True if there are no invalid points (NaN or Inf)
} PointCloud2;

typedef struct PointXYZI {
  float x;
  float y;
  float z;
  float intensity;
} PointXYZI;

UENUM(BlueprintType)
enum EOusterModelName {
  OS1 UMETA(DisplayName = "OS-1"),
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
  TArray<FHitResult> RecordedHits;
  TArray<float> AzimuthAngle;
  uint8 SamplingRate;
  uint8 ReturnMode;
  uint8 ModelNumber;
  uint32 PacketSize;
  float MinRange;
  float MaxRange;
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

  uint32 PacketSeq;

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

  uint32 CalculatePointStep(const TArray<PointField> &fields);
  // Function to get the size of the data type in bytes
  uint32 GetDataTypeSize(uint8 type);
};
