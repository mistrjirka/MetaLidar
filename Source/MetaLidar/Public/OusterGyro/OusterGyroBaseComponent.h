// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "SharedStructure.h"
#include <random>
#include <sys/types.h>

#include "OusterGyroBaseComponent.generated.h"

#define ACCELERATION_BUFFER_SIZE 2
#define ROTATION_BUFFER_SIZE 2

//IMU is/will be inspired by the following link https://bitbucket.org/frostlab/holoocean-engine/src/master/Source/Holodeck/Sensors/Private/IMUSensor.cpp

class UPhysicalMaterial;
class PacketGenerationTask;

USTRUCT()
struct FOusterGyro {
  GENERATED_BODY()

  public:
  uint32 Frequency;
  FString MemoryLabel;
  uint32 MemorySize;
  TArray<uint8> DataPacket;

};

typedef struct OusterGyroData {
  Time time;
  uint32 seq;
  RQuaternion orientation;
  double orientation_covariance[9];
  RVector3 angular_velocity;
  double angular_velocity_covariance[9];
  RVector3 linear_acceleration;
  double linear_acceleration_covariance[9];
} OusterGyroData;

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class METALIDAR_API UOusterGyroBaseComponent : public USceneComponent {
  GENERATED_BODY()

public:
  // Sets default values for this component's properties
  UOusterGyroBaseComponent();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OusterGyro")
  float Frequency;
  uint32 LastTimeStamp;

  uint32 MAX_PACKET_SIZE;

  FOusterGyro Sensor;

  /**
   * Generate Ouster packet data using raycast results.
   *
   * @param TimeStamp current time of game in microseconds
   */
  bool GenerateDataPacket(uint32 TimeStamp);

  void InitializeSensor();



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
  FRotator CurrentRotation;
  std::pair<FVector, uint32> AccelerationBuffer[ACCELERATION_BUFFER_SIZE];
  std::pair<FVector, uint32> RotationBuffer[ROTATION_BUFFER_SIZE];
  uint32 AccelerationIndex;
  uint32 RotationIndex;
  


  float Gravity;
  UWorld *CurrentWorld;
  uint32 PacketSeq;

  float GenerateGaussianNoise(float mean, float stdDev);

  float GetNoiseValue(FHitResult result);

  /**
   * Get current location of Actor.
   */
  FVector GetActorLinearAccel();

  /**
   * Get current rotation of Actor.
   */
  FVector GetActorRotationSpeed();

  /**
   * Get current time of game.
   */
  uint32 GetTimestampMicroseconds();

  void TakeSnapshot(uint32 TimeStamp);

  bool ReadyToProcess();

  AActor *Parent;

};
