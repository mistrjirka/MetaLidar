// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Kismet/KismetMathLibrary.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "SharedStructure.h"
#include "CircularBuffer/CircularBuffer.h"
#include <array>
#include <random>
#include <sys/types.h>

#include "OusterGyroBaseComponent.generated.h"

#define ACCELERATION_BUFFER_SIZE 3
#define ROTATION_BUFFER_SIZE 3

//IMU is/will be inspired by the following link https://bitbucket.org/frostlab/holoocean-engine/src/master/Source/Holodeck/Sensors/Private/IMUSensor.cpp

class UPhysicalMaterial;
class PacketGenerationTask;

USTRUCT()
struct FOusterGyro {
  GENERATED_BODY()

  public:
  uint32 Frequency;
  float SamplingRate;

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
  uint32 LastTimeStamp;
  uint32 LastTimeSnapshotStamp;
  FRotator CurrentRotation;
  CircularBuffer<std::pair<FVector, uint32>, ACCELERATION_BUFFER_SIZE> AccelerationBuffer;
  CircularBuffer<std::pair<FVector, uint32>, ROTATION_BUFFER_SIZE> RotationBuffer;
  
  FVector linear_fit_a_vel;
  FVector linear_fit_b_vel;
  FVector linear_fit_a_rot;
  FVector linear_fit_b_rot;

  float Gravity;
  UWorld *CurrentWorld;
  uint32 PacketSeq;

  float GenerateGaussianNoise(float mean, float stdDev);

  float GetNoiseValue(FHitResult result);

  FVector getExtrapolatedVelocity(double time);

  FVector getExtrapolatedRotation(double time);

  /**
   * Get current location of Actor.
   */
  FVector GetActorLinearAccel(double time);

  /**
   * Get current rotation of Actor.
   */
  FVector GetActorRotationSpeed(double time);

  /**
   * Get current time of game.
   */
  uint32 GetTimestampMicroseconds();

  template<typename T, size_t S>
  void calculateLinearFit(CircularBuffer<T, S> circBuffer, size_t size, FVector& vector_fit_a, FVector& vector_fit_b);

  void TakeSnapshot(uint32 TimeStamp);

  bool ReadyToProcess();

  AActor *Parent;

};
