// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Ouster/OusterBaseComponent.h"
#include "LidarBaseActor.h"
#include "OusterLidarActor.generated.h"

#define MAX_PACKET_SIZE 65507-10
typedef struct {
  uint32 seq;
  uint16 packet_number;
  uint16 total_packets; 
  uint8 data[];
} DividedPacket;

/**
 *
 */
UCLASS()
class METALIDAR_API AOusterLidarActor : public ALidarBaseActor
{
  GENERATED_BODY()

public:
  // Sets default values for this actor's properties
  AOusterLidarActor();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MetaLidar")
  class UOusterBaseComponent* LidarComponent;

protected:
  TArray<TArray<uint8>> DataToSend;
  
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

  // Called when the game end
  virtual void EndPlay(EEndPlayReason::Type Reason) override;

  void GenerateDataPacket(TArray<uint8> &DataToSend);


public:
  /**
  * Set UDP communication parameters for scan data
  */
  virtual void ConfigureUDPScan() override;

  /**
  * Main routine
  * calculate raytracing and generate LiDAR packet data
  */
  virtual void LidarThreadTick() override;

};
