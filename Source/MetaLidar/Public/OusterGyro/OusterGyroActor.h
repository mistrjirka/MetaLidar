#pragma once

#include <pthread.h>
#include "CoreMinimal.h"
#include "OusterGyro/OusterGyroBaseComponent.h"
#include "SharedMemory/SharedMemory.h"
#include "LidarBaseActor.h"
#include "SharedStructure.h"
#include "OusterGyroActor.generated.h"

/**
 *
 */
UCLASS()
class METALIDAR_API AOusterGyroActor : public ALidarBaseActor
{
  GENERATED_BODY()

public:

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MetaLidar")
  class UOusterGyroBaseComponent* GyroComponent;
  std::unique_ptr<SharedMemory> shared_memory;

  AOusterGyroActor();
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

  // Called when the game end
  virtual void EndPlay(EEndPlayReason::Type Reason) override;

  void SendDataPacket(TArray<uint8>& DataToSend);

    /**
  * Main routine
  * calculate raytracing and generate LiDAR packet data
  */
  virtual void LidarThreadTick() override;
};