// Fill out your copyright notice in the Description page of Project Settings.

#include "Ouster/OusterLidarActor.h"
THIRD_PARTY_INCLUDES_START
#include "zlib.h"
THIRD_PARTY_INCLUDES_END

TArray<uint8> CompressData(const TArray<uint8>& InData)
{
  if (InData.Num() == 0)
  {
    return TArray<uint8>();
  }

  uLongf OutBufferSize = compressBound(InData.Num());
  TArray<uint8> OutData;
  OutData.AddUninitialized(OutBufferSize);

  if (compress2(OutData.GetData(), &OutBufferSize, InData.GetData(), InData.Num(), Z_BEST_COMPRESSION) != Z_OK)
  {
    // Handle error
  }

  OutData.SetNum(OutBufferSize);
  return OutData;
}
// Sets default values
AOusterLidarActor::AOusterLidarActor()
{
  // Set this actor to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = false;

  LidarComponent = CreateDefaultSubobject<UOusterBaseComponent>(TEXT("OusterComponent"));
  this->AddOwnedComponent(LidarComponent);
}

// Called when the game starts or when spawned
void AOusterLidarActor::BeginPlay()
{
  Super::BeginPlay();

  FTimespan ThreadSleepTime = FTimespan::FromSeconds((float)(1.f /
   (float)LidarComponent->Sensor.SamplingRate));
  //FTimespan ThreadSleepTime = FTimespan::FromSeconds(2.0f);
  UE_LOG(LogTemp, Warning, TEXT("ThreadSleepTime: %f"), ThreadSleepTime.GetTotalSeconds());
  FString UniqueThreadName = "LidarThreadOuster";

  LidarThread = new LidarThreadProcess(ThreadSleepTime, *UniqueThreadName, this);

  this->shared_memory = std::make_unique<SharedMemory>(TCHAR_TO_ANSI(*LidarComponent->Sensor.MemoryLabel), LidarComponent->Sensor.MemorySize);

  MemoryPacket* packet = (MemoryPacket*)this->shared_memory->get_ptr();
  packet->seq = 0;
  packet->packet_size = 0;

  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
  pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
  pthread_mutex_init(&(packet->mutex), &attr);

  if (LidarThread)
  {
    LidarThread->Init();
    LidarThread->LidarThreadInit();
    UE_LOG(LogTemp, Warning, TEXT("Lidar thread initialized!"));
  }
}

void AOusterLidarActor::EndPlay(EEndPlayReason::Type Reason)
{
  UE_LOG(LogTemp, Warning, TEXT("EndPlay called!"));
  if (LidarThread)
  {
    LidarThread->LidarThreadShutdown();
    LidarThread->Stop();
    pthread_mutex_consistent(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);
    pthread_mutex_unlock(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);
    pthread_mutex_destroy(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);

    UE_LOG(LogTemp, Warning, TEXT("Lidar thread stopped!"));
  }

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //! Wait here until WorkerThread is verified as having stopped!
  //!
  //! This is a safety feature, that will delay PIE EndPlay or
  //! closing of the game while complex calcs occurring on the
  //! thread have a chance to finish
  //!

  uint32 LoopCount = 0;

  while (!LidarThread->ThreadHasStopped() && LoopCount < 50)
  {
    UE_LOG(LogTemp, Warning, TEXT("Waiting for LidarThread to end..."));
    FPlatformProcess::Sleep(0.1);
    LoopCount++;
  }

  if (!LidarThread->ThreadHasStopped())
  {
    if (LidarThread->Thread)
      LidarThread->Thread->Kill();
    UE_LOG(LogTemp, Warning, TEXT("LidarThread failed to end in time!"));

    UE_LOG(LogTemp, Warning, TEXT("LidarThread failed to end in time!"));
  }
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // Do this last
  delete LidarThread;

  Super::EndPlay(Reason);
}

void AOusterLidarActor::SendDataPacket(TArray<uint8>& wholePacket)
{
  uint32 packetSize = wholePacket.Num();
  size_t newSizeToAllocate = sizeof(MemoryPacket) + packetSize;
  pthread_mutex_lock(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);
  MemoryPacket* packet = (MemoryPacket*)this->shared_memory->get_ptr();
  packet->seq++;
  packet->packet_size = packetSize;
  FMemory::Memcpy(packet->data, wholePacket.GetData(), packetSize);
  pthread_mutex_unlock(&(packet->mutex));
}

// ! On Thread (not game thread)
// Never stop until finished calculating!
// This would be a verrrrry large hitch if done on game thread!
void AOusterLidarActor::LidarThreadTick()
{
  float TimeDiffMs = 0;

  //! Make sure to come all the way out of all function routines with this same
  //! check so as to ensure thread exits as quickly as possible, allowing game
  //! thread to finish See EndPlay for more information.
  if (LidarThread && LidarThread->IsThreadPaused())
  {
    return;
  }

  if (BeginTimestamp == 0)
  {
    PacketTimestamp = 0;
  }
  else
  {
    PacketTimestamp += (uint32)(1e6 * (FPlatformTime::Seconds() - BeginTimestamp));
  }
  BeginTimestamp = FPlatformTime::Seconds();

  // Generate raycasting data
  LidarComponent->GetScanData();

  LidarComponent->GenerateDataPacket(PacketTimestamp);
  this->SendDataPacket(LidarComponent->Sensor.DataPacket);
}

void AOusterLidarActor::ConfigureUDPScan()
{
}
