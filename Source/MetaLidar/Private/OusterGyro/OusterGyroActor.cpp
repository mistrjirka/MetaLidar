#include "OusterGyro/OusterGyroActor.h"
// Sets default values
AOusterGyroActor::AOusterGyroActor()
{
  // Set this actor to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = false;

  GyroComponent = CreateDefaultSubobject<UOusterGyroBaseComponent>(TEXT("OusterComponent"));
  this->AddOwnedComponent(GyroComponent);
}

// Called when the game starts or when spawned
void AOusterGyroActor::BeginPlay()
{
  Super::BeginPlay();
  GyroComponent->InitializeSensor();

  FTimespan ThreadSleepTime = FTimespan::FromSeconds((float)(1.f /
   (float)(GyroComponent->Sensor.Frequency*ACCELERATION_BUFFER_SIZE)));
  //FTimespan ThreadSleepTime = FTimespan::FromSeconds(2.0f);
  UE_LOG(LogTemp, Warning, TEXT("ThreadSleepTime: %f %d"), ThreadSleepTime.GetTotalSeconds(), GyroComponent->Sensor.Frequency*ACCELERATION_BUFFER_SIZE);
  FString UniqueThreadName = "LidarThreadOusterGyro";

  LidarThread = new LidarThreadProcess(ThreadSleepTime, *UniqueThreadName, this);

  this->shared_memory = std::make_unique<SharedMemory>(TCHAR_TO_ANSI(*GyroComponent->Sensor.MemoryLabel), GyroComponent->Sensor.MemorySize);

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

void AOusterGyroActor::EndPlay(EEndPlayReason::Type Reason)
{
  UE_LOG(LogTemp, Warning, TEXT("gyro EndPlay called!"));
  pthread_mutex_consistent(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);
  pthread_mutex_unlock(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);
  pthread_mutex_destroy(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);
  /*if (LidarThread)
  {
    LidarThread->LidarThreadShutdown();
    LidarThread->Stop();

    

    UE_LOG(LogTemp, Warning, TEXT("Gyro thread stopped!"));
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
  }*/

  if (!LidarThread->ThreadHasStopped())
  {
    if (LidarThread->Thread)
    LidarThread->Thread->Kill();
    UE_LOG(LogTemp, Warning, TEXT("LidarThread failed to end in time!"));
  }
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // Do this last
  delete LidarThread;

  Super::EndPlay(Reason);
}

void AOusterGyroActor::SendDataPacket(TArray<uint8>& wholePacket)
{
  uint32 packetSize = wholePacket.Num();
  //UE_LOG(LogTemp, Warning, TEXT("packetSize: %d"), packetSize);
  size_t newSizeToAllocate = sizeof(MemoryPacket) + packetSize;
  pthread_mutex_lock(&((MemoryPacket*)this->shared_memory->get_ptr())->mutex);
  MemoryPacket* packet = (MemoryPacket*)this->shared_memory->get_ptr();
  //UE_LOG(LogTemp, Warning, TEXT("packetSize: %d"), packet->seq);

  packet->seq++;
  packet->packet_size = packetSize;
  FMemory::Memcpy(packet->data, wholePacket.GetData(), packetSize);
  pthread_mutex_unlock(&(packet->mutex));
}

// ! On Thread (not game thread)
// Never stop until finished calculating!
// This would be a verrrrry large hitch if done on game thread!
void AOusterGyroActor::LidarThreadTick()
{
  float TimeDiffMs = 0;
  PacketTimestamp = (uint32)(1e6 * (FPlatformTime::Seconds()));
  if(GyroComponent->GenerateDataPacket(PacketTimestamp))
  {
    //UE_LOG(LogTemp, Warning, TEXT("GYRO PacketTimestamp: %d"), PacketTimestamp);

    TArray<uint8> packetGyro = GyroComponent->Sensor.DataPacket;
    //convert GyroComponent->OdomData to uint8 array
    TArray<uint8> odomData = TArray<uint8>(static_cast<uint8*>(static_cast<void*>(&(GyroComponent->Odom))), sizeof(Odometry));
    packetGyro.Append(odomData);

    this->SendDataPacket(packetGyro);
  }
}