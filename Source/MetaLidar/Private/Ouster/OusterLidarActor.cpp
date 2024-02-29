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
  // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = false;

  LidarComponent = CreateDefaultSubobject<UOusterBaseComponent>(TEXT("OusterComponent"));
  this->AddOwnedComponent(LidarComponent);
}

// Called when the game starts or when spawned
void AOusterLidarActor::BeginPlay()
{
  Super::BeginPlay();

  if(UdpScanComponent)
  {
    ConfigureUDPScan();
    UdpScanComponent->OpenSendSocket(UdpScanComponent->Settings.SendIP, UdpScanComponent->Settings.SendPort);
    UdpScanComponent->OpenReceiveSocket(UdpScanComponent->Settings.ReceiveIP, UdpScanComponent->Settings.SendPort);
  }

  //FTimespan ThreadSleepTime = FTimespan::FromSeconds((float)(1.f / (float)LidarComponent->Sensor.SamplingRate));
  FTimespan ThreadSleepTime = FTimespan::FromSeconds(2.0f);
  UE_LOG(LogTemp, Warning, TEXT("ThreadSleepTime: %f"), ThreadSleepTime.GetTotalSeconds());
  FString UniqueThreadName = "LidarThread";

  LidarThread = new LidarThreadProcess(ThreadSleepTime,*UniqueThreadName, this);

  if(LidarThread)
  {
    LidarThread->Init();
    LidarThread->LidarThreadInit();
    UE_LOG(LogTemp, Warning, TEXT("Lidar thread initialized!"));
  }
}

void AOusterLidarActor::EndPlay(EEndPlayReason::Type Reason)
{
  if(LidarThread)
  {
    LidarThread->LidarThreadShutdown();
    LidarThread->Stop();
  }

  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //! Wait here until WorkerThread is verified as having stopped!
  //!
  //! This is a safety feature, that will delay PIE EndPlay or
  //! closing of the game while complex calcs occurring on the
  //! thread have a chance to finish
  //!
  while(!LidarThread->ThreadHasStopped())
  {
    FPlatformProcess::Sleep(0.1);
  }
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  //Do this last
  delete LidarThread;

  Super::EndPlay(Reason);
}

void AOusterLidarActor::GenerateDataPacket(TArray<uint8> &wholePacket)
{
  uint32 usablePacketSize = MAX_PACKET_SIZE - sizeof(DividedPacket);
  uint32 packetSize = wholePacket.Num();
  uint32 totalPackets = FMath::FloorToInt((float)packetSize / usablePacketSize) + 1;

  for (uint32 i = 0; i < totalPackets; i++)
  {
    uint32 packetStart = i * usablePacketSize;
    uint32 packetEnd = FMath::Min(packetStart + usablePacketSize, packetSize);
    
    DividedPacket packet;
    packet.seq = LidarComponent->PacketSeq;
    packet.packet_number = i;
    packet.total_packets = totalPackets;

    TArray<uint8> PacketData;
    PacketData.Init(0, sizeof(DividedPacket) + (packetEnd - packetStart));
    FMemory::Memcpy(PacketData.GetData(), &packet, sizeof(DividedPacket));
    FMemory::Memcpy(PacketData.GetData() + sizeof(DividedPacket), LidarComponent->Sensor.DataPacket.GetData() + packetStart, packetEnd - packetStart);
    
    this->DataToSend.Add(PacketData);
  }

}

// ! On Thread (not game thread)
// Never stop until finished calculating!
// This would be a verrrrry large hitch if done on game thread!
void AOusterLidarActor::LidarThreadTick()
{
  float TimeDiffMs = 0;

  //! Make sure to come all the way out of all function routines with this same check
  //! so as to ensure thread exits as quickly as possible, allowing game thread to finish
  //! See EndPlay for more information.
  if(LidarThread && LidarThread->IsThreadPaused())
  {
    return;
  }

  if ( BeginTimestamp == 0 )
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
  this->GenerateDataPacket(LidarComponent->Sensor.DataPacket);
  UE_LOG(LogTemp, Warning, TEXT("Packets waiting to be sent: %d"), DataToSend.Num());
  while(this->DataToSend.Num() > 0)
  {
    UE_LOG(LogTemp, Warning, TEXT("Packet acctual size: %d Packet divided %d now sending %d"), LidarComponent->Sensor.DataPacket.Num(), DataToSend.Num(), this->DataToSend[0].Num());
    UdpScanComponent->EmitBytes(this->DataToSend[0]);
    this->DataToSend.RemoveAt(0);
    FPlatformProcess::Sleep(0.01);

  }
  FPlatformProcess::Sleep(1.0);

  //UE_LOG(LogTemp, Warning, TEXT("Packet acctual size: %d Packet Expected size %d Compressed %d"), LidarComponent->Sensor.DataPacket.Num(), LidarComponent->Sensor.PacketSize, DataToSend.Num());
  //UE_LOG(LogTemp, Warning, TEXT("sending"));
  //UdpScanComponent->EmitBytes(DataToSend);
}

void AOusterLidarActor::ConfigureUDPScan()
{
  UdpScanComponent->Settings.SendIP    = LidarComponent->DestinationIP;
  UdpScanComponent->Settings.ReceiveIP = LidarComponent->SensorIP;
  UdpScanComponent->Settings.SendPort  = 11021;
  UdpScanComponent->Settings.SendSocketName = FString(TEXT("ue5-scan-send"));
  UdpScanComponent->Settings.BufferSize = 20;
}
