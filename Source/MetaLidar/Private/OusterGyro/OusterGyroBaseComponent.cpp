#include "OusterGyro/OusterGyroBaseComponent.h"

UOusterGyroBaseComponent::UOusterGyroBaseComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  this->Sensor.Frequency = 5;
  this->Sensor.MemoryLabel = "/RbjJCakPup_OusterGyro";
  this->Sensor.MemorySize = (sizeof(OusterGyroData) + sizeof(MemoryPacket)) * 2;
  this->PacketSeq = 0;
  this->AccelerationIndex = 0;
  this->RotationIndex = 0;
}

void UOusterGyroBaseComponent::BeginPlay()
{
  Super::BeginPlay();
}

void UOusterGyroBaseComponent::InitializeSensor()
{
  Parent = nullptr;
  UE_LOG(LogTemp, Warning, TEXT("initialized gyro component!"));

  /*while ((this->CurrentWorld = this->GetWorld()) == nullptr)
  {
    UE_LOG(LogTemp, Warning, TEXT("Waiting for world to be valid!"));
    FPlatformProcess::Sleep(0.1);
  }*/

  this->Gravity = -980.f/*CurrentWorld->GetWorldSettings(false, false)->GetGravityZ()*/;
  AActor* myParent = this->GetAttachmentRootActor();
  if (myParent == nullptr)
  {
    UE_LOG(LogTemp, Warning, TEXT("Parent is null!"));
    return;
  }
  Parent = myParent->GetAttachParentActor();
}

void UOusterGyroBaseComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                             FActorComponentTickFunction* ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UOusterGyroBaseComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
}

void UOusterGyroBaseComponent::TakeSnapshot(uint32 TimeStamp)
{
  if (AccelerationIndex < ACCELERATION_BUFFER_SIZE)
  {
    //UE_LOG(LogTemp, Warning, TEXT("acceleration index: %d"), AccelerationIndex);
    FVector ActorVelocity = this->Parent->GetVelocity();
    this->AccelerationBuffer[AccelerationIndex++] =
        std::pair<FVector, uint32>(ActorVelocity, TimeStamp);
  }
  if (AccelerationIndex > ACCELERATION_BUFFER_SIZE - ROTATION_BUFFER_SIZE && RotationIndex < ROTATION_BUFFER_SIZE)
  {
    FRotator ActorRotation = this->Parent->GetActorRotation();
    FVector ActorRotationRadians;
    ActorRotationRadians.X = FMath::DegreesToRadians(ActorRotation.Pitch);
    ActorRotationRadians.Y = FMath::DegreesToRadians(ActorRotation.Yaw);
    ActorRotationRadians.Z = FMath::DegreesToRadians(ActorRotation.Roll);

    //UE_LOG(LogTemp, Warning, TEXT("Rotation: %f, %f, %f"), ActorRotationRadians.X, ActorRotationRadians.Y, ActorRotationRadians.Z);
    this->RotationBuffer[RotationIndex++] = std::pair<FVector, uint32>(ActorRotationRadians, TimeStamp);
  }
}

bool UOusterGyroBaseComponent::ReadyToProcess()
{
  return this->AccelerationIndex >= ACCELERATION_BUFFER_SIZE && this->RotationIndex >= ROTATION_BUFFER_SIZE;
}

bool UOusterGyroBaseComponent::GenerateDataPacket(uint32 TimeStamp)
{
  float TimeBetween = (TimeStamp - LastTimeStamp) / 1000000.0;

  if (TimeBetween <= 1.f/(this->Sensor.Frequency*ACCELERATION_BUFFER_SIZE))
  {
    return false;
  }

  if (TimeBetween == TimeStamp)
  {
    TimeBetween = (1.f / this->Sensor.Frequency);
  }

  LastTimeStamp = TimeStamp;
  TakeSnapshot(TimeStamp);

  if (!ReadyToProcess())
  {
    return false;
  }

  uint32_t timestamp_sec = TimeStamp / 1000000;
  uint32_t timestamp_nsec = TimeStamp * 1000;

  // Get the current location of the actor
  FVector ActorLocation = GetActorLinearAccel();
  FVector ActorRotation = GetActorRotationSpeed();
  this->Sensor.DataPacket.SetNum(sizeof(OusterGyroData));
  OusterGyroData* Data = reinterpret_cast<OusterGyroData*>(this->Sensor.DataPacket.GetData());

  Data->seq = this->PacketSeq++;
  Data->time.sec = timestamp_sec;
  Data->time.nsec = timestamp_nsec;
  Data->orientation.x = 0;
  Data->orientation.y = 0;
  Data->orientation.z = 0;
  Data->orientation.w = 0;
  Data->angular_velocity.x = ActorRotation.Z;
  Data->angular_velocity.y = ActorRotation.X;
  Data->angular_velocity.z = ActorRotation.Y;
  Data->linear_acceleration.x = ActorLocation.X;
  Data->linear_acceleration.y = ActorLocation.Y;
  Data->linear_acceleration.z = ActorLocation.Z;
  //UE_LOG(LogTemp, Warning, TEXT("Linear Accel: %f, %f, %f"), ActorLocation.X, ActorLocation.Y, ActorLocation.Z);
  //UE_LOG(LogTemp, Warning, TEXT("Angular Velocity: %f, %f, %f"), ActorRotation.Pitch, ActorRotation.Yaw, ActorRotation.Roll);
  for (int i = 0; i < 9; i++)
  {
      Data->orientation_covariance[i] = -1;
      Data->angular_velocity_covariance[i] = -1;
      Data->linear_acceleration_covariance[i] = -1;
  }
  this->RotationIndex = 0;
  this->AccelerationIndex = 0;
  return true;
}

FVector UOusterGyroBaseComponent::GetActorLinearAccel()
{

  FVector velocityThen = this->AccelerationBuffer[0].first/100;
  FVector velocityNow = this->AccelerationBuffer[1].first/100;
  

  //UE_LOG(LogTemp, Warning, TEXT("VelocityThen: %f, %f, %f"),velocityThen.X , velocityThen.Y, velocityThen.Z);
  //UE_LOG(LogTemp, Warning, TEXT("VelocityNow: %f, %f, %f"), velocityNow.X, velocityNow.Y, velocityNow.Z);

  this->CurrentRotation = this->Parent->GetActorRotation() /*this->GetComponentRotation()*/;
  double TimeBetween =
      (this->AccelerationBuffer[ACCELERATION_BUFFER_SIZE - 1].second - this->AccelerationBuffer[0].second) / 1000000.0;
  //UE_LOG(LogTemp, Warning, TEXT("TimeBetween: %f"), TimeBetween);
  //UE_LOG(LogTemp, Warning, TEXT("Velocity delta: %f, %f, %f"), (velocityNow - velocityThen).X, (velocityNow - velocityThen).Y, (velocityNow - velocityThen).Z);
  FVector Delta = (velocityNow - velocityThen) / TimeBetween;
  Delta.Z -= this->Gravity/100;
  Delta = this->CurrentRotation.UnrotateVector(Delta);
  return Delta;
}

FVector UOusterGyroBaseComponent::GetActorRotationSpeed()
{
  FVector AngularVelocity = (this->RotationBuffer[ROTATION_BUFFER_SIZE - 1].first - this->RotationBuffer[0].first);
  // UE_LOG(LogTemp, Warning, TEXT("Angular Velocity: %f, %f,"), this->RotationBuffer[ROTATION_BUFFER_SIZE - 1].first.Pitch, this->RotationBuffer[0].first.Pitch);
  
  double TimeBetween =
      (this->RotationBuffer[ROTATION_BUFFER_SIZE - 1].second - this->RotationBuffer[0].second) / 1000000.0;
  AngularVelocity.X /= TimeBetween;
  AngularVelocity.Y /= TimeBetween;
  AngularVelocity.Z /= TimeBetween;
 // AngularVelocity = AngularVelocity - this->CurrentRotation;

  return AngularVelocity;
}