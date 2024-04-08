#include "OusterGyro/OusterGyroBaseComponent.h"

UOusterGyroBaseComponent::UOusterGyroBaseComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  this->Sensor.Frequency = 50;
  this->Sensor.SamplingRate = 5;
  this->Sensor.MemoryLabel = "/RbjJCakPup_OusterGyro";
  this->Sensor.MemorySize = (sizeof(OusterGyroData) + sizeof(MemoryPacket)) * 2;
  this->PacketSeq = 0;
  this->LastTimeSnapshotStamp = 0;
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

  this->Gravity = -980.f /*CurrentWorld->GetWorldSettings(false, false)->GetGravityZ()*/;
  AActor* myParent = this->GetAttachmentRootActor();
  if (myParent == nullptr)
  {
    UE_LOG(LogTemp, Warning, TEXT("Parent is null!"));
    return;
  }
  Parent = myParent->GetAttachParentActor();
  if (Parent == nullptr)
  {
    UE_LOG(LogTemp, Warning, TEXT("Parent is null! 2"));
    return;
  }
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

template <typename T, size_t S>
void UOusterGyroBaseComponent::calculateLinearFit(CircularBuffer<T, S> circBuffer, size_t size, FVector& vector_fit_a,
                                                  FVector& vector_fit_b)
{
  if (circBuffer.size() < S)
  {
    return;
  }
  std::array<std::pair<FVector, uint32>, S> buffer = circBuffer.get_all();
  double sumX = 0.0, sumY[3], sumXY[3], sumX2 = 0.0;
  for (int i = 0; i < 3; i++)
  {
    sumY[i] = 0.0;
    sumXY[i] = 0.0;
  }

  for (int i = 0; i < S; i++)
  {
    sumX += buffer[i].second;
    sumY[0] += buffer[i].first.X;
    sumY[1] += buffer[i].first.Y;
    sumY[2] += buffer[i].first.Z;
    sumXY[0] += buffer[i].second * buffer[i].first.X;
    sumXY[1] += buffer[i].second * buffer[i].first.Y;
    sumXY[2] += buffer[i].second * buffer[i].first.Z;
    sumX2 += buffer[i].second * buffer[i].second;
  }
  vector_fit_a[0] = (S * sumXY[0] - sumX * sumY[0]) / (S * sumX2 - sumX * sumX);
  vector_fit_a[1] = (S * sumXY[1] - sumX * sumY[1]) / (S * sumX2 - sumX * sumX);
  vector_fit_a[2] = (S * sumXY[2] - sumX * sumY[2]) / (S * sumX2 - sumX * sumX);

  vector_fit_b[0] = (sumY[0] - vector_fit_a[0] * sumX) / S;
  vector_fit_b[1] = (sumY[1] - vector_fit_a[1] * sumX) / S;
  vector_fit_b[2] = (sumY[2] - vector_fit_a[2] * sumX) / S;
}

void UOusterGyroBaseComponent::TakeSnapshot(uint32 TimeStamp)
{
  uint32 TimeStamp_sec = TimeStamp / 10e6;
  FVector ActorVelocity = this->Parent->GetVelocity();

  this->AccelerationBuffer.put(std::pair<FVector, uint32>(ActorVelocity, TimeStamp_sec));

  this->calculateLinearFit(this->AccelerationBuffer, ACCELERATION_BUFFER_SIZE, this->linear_fit_a_vel,
                           this->linear_fit_b_vel);

  FRotator ActorRotation = this->Parent->GetActorRotation();
  FVector ActorRotationRadians;
  ActorRotationRadians.X = FMath::DegreesToRadians(ActorRotation.Pitch);
  ActorRotationRadians.Y = FMath::DegreesToRadians(ActorRotation.Yaw);
  ActorRotationRadians.Z = FMath::DegreesToRadians(ActorRotation.Roll);

  // UE_LOG(LogTemp, Warning, TEXT("Rotation: %f, %f, %f"), ActorRotationRadians.X, ActorRotationRadians.Y,
  // ActorRotationRadians.Z);
  this->RotationBuffer.put(std::pair<FVector, uint32>(ActorRotationRadians, TimeStamp_sec));
  this->calculateLinearFit(this->RotationBuffer, ROTATION_BUFFER_SIZE, this->linear_fit_a_rot, this->linear_fit_b_rot);
}

bool UOusterGyroBaseComponent::ReadyToProcess()
{
  return this->AccelerationBuffer.size() >= ACCELERATION_BUFFER_SIZE &&
         this->RotationBuffer.size() >= ROTATION_BUFFER_SIZE;
}

FVector UOusterGyroBaseComponent::getExtrapolatedVelocity(double time)
{
  FVector result;
  result.X = this->linear_fit_a_vel[0] * time + this->linear_fit_b_vel[0];
  result.Y = this->linear_fit_a_vel[1] * time + this->linear_fit_b_vel[1];
  result.Z = this->linear_fit_a_vel[2] * time + this->linear_fit_b_vel[2];
  return result;
}

FVector UOusterGyroBaseComponent::getExtrapolatedRotation(double time)
{
  FVector result;
  result.X = this->linear_fit_a_rot[0] * time + this->linear_fit_b_rot[0];
  result.Y = this->linear_fit_a_rot[1] * time + this->linear_fit_b_rot[1];
  result.Z = this->linear_fit_a_rot[2] * time + this->linear_fit_b_rot[2];
  return result;
}

bool UOusterGyroBaseComponent::GenerateDataPacket(uint32 TimeStamp)
{
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Gyro generation")
    float TimeBetween = (TimeStamp - LastTimeStamp) / 1000000.0;

    if (TimeBetween <= 1.f / (this->Sensor.Frequency))
    {
      return false;
    }
    if (this->Parent == nullptr)
    {
      UE_LOG(LogTemp, Warning, TEXT("Parent is null!"));
      return false;
    }

    if (TimeStamp - LastTimeSnapshotStamp >= 1.f / (this->Sensor.SamplingRate))
    {
      LastTimeSnapshotStamp = TimeStamp;
      TakeSnapshot(TimeStamp);
    }

    if (TimeBetween == TimeStamp)
    {
      TimeBetween = (1.f / this->Sensor.Frequency);
    }

    LastTimeStamp = TimeStamp;

    if (!ReadyToProcess())
    {
      return false;
    }
    // UE_LOG(LogTemp, Warning, TEXT("Generating data packet!"));
    uint32_t timestamp_sec = TimeStamp / 1000000;
    uint32_t timestamp_nsec = TimeStamp * 1000;

    // Get the current location of the actor
    FVector ActorLocation = GetActorLinearAccel(TimeStamp);
    FVector ActorRotation = GetActorRotationSpeed(TimeStamp);
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
    // UE_LOG(LogTemp, Warning, TEXT("Linear Accel: %f, %f, %f"), ActorLocation.X, ActorLocation.Y, ActorLocation.Z);
    // UE_LOG(LogTemp, Warning, TEXT("Angular Velocity: %f, %f, %f"), ActorRotation.Pitch, ActorRotation.Yaw,
    // ActorRotation.Roll);
    for (int i = 0; i < 9; i++)
    {
      Data->orientation_covariance[i] = -1;
      Data->angular_velocity_covariance[i] = -1;
      Data->linear_acceleration_covariance[i] = -1;
    }
  }

  return true;
}

FVector UOusterGyroBaseComponent::GetActorLinearAccel(double time)
{
  time = time / (double)10e6;
  FVector velBefore = this->getExtrapolatedVelocity(time - (1.f / this->Sensor.Frequency));
  FVector velNow = this->getExtrapolatedVelocity(time);
  FVector accel = (velNow - velBefore) / (1.f / this->Sensor.Frequency);

  this->CurrentRotation = this->Parent->GetActorRotation();
  // accel = CurrentRotation.RotateVector(accel);
  accel.Z -= this->Gravity / 100;

  return accel;
}

FVector UOusterGyroBaseComponent::GetActorRotationSpeed(double time)
{
  time = time / (double)10e6;
  FVector AngularPosPrev = this->getExtrapolatedRotation(time - (1.f / this->Sensor.Frequency));
  FVector AngularPosNow = this->getExtrapolatedRotation(time);
  FVector vel = (AngularPosNow - AngularPosPrev) / (1.f / this->Sensor.Frequency);

  // AngularVelocity = AngularVelocity - this->CurrentRotation;

  return vel;
}