#include "OusterGyro/OusterGyroBaseComponent.h"
#include <cmath>

#define LENGTH_DIVIDER 100

UOusterGyroBaseComponent::UOusterGyroBaseComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  this->Sensor.Frequency = 50;
  this->Sensor.SamplingRate = 5;
  this->Sensor.MemoryLabel = "/RbjJCakPup_OusterGyro";
  this->Sensor.MemorySize = (sizeof(OusterGyroData) + sizeof(MemoryPacket)) * 2;
  this->PacketSeq = 0;
  this->LastTimeSnapshotStamp = 0;
  this->CurrentPosition = FVector(0, 0, 0);

  FMemory::Memset(&this->OdomData, 0, sizeof(Odometry));

  CircularBuffer<uint32, 3> testBuffer;

  testBuffer.put(1);
  testBuffer.put(2);
  testBuffer.put(3);
  if (testBuffer.size() != 3)
  {
    UE_LOG(LogTemp, Warning, TEXT("Buffer size is not 3!"));
  }
  std::array<uint32, 3> buffer = testBuffer.get_all();
  for (int i = 0; i < 3; i++)
  {
    if(buffer[i] != i+1)
    {
        UE_LOG(LogTemp, Warning, TEXT("Buffer is not correct! on index %d value should be %d actually %d"), i, i+1, buffer[i]);
    }
  }

  testBuffer.put(4);
  if (testBuffer.size() != 3)
  {
    UE_LOG(LogTemp, Warning, TEXT("Buffer size is not 3!"));
  }
  buffer = testBuffer.get_all();

  for (int i = 0; i < 3; i++)
  {
    if (buffer[i] != i + 2)
    {
      UE_LOG(LogTemp, Warning, TEXT("Buffer is not correct! on index %d value should be %d actually %d"), i, i+2, buffer[i]);
    }
  }

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
                                                  FVector& vector_fit_b, bool print)
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
    /*if(print)
    {
      UE_LOG(LogTemp, Warning, TEXT("Linear fit %d: %f, %f, %f"),i, buffer[i].first.X, buffer[i].first.Y, buffer[i].first.Z);
    }*/
    //check(std::isnan(buffer[i].first.Z) == false);
    
    sumXY[0] += buffer[i].second * buffer[i].first.X;
    sumXY[1] += buffer[i].second * buffer[i].first.Y;
    sumXY[2] += buffer[i].second * buffer[i].first.Z;
    
    sumX2 += buffer[i].second * buffer[i].second;
  }

  double denominator = S * sumX2 - sumX * sumX; 
  ////check(std::isnan(denominator) == false);
  ////check(std::isnan(sumXY[2]) == false);
  ////check(std::isnan(sumX) == false);
  ////check(std::isnan(sumY[2]) == false);
  if (denominator == 0.0 )
  {
    // Handle singular matrix case
    vector_fit_a[0] = vector_fit_a[1] = vector_fit_a[2] = 0.0;
    vector_fit_b[0] = vector_fit_b[1] = vector_fit_b[2] = 0.0;
    return;
  }

  vector_fit_a[0] = (S * sumXY[0] - sumX * sumY[0]) / denominator;
  vector_fit_a[1] = (S * sumXY[1] - sumX * sumY[1]) / denominator;
  vector_fit_a[2] = (S * sumXY[2] - sumX * sumY[2]) / denominator;
  ////check(std::isnan(vector_fit_a[2]) == false);

  vector_fit_b[0] = (sumY[0] - vector_fit_a[0] * sumX) / S;
  vector_fit_b[1] = (sumY[1] - vector_fit_a[1] * sumX) / S;
  vector_fit_b[2] = (sumY[2] - vector_fit_a[2] * sumX) / S;
  if(print)
  {
    UE_LOG(LogTemp, Warning, TEXT("Linear fit: %f, %f, %f, %f, %f, %f"), vector_fit_a[0], vector_fit_a[1], vector_fit_a[2], vector_fit_b[0], vector_fit_b[1], vector_fit_b[2]);
  }
}

void UOusterGyroBaseComponent::TakeSnapshot(uint32 TimeStamp)
{
  uint32 TimeStamp_sec = TimeStamp / 10e6;
  FVector ActorVelocity = this->Parent->GetVelocity();

  FRotator ActorRotation = this->Parent->GetActorRotation();
  ActorVelocity = ActorRotation.UnrotateVector(ActorVelocity);
  this->AccelerationBuffer.put(std::pair<FVector, uint32>(ActorVelocity, TimeStamp_sec));

  this->calculateLinearFit(this->AccelerationBuffer, ACCELERATION_BUFFER_SIZE, this->linear_fit_a_vel,
                           this->linear_fit_b_vel, false);

  FVector ActorRotationRadians;
  ActorRotationRadians.X = FMath::DegreesToRadians(ActorRotation.Roll);
  ActorRotationRadians.Y = FMath::DegreesToRadians(ActorRotation.Pitch);
  ActorRotationRadians.Z = FMath::DegreesToRadians(ActorRotation.Yaw);

  //UE_LOG(LogTemp, Warning, TEXT("Rotation: %f, %f, %f"), ActorRotationRadians.X, ActorRotationRadians.Y, ActorRotationRadians.Z);
  this->RotationBuffer.put(std::pair<FVector, uint32>(ActorRotationRadians, TimeStamp_sec));
  this->calculateLinearFit(this->RotationBuffer, ROTATION_BUFFER_SIZE, this->linear_fit_a_rot, this->linear_fit_b_rot, false);
  //check(std::isnan(this->linear_fit_a_rot[2]) == false);
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
 
  /*if(PacketSeq % 100 == 0)
  {
    UE_LOG(LogTemp, Warning, TEXT("Extrapolation parameters: %f, %f, %f, %f"), this->linear_fit_a_vel[0], this->linear_fit_a_vel[1], this->linear_fit_a_vel[2], time);
    UE_LOG(LogTemp, Warning, TEXT("Extrapolation parameters offset: %f, %f, %f, %f"), this->linear_fit_b_vel[0], this->linear_fit_b_vel[1], this->linear_fit_b_vel[2],time);
  }*/
  
  return result;
}

FVector UOusterGyroBaseComponent::getExtrapolatedRotation(double time)
{
  FVector result;
  result.X = this->linear_fit_a_rot[0] * time + this->linear_fit_b_rot[0];
  result.Y = this->linear_fit_a_rot[1] * time + this->linear_fit_b_rot[1];
  result.Z = this->linear_fit_a_rot[2] * time + this->linear_fit_b_rot[2];
  ////check(std::isnan(this->linear_fit_a_rot[2]) == false);

  /*if(PacketSeq % 100 == 0)
  {
    UE_LOG(LogTemp, Warning, TEXT("Extrapolation parameters: %f, %f, %f, %f"), this->linear_fit_a_rot[0], this->linear_fit_a_rot[1], this->linear_fit_a_rot[2], time);
    UE_LOG(LogTemp, Warning, TEXT("Extrapolation parameters offset: %f, %f, %f, %f"), this->linear_fit_b_rot[0], this->linear_fit_b_rot[1], this->linear_fit_b_rot[2],time);
  }*/
  return result;
}

bool UOusterGyroBaseComponent::GenerateDataPacket(uint32 TimeStamp)
{
  double time = static_cast<double>(TimeStamp);
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


    this->CurrentPosition = this->Parent->GetActorLocation();
    
    this->GenerateOdomData(TimeStamp);

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
    FVector ActorLocation = GetActorLinearAccel(time);
    FVector ActorRotation = GetActorRotationSpeed(time);
    this->Sensor.DataPacket.SetNum(sizeof(OusterGyroData));
    OusterGyroData* Data = reinterpret_cast<OusterGyroData*>(this->Sensor.DataPacket.GetData());
    FRotator angles = this->Parent->GetActorRotation();
    FQuat quat = FQuat(angles);

    Data->seq = this->PacketSeq++;
    Data->time.sec = timestamp_sec;
    Data->time.nsec = timestamp_nsec;
    Data->orientation.x = quat.X;
    Data->orientation.y = quat.Y;
    Data->orientation.z = quat.Z;
    Data->orientation.w = quat.W;
    Data->angular_velocity.x = ActorRotation.X;
    Data->angular_velocity.y = ActorRotation.Y;
    Data->angular_velocity.z = ActorRotation.Z;
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

void UOusterGyroBaseComponent::GenerateOdomData(uint32 TimeStamp)
{
  double time = static_cast<double>(TimeStamp);
  this->Odom.seq = this->PacketSeq;
  this->Odom.stamp.sec = time / 1000000;
  this->Odom.stamp.nsec = time * 1000;
  this->Odom.pose_position.x = this->CurrentPosition.X/LENGTH_DIVIDER;
  this->Odom.pose_position.y = this->CurrentPosition.Y/LENGTH_DIVIDER;
  this->Odom.pose_position.z = this->CurrentPosition.Z/LENGTH_DIVIDER;
  FRotator ActorRotation = this->Parent->GetActorRotation();
  FVector ActorVelocity = this->Parent->GetVelocity()/LENGTH_DIVIDER;
  ActorVelocity = ActorRotation.UnrotateVector(ActorVelocity);
  FVector ActorAngularVelocity = this->GetActorRotationSpeed(time);

  FQuat quat = FQuat(ActorRotation);

  if(this->PacketSeq % 100 == 0)
  {
    UE_LOG(LogTemp, Warning, TEXT("Quaternion: %f, %f, %f, %f"), quat.X, quat.Y, quat.Z, quat.W);
    UE_LOG(LogTemp, Warning, TEXT("Yaw: %f"), ActorRotation.Yaw);
    UE_LOG(LogTemp, Warning, TEXT("Velocity: %f, %f, %f"), ActorRotation.Pitch, ActorRotation.Roll, ActorRotation.Yaw);
    UE_LOG(LogTemp, Warning, TEXT("Angular Velocity: %f, %f, %f"), ActorAngularVelocity.X, ActorAngularVelocity.Y, ActorAngularVelocity.Z);
  }

  this->Odom.pose_orientation.x = quat.X;
  this->Odom.pose_orientation.y = quat.Y;
  this->Odom.pose_orientation.z = quat.Z;
  this->Odom.pose_orientation.w = quat.W;

  for (int i = 0; i < 36; i++)
  {
    this->Odom.pose_covariance[i] = -1;
    this->Odom.twist_covariance[i] = -1;
  }

  this->Odom.twist_linear.x = ActorVelocity.X;
  this->Odom.twist_linear.y = ActorVelocity.Y;
  this->Odom.twist_linear.z = ActorVelocity.Z;
  this->Odom.twist_angular.x = ActorAngularVelocity.X;
  this->Odom.twist_angular.y = ActorAngularVelocity.Y;
  this->Odom.twist_angular.z = ActorAngularVelocity.Z;
}

FVector UOusterGyroBaseComponent::GetActorLinearAccel(double time)
{
  time = time / (double)10e6;
  FVector velBefore = this->getExtrapolatedVelocity(time - (1.f / this->Sensor.Frequency));

  FVector velNow = this->getExtrapolatedVelocity(time);

  velBefore = velBefore / LENGTH_DIVIDER;
  velNow = velNow / LENGTH_DIVIDER;

  FVector accel = (velNow - velBefore) / (1.f / this->Sensor.Frequency);
  accel.Z -= this->Gravity / LENGTH_DIVIDER;

  this->CurrentRotation = this->Parent->GetActorRotation();
  accel = CurrentRotation.RotateVector(accel);

  return accel;
}

FVector UOusterGyroBaseComponent::GetActorRotationSpeed(double time)
{
  time = time / (double)10e6;
  FVector AngularPosPrev = this->getExtrapolatedRotation(time - (1.f / this->Sensor.Frequency));
  
  FVector AngularPosNow = this->getExtrapolatedRotation(time);
  
  
  /*if(PacketSeq % 100 == 0)
  {
    UE_LOG(LogTemp, Warning, TEXT("AngularPosPrev: %f, %f, %f"), AngularPosPrev.X, AngularPosPrev.Y, AngularPosPrev.Z);
    UE_LOG(LogTemp, Warning, TEXT("AngularPosNow: %f, %f, %f"), AngularPosNow.X, AngularPosNow.Y, AngularPosNow.Z);
  }*/
  FVector vel = (AngularPosNow - AngularPosPrev) / (1.f / this->Sensor.Frequency);

  // AngularVelocity = AngularVelocity - this->CurrentRotation;

  return vel;
}