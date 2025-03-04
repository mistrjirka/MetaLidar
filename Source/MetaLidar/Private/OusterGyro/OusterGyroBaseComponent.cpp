#include "OusterGyro/OusterGyroBaseComponent.h"
#include <cmath>

UOusterGyroBaseComponent::UOusterGyroBaseComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  this->Sensor.Frequency = 50;
  this->Sensor.SamplingRate = 5;
  this->Sensor.MemoryLabel = "/RbjJCakPup_OusterGyro";
  this->Sensor.MemorySize = (sizeof(OusterGyroData) + sizeof(MemoryPacket)) * 2;
  this->PacketSeq = 0;
  this->PushedElements = 0;
  this->LastTimeSnapshotStamp = 0;
  this->BeginPosition = FVector(0, 0, 0);

  FMemory::Memset(&this->OdomData, 0, sizeof(Odometry));
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
  AActor *myParent = this->GetAttachmentRootActor();
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
                                             FActorComponentTickFunction *ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UOusterGyroBaseComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  Super::EndPlay(EndPlayReason);
}

void UOusterGyroBaseComponent::TakeSnapshot(uint32 TimeStamp)
{
  SnapshotCurrentData();
  uint32 TimeStamp_sec = TimeStamp / 10e6;
  FVector ActorVelocity = GetRosVelocity();

  uint32 nextIndex = AccelerationBuffer.GetNextIndex(0);
  AccelerationBuffer[nextIndex] = TPair<FVector, uint32>(ActorVelocity, TimeStamp_sec);

  MathToolkitLibrary::calculateLinearFit(AccelerationBuffer, linear_fit_a_vel, linear_fit_b_vel, false);

  FRotator ActorRotation = GetRosCurrentRotation();
  FVector ActorRotationRadians(
      FMath::DegreesToRadians(ActorRotation.Roll),
      FMath::DegreesToRadians(ActorRotation.Pitch),
      FMath::DegreesToRadians(ActorRotation.Yaw));

  nextIndex = RotationBuffer.GetNextIndex(0);
  RotationBuffer[nextIndex] = TPair<FVector, uint32>(ActorRotationRadians, TimeStamp_sec);
  if(PushedElements < ROTATION_BUFFER_SIZE)
  {
    PushedElements++;
  }

  MathToolkitLibrary::calculateLinearFit(RotationBuffer, linear_fit_a_rot, linear_fit_b_rot, false);
}

bool UOusterGyroBaseComponent::ReadyToProcess()
{
  return PushedElements >= ACCELERATION_BUFFER_SIZE &&
         PushedElements >= ROTATION_BUFFER_SIZE;
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
void UOusterGyroBaseComponent::SnapshotCurrentData()
{
  this->CurrentVelocity = this->Parent->GetVelocity();
  this->CurrentRotation = this->Parent->GetActorRotation();
}

FVector UOusterGyroBaseComponent::GetRosVelocity()
{

  FVector ActorVelocity = this->CurrentVelocity;
  FRotator ActorRotation = this->CurrentRotation;
  ActorVelocity = ActorRotation.UnrotateVector(ActorVelocity);
  ActorVelocity = MathToolkitLibrary::ConvertUEToROS(ActorVelocity);
  return ActorVelocity;
}

FVector UOusterGyroBaseComponent::GetRosCurrentPosition()
{
  FVector ActorPosition = this->Parent->GetActorLocation() - this->BeginPosition;
  FRotator rotation = FRotator(this->BeginRotation.Pitch, this->BeginRotation.Yaw, this->BeginRotation.Roll);
  FVector UnrotatedPosition = rotation.UnrotateVector(ActorPosition);
  ActorPosition = MathToolkitLibrary::ConvertUEToROS(UnrotatedPosition);
  return ActorPosition;
}

FRotator UOusterGyroBaseComponent::GetRosCurrentRotation()
{
  FRotator ActorRotation = this->CurrentRotation;

  ActorRotation = MathToolkitLibrary::ConvertUEToROSAngleDegree(ActorRotation);
  ActorRotation.Yaw = ActorRotation.Yaw;
  return ActorRotation;
}

bool UOusterGyroBaseComponent::GenerateDataPacket(uint32 TimeStamp)
{
  double time = static_cast<double>(TimeStamp);

  float TimeBetween = (TimeStamp - LastTimeStamp) / 1000000.0;
  if (TimeBetween <= 1.f / (this->Sensor.Frequency))
  {
    //UE_LOG(LogTemp, Warning, TEXT("Time between is too small! %f it should be atleast %f"), TimeBetween, 1.f / (this->Sensor.Frequency));
    return false;
  }
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Gyro generation")
    if (this->Parent == nullptr)
    {
      //UE_LOG(LogTemp, Warning, TEXT("Parent is null!"));
      return false;
    }
    else if (!ready)
    {
      this->BeginPosition = this->Parent->GetActorLocation();
      this->BeginRotation = this->Parent->GetActorRotation();
      //UE_LOG(LogTemp, Warning, TEXT("BeginPosition: %f, %f, %f"), this->BeginPosition.X, this->BeginPosition.Y, this->BeginPosition.Z);
      //UE_LOG(LogTemp, Warning, TEXT("BeginRotation: %f, %f, %f"), this->Parent->GetActorRotation().Pitch, this->Parent->GetActorRotation().Yaw, this->Parent->GetActorRotation().Roll);
      ready = true;
    }

    SnapshotCurrentData();

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
      //UE_LOG(LogTemp, Warning, TEXT("Not ready to process!"));
      return false;
    }
    // UE_LOG(LogTemp, Warning, TEXT("Generating data packet!"));
    uint32_t timestamp_sec = TimeStamp / 1000000;
    uint32_t timestamp_nsec = TimeStamp * 1000;

    // Get the current location of the actor
    FVector ActorLocation = GetActorLinearAccel(time);
    FVector ActorRotation = GetActorRotationSpeed(time);
    this->Sensor.DataPacket.SetNum(sizeof(OusterGyroData));
    OusterGyroData *Data = reinterpret_cast<OusterGyroData *>(this->Sensor.DataPacket.GetData());
    FRotator angles = GetRosCurrentRotation();
    // UE_LOG(LogTemp, Warning, TEXT("Yaw: %f"), angles.Yaw);
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

  FVector rosPosition = GetRosCurrentPosition();
  this->Odom.pose_position.x = rosPosition.X;
  this->Odom.pose_position.y = rosPosition.Y;
  this->Odom.pose_position.z = rosPosition.Z;

  FVector ActorVelocity = GetRosVelocity();
  FVector ActorAngularVelocity = this->GetActorRotationSpeed(time);
  FQuat quat = FQuat(GetRosCurrentRotation());

  // UE_LOG(LogTemp, Warning, TEXT("Yaw: %f Position: %f, %f, %f"), GetRosCurrentRotation().Yaw, rosPosition.X, rosPosition.Y, rosPosition.Z);

  /*if(this->PacketSeq % 100 == 0)
  {
    UE_LOG(LogTemp, Warning, TEXT("Quaternion: %f, %f, %f, %f"), quat.X, quat.Y, quat.Z, quat.W);
    UE_LOG(LogTemp, Warning, TEXT("Yaw: %f"), ActorRotation.Yaw);
    UE_LOG(LogTemp, Warning, TEXT("Velocity: %f, %f, %f"), ActorRotation.Pitch, ActorRotation.Roll, ActorRotation.Yaw);
    UE_LOG(LogTemp, Warning, TEXT("Angular Velocity: %f, %f, %f"), ActorAngularVelocity.X, ActorAngularVelocity.Y, ActorAngularVelocity.Z);
  }*/

  this->Odom.pose_orientation.x = quat.X;
  this->Odom.pose_orientation.y = quat.Y;
  this->Odom.pose_orientation.z = quat.Z;
  this->Odom.pose_orientation.w = quat.W;

  for (int i = 0; i < 36; i++)
  {
    this->Odom.pose_covariance[i] = -1;
    this->Odom.twist_covariance[i] = -1;
  }
  /*
    this->Odom.twist_linear.x =0;
    this->Odom.twist_linear.y = 0;
    this->Odom.twist_linear.z = 0;
    this->Odom.twist_angular.x = 0;
    this->Odom.twist_angular.y = 0;
    this->Odom.twist_angular.z = 0;
    /*/
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

  FVector accel = (velNow - velBefore) / (1.f / this->Sensor.Frequency);
  accel.Z -= this->Gravity;

  FRotator ActorRotation = this->CurrentRotation;
  accel = ActorRotation.UnrotateVector(accel);
  accel = MathToolkitLibrary::ConvertUEToROS(accel);
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