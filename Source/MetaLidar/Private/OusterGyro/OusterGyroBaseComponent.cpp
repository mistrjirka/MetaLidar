#include "OusterGyro/OusterGyroBaseComponent.h"

UOusterGyroBaseComponent::UOusterGyroBaseComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  this->Sensor.Frequency = 100;
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

  this->Gravity = -980.f/*CurrentWorld->GetWorldSettings(false, false)->GetGravityZ()*/;
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

template<typename T, size_t S>
void calculateLinearFit(CircularBuffer<T, S> buffer, size_t size, FVector& vector_fit_a, FVector& vector_fit_b)
{
  if(buffer.size() < S)
  {
    return;
  }
  std::array<std::pair<FVector, uint32>, S> buffer = buffer.get_all();
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

  linear_fit_b[0] = (sumY[0] - linear_fit_a[0] * sumX) / S;
  linear_fit_b[1] = (sumY[1] - linear_fit_a[1] * sumX) / S;
  linear_fit_b[2] = (sumY[2] - linear_fit_a[2] * sumX) / S;
}


void UOusterGyroBaseComponent::TakeSnapshot(uint32 TimeStamp)
{
  FVector ActorVelocity = this->Parent->GetVelocity();

  this->AccelerationBuffer.put(std::pair<FVector, uint32>(ActorVelocity, TimeStamp/10e6));

  if(this->AccelerationBuffer.size() >= ACCELERATION_BUFFER_SIZE)
  {
    //inspired by https://www.bragitoff.com/2015/09/c-program-to-linear-fit-the-data-using-least-squares-method/
    std::array<std::pair<FVector, uint32>, ACCELERATION_BUFFER_SIZE> 
      buffer = this->AccelerationBuffer.get_all();
    double sumX = 0.0, sumY[3], sumXY[3], sumX2 = 0.0;
    for (int i = 0; i < 3; i++)
    {
      sumY[i] = 0.0;
      sumXY[i] = 0.0;
    }
    
    for (int i = 0; i < ACCELERATION_BUFFER_SIZE; i++)
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
    this->linear_fit_a[0] = (ACCELERATION_BUFFER_SIZE * sumXY[0] - sumX * sumY[0]) / (ACCELERATION_BUFFER_SIZE * sumX2 - sumX * sumX);
    this->linear_fit_a[1] = (ACCELERATION_BUFFER_SIZE * sumXY[1] - sumX * sumY[1]) / (ACCELERATION_BUFFER_SIZE * sumX2 - sumX * sumX);
    this->linear_fit_a[2] = (ACCELERATION_BUFFER_SIZE * sumXY[2] - sumX * sumY[2]) / (ACCELERATION_BUFFER_SIZE * sumX2 - sumX * sumX);

    this->linear_fit_b[0] = (sumY[0] - this->linear_fit_a[0] * sumX) / ACCELERATION_BUFFER_SIZE;
    this->linear_fit_b[1] = (sumY[1] - this->linear_fit_a[1] * sumX) / ACCELERATION_BUFFER_SIZE;
    this->linear_fit_b[2] = (sumY[2] - this->linear_fit_a[2] * sumX) / ACCELERATION_BUFFER_SIZE;

  }else{
    UE_LOG(LogTemp, Warning, TEXT("Buffer not full yet! %d"), this->AccelerationBuffer.size());
  }
  
  FRotator ActorRotation = this->Parent->GetActorRotation();
  FVector ActorRotationRadians;
  ActorRotationRadians.X = FMath::DegreesToRadians(ActorRotation.Pitch);
  ActorRotationRadians.Y = FMath::DegreesToRadians(ActorRotation.Yaw);
  ActorRotationRadians.Z = FMath::DegreesToRadians(ActorRotation.Roll);

  //UE_LOG(LogTemp, Warning, TEXT("Rotation: %f, %f, %f"), ActorRotationRadians.X, ActorRotationRadians.Y, ActorRotationRadians.Z);
  this->AccelerationBuffer.put(std::pair<FVector, uint32>(ActorRotationRadians, TimeStamp));
}

bool UOusterGyroBaseComponent::ReadyToProcess()
{
  return this->AccelerationBuffer.size() >= ACCELERATION_BUFFER_SIZE && this->AccelerationBuffer.size() >= ROTATION_BUFFER_SIZE;
}

FVector UOusterGyroBaseComponent::getExtrapolatedVelocity(double time)
{
  FVector result;
  result.X = this->linear_fit_a[0] * time + this->linear_fit_b[0];
  result.Y = this->linear_fit_a[1] * time + this->linear_fit_b[1];
  result.Z = this->linear_fit_a[2] * time + this->linear_fit_b[2];
  return result;
}


bool UOusterGyroBaseComponent::GenerateDataPacket(uint32 TimeStamp)
{
  float TimeBetween = (TimeStamp - LastTimeStamp) / 1000000.0;

  if (TimeBetween <= 1.f/(this->Sensor.Frequency))
  {
    return false;
  }
  if(this->Parent == nullptr)
  {
    UE_LOG(LogTemp, Warning, TEXT("Parent is null!"));
    return false;
  }

  if(TimeStamp-LastTimeSnapshotStamp >= 1.f/(this->Sensor.SamplingRate))
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

  uint32_t timestamp_sec = TimeStamp / 1000000;
  uint32_t timestamp_nsec = TimeStamp * 1000;

  // Get the current location of the actor
  FVector ActorLocation = GetActorLinearAccel(TimeStamp);
  FVector ActorRotation = FVector(0,0,0) /*GetActorRotationSpeed()*/;
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
  
  return true;
}

FVector UOusterGyroBaseComponent::GetActorLinearAccel(double time)
{
  time = time / (double)10e6;
  FVector velNow = this->getExtrapolatedVelocity(time);
  FVector velBefore = this->getExtrapolatedVelocity(time - (1.f / this->Sensor.Frequency));
  FVector accel = (velNow - velBefore) / (1.f / this->Sensor.Frequency);
  /*FVector accel = FVector(0,0,0);
  accel.X = this->linear_fit_a[0];
  accel.Y = this->linear_fit_a[1];
  accel.Z = this->linear_fit_a[2];*/

  this->CurrentRotation = this->Parent->GetActorRotation();
  accel.Z -= this->Gravity/100;

  //accel = this->CurrentRotation.UnrotateVector(accel);
  
  return accel;
  /*FVector velocityThen = this->AccelerationBuffer[0].first/100;
  FVector velocityNow = this->AccelerationBuffer[1].first/100;
  

  //UE_LOG(LogTemp, Warning, TEXT("VelocityThen: %f, %f, %f"),velocityThen.X , velocityThen.Y, velocityThen.Z);
  //UE_LOG(LogTemp, Warning, TEXT("VelocityNow: %f, %f, %f"), velocityNow.X, velocityNow.Y, velocityNow.Z);

  this->CurrentRotation = this->Parent->GetActorRotation() this->GetComponentRotation()*/;
  /*double TimeBetween =
      (this->AccelerationBuffer[ACCELERATION_BUFFER_SIZE - 1].second - this->AccelerationBuffer[0].second) / 1000000.0;
  //UE_LOG(LogTemp, Warning, TEXT("TimeBetween: %f"), TimeBetween);
  //UE_LOG(LogTemp, Warning, TEXT("Velocity delta: %f, %f, %f"), (velocityNow - velocityThen).X, (velocityNow - velocityThen).Y, (velocityNow - velocityThen).Z);
  FVector Delta = (velocityNow - velocityThen) / TimeBetween;
  Delta.Z -= this->Gravity/100;
  Delta = this->CurrentRotation.UnrotateVector(Delta);*/
  //return Delta;
}

FVector UOusterGyroBaseComponent::GetActorRotationSpeed()
{
  //FVector AngularVelocity = (this->RotationBuffer[ROTATION_BUFFER_SIZE - 1].first - this->RotationBuffer[0].first);
  // UE_LOG(LogTemp, Warning, TEXT("Angular Velocity: %f, %f,"), this->RotationBuffer[ROTATION_BUFFER_SIZE - 1].first.Pitch, this->RotationBuffer[0].first.Pitch);
  
/*  double TimeBetween =
      (this->RotationBuffer[ROTATION_BUFFER_SIZE - 1].second - this->RotationBuffer[0].second) / 1000000.0;
  AngularVelocity.X /= TimeBetween;
  AngularVelocity.Y /= TimeBetween;
  AngularVelocity.Z /= TimeBetween;*/
 // AngularVelocity = AngularVelocity - this->CurrentRotation;

  return FVector(0,0,0);
}