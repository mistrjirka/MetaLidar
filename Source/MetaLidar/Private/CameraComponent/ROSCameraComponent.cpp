#include "ROSCameraComponent/ROSCameraComponent.h"

// Sets default values for this component's properties
UROSCameraComponent::UROSCameraComponent()
{
    // Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
    // off to improve performance if you don't need them.
    PrimaryComponentTick.bCanEverTick = true;
}

// Called when the game starts
void UROSCameraComponent::BeginPlay()
{
    Super::BeginPlay();
    this->resolutionX = 1920;
    this->resolutionY = 1200;
    this->sensorSizeX = 6.6;
    this->focalLength = 4;
    this->iris = 2.8;
    this->FPS = 2;
    accumulatedTime = 0;
    this->shared_memory = std::make_unique<SharedMemory>(SHARED_MEMORY_NAME_CAMERA, sizeof(Image) + sizeof(MemoryPacket) + this->resolutionX * this->resolutionY * sizeof(FColor) * 2);
    MemoryPacket *packet = (MemoryPacket *)this->shared_memory->get_ptr();

    packet->seq = 0;
    packet->packet_size = 0;

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&(packet->mutex), &attr);

    InitializeCameraComponent();
}

uint32 UROSCameraComponent::GetTimestampMicroseconds()
{
    return (uint32)(fmod(GetWorld()->GetTimeSeconds(), 3600.f) * 1000000); // sec -> microsec
}

// Initialize Camera Component
void UROSCameraComponent::InitializeCameraComponent()
{
    AActor *Owner = GetOwner();
    this->RenderTarget = NewObject<UTextureRenderTarget2D>(this);
    this->RenderTarget->InitAutoFormat(this->resolutionX, this->resolutionY);
    this->RenderTarget->UpdateResourceImmediate();

    this->SceneCapture = NewObject<USceneCaptureComponent2D>(this);
    this->SceneCapture->RegisterComponent();
    this->SceneCapture->AttachToComponent(Owner->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
    this->SceneCapture->SetRelativeLocation(FVector(0, 0, 0));
    this->SceneCapture->SetRelativeRotation(FRotator(0, 0, 0));
    this->SceneCapture->FOVAngle = UMathToolkit::calculateHorizontalFOV(this->sensorSizeX, this->focalLength);
    this->SceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    this->SceneCapture->TextureTarget = this->RenderTarget;
    this->SceneCapture->bCaptureEveryFrame = false;
    this->SceneCapture->bCaptureOnMovement = false;
    this->SceneCapture->PostProcessSettings.DepthOfFieldSensorWidth = this->sensorSizeX;
    this->SceneCapture->PostProcessSettings.DepthOfFieldFocalDistance = this->focalLength;
    this->SceneCapture->UpdateContent();

    this->ImageData.Init(FColor(0, 0, 0, 0), this->RenderTarget->SizeX * this->RenderTarget->SizeY);
}

void UROSCameraComponent::SendImageToROS(uint32 timestamp)
{
    uint32 timestamp_sec = timestamp / 1000000;
    uint32 timestamp_nsec = (timestamp % 1000000) * 1000;

    MemoryPacket *packet = (MemoryPacket *)this->shared_memory->get_ptr();
    Image *image = (Image *)(packet->data);
    pthread_mutex_lock(&(packet->mutex));
    image->is_bigendian = false;
    image->step = this->resolutionX * sizeof(FColor);
    image->height = this->resolutionY;
    image->width = this->resolutionX;
    image->time.sec = timestamp_sec;
    image->time.nsec = timestamp_nsec;
    packet->packet_size = this->resolutionX * this->resolutionY * sizeof(uint32) + sizeof(Image);
    packet->seq++;
    UE_LOG(LogTemp, Warning, TEXT("Sending Image to ROS"));
    ParallelFor(this->resolutionY, [&](int32 y) {
        FColor *row = &this->ImageData[y * this->resolutionX];
        for (int32 x = 0; x < this->resolutionX; x++)
        {
            image->data[y * this->resolutionX + x] = row[x].R | (row[x].G << 8) | (row[x].B << 16) | (row[x].A << 24);
        }
    });
    
    //FMemory::Memcpy(image->data, this->ImageData.GetData(), this->resolutionX * this->resolutionY * sizeof(uint32));

    pthread_mutex_unlock(&(packet->mutex));
}

// Called every frame
void UROSCameraComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    accumulatedTime += DeltaTime;
    if (accumulatedTime > 1.0 / this->FPS)
    {
        // time trace 

        TRACE_CPUPROFILER_EVENT_SCOPE_STR("Capture")

        UE_LOG(LogTemp, Warning, TEXT("Capturing Image"));
        SceneCapture->CaptureScene();
        FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
        RenderTargetResource->ReadPixels(ImageData);
        AsyncTask(ENamedThreads::GameThread, [this]() {
            SendImageToROS(this->GetTimestampMicroseconds());
        });
        accumulatedTime = 0.0;
    }
}
