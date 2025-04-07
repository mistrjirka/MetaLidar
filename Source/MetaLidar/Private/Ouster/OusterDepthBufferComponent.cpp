#include "Ouster/OusterDepthBufferComponent.h"
#include <cstring>
#include <chrono>
#define MAX_SENSORS 8
#define MIN_SENSORS 3
using namespace std::chrono;

UOusterDepthBufferComponent::UOusterDepthBufferComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    captureReady.store(false);
    readySendingData.store(true);
    config.horizontalResolution = 1024;
    config.verticalResolution = 128;
    config.verticalFOV = 45.0f;
    config.frequency = 10;
    config.MemoryLabel = "/t07ySQdKFH_meta_lidar";
    config.MemorySize = 40000000;
    config.PointStep = sizeof(PointXYZI);
    BufferIndex = 0;
    packetSeq = 0;
    zoffset = 30.0f;
    SensorsUpdated = 0;

    frequncyDelta = 1.f / config.frequency;
    cumulativeTime = 0.0f;
    isProcessing.store(false);
}

void UOusterDepthBufferComponent::SwitchBuffer()
{
    BufferIndex = (BufferIndex + 1) % 2;
}

FMatrix CalculateInverseProjectionMatrix(const FMatrix &OriginalMatrix)
{
    // Calculate the inverse of the original projection matrix
    return OriginalMatrix.Inverse();
}

void UOusterDepthBufferComponent::BeginPlay()
{
    Super::BeginPlay();
    angleOffset = 0.f; //-90.0f;
    config.PointStep = sizeof(PointXYZI);
    UE_LOG(LogTemp, Warning, TEXT("Point step: %d"), config.PointStep);
    config.horizontalResolution = 1024; // 1024;
    config.verticalResolution = 128;
    UE_LOG(LogTemp, Warning, TEXT("OusterDepthBufferComponent BeginPlay"));
    UE_LOG(LogTemp, Warning, TEXT("horizontal resolution %d, vertical resolution %d, vertical fov %f, frequency %d"), config.horizontalResolution, config.verticalResolution, config.verticalFOV, config.frequency);

    this->shared_memory = std::make_unique<SharedMemory>(TCHAR_TO_ANSI(*config.MemoryLabel), config.MemorySize);
    MemoryPacket *packet = (MemoryPacket *)this->shared_memory->get_ptr();
    packet->seq = 0;
    packet->packet_size = 0;
    readySendingData.store(true);
    readyCapturingData.store(false);

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&(packet->mutex), &attr);
    this->ValidityTime = 1;

    InitializeCaptureComponent();
    InitializeCache();

    int32 NumThreads = FPlatformMisc::NumberOfWorkerThreadsToSpawn();

    UE_LOG(LogTemp, Warning, TEXT("Number of threads: %d"), NumThreads);
}


void UOusterDepthBufferComponent::InitializeCache()
{
    
    PixelCache.Empty();
    PixelCache.SetNum(Sensors.Num());


    float width = Sensors[0].RenderTarget->SizeX;
    float height = Sensors[0].RenderTarget->SizeY;


    float AspectRatio = static_cast<float>(width) / height;

    FOVV = FMath::DegreesToRadians(Sensors[0].SceneCapture->FOVAngle);
    FOVH = 2.0f * FMath::Atan(FMath::Tan(FOVV / 2.0f) * AspectRatio);
    
    TanHalfFOVHRad = FMath::Tan(FOVH / 2.0f);
    TanHalfFOVVRad = FMath::Tan(FOVV / 2.0f);

    float HorizontalAngleStep = 360.0f / config.horizontalResolution;
    float VerticalAngleStep = config.verticalFOV / config.verticalResolution;
    float horizontalFOV = config.horizontalResolution;
    float verticalFOV = config.verticalResolution;
    int singleSensorHorizontalCapture = FMath::CeilToInt((float)config.horizontalResolution / Sensors.Num());
    
    for(int i = 0; i < Sensors.Num(); i++)
    {
        PointCloud.Add(TArray<PointXYZI>());
        PointCloud[i].SetNum(singleSensorHorizontalCapture * config.verticalResolution);
    }
    for (int i = 0; i < Sensors.Num(); i++)
    {
        float HorizontalAngle = 0;
        PixelCache[i].SetNum(config.horizontalResolution * config.verticalResolution);
        for(int j = 0; j < singleSensorHorizontalCapture; j++)
        {
            float VerticalAngle = - config.verticalFOV / 2.0f;
            for(int k = 0; k < config.verticalResolution; k++)
            {
                UE_LOG(LogTemp, Warning, TEXT("Horizontal angle: %f, Vertical angle: %f"), NormalizedAngle(HorizontalAngle), VerticalAngle);
                float hcoord = FMath::DegreesToRadians(NormalizedAngle(HorizontalAngle));
                float vcoord = FMath::DegreesToRadians(VerticalAngle + 90.0f ); 
                int x = FMath::RoundToInt((width / 2) * (1 + (FMath::Tan(hcoord)/FMath::Tan(FOVH / 2.0f))));
                int y = FMath::RoundToInt((height/2) * (1 - (FMath::Cos(vcoord) / (FMath::Sin(vcoord)*FMath::Cos(hcoord)*FMath::Tan(FOVV/2.0f)))));
                if(x < 0 || x >= width || y < 0 || y >= height)
                {
                    FString errorString = FString::Printf(TEXT("Out of bounds: %d, %d for angle %f, %f"), x, y, HorizontalAngle, VerticalAngle);
                    UE_LOG(LogTemp, Error, TEXT("%s"), *errorString);
                    return;

                }
                
                PixelCache[i][j * config.verticalResolution+k].x = x;
                PixelCache[i][j * config.verticalResolution+k].y = y;

                VerticalAngle += VerticalAngleStep; 
            }

            HorizontalAngle += HorizontalAngleStep;

        }
    }
}

int UOusterDepthBufferComponent::ScheduleCaptures()
{
    // check if fixed framerate is enabled
    if (!GEngine->bUseFixedFrameRate)
    {
        UE_LOG(LogTemp, Error, TEXT("Fixed frame rate is not enabled!"));
        checkf(false, TEXT("Fixed frame rate must be enabled."));
    }
    float FixedFPS = GEngine->FixedFrameRate;
    UE_LOG(LogTemp, Log, TEXT("Fixed frame rate is set to: %f"), FixedFPS);

    // calculate number of frames for one capture

    int NumberOfFramesForProcessing = FMath::Max(FMath::Floor(FixedFPS / config.frequency), 1);
    frameIndex = NumberOfFramesForProcessing;
    FramesAvailableForProcessing = NumberOfFramesForProcessing;
    UE_LOG(LogTemp, Warning, TEXT("Number of frames for processing: %d"), NumberOfFramesForProcessing);
    int NumberOfSensors = FMath::Clamp(NumberOfFramesForProcessing, MIN_SENSORS, FMath::Min(MAX_SENSORS, FMath::Floor(360 / config.verticalFOV)));
    UE_LOG(LogTemp, Warning, TEXT("Number of sensors: %d"), NumberOfSensors);
    int SensorsAdded = 0;
    int SensorsPerFrame = FMath::Max(UKismetMathLibrary::FCeil(static_cast<float>(NumberOfSensors) / NumberOfFramesForProcessing), 1);
    UE_LOG(LogTemp, Warning, TEXT("Sensors per frame: %d should be around %f"), SensorsPerFrame, static_cast<float>(NumberOfSensors) / NumberOfFramesForProcessing);
    for (int i = 0; i < NumberOfFramesForProcessing; i++)
    {
        int SensorsToAdd = FMath::Min(SensorsPerFrame, NumberOfSensors - SensorsAdded);
        ScheduledCaptures.Add(TArray<uint32>());
        for (int j = 0; j < SensorsToAdd; j++)
        {
            UE_LOG(LogTemp, Warning, TEXT("Adding sensor %d to frame %d"), SensorsAdded, i);
            ScheduledCaptures[i].Add(SensorsAdded);
            SensorsAdded++;
        }
    }
    UE_LOG(LogTemp, Warning, TEXT("Scheduled captures: %d"), ScheduledCaptures.Num());

    return NumberOfSensors;
}

void UOusterDepthBufferComponent::InitializeCaptureComponent()
{
    if (AActor *Owner = GetOwner())
    {
        int numberSensors = ScheduleCaptures();
        // if (fmod(360.f, config.verticalFOV) == 0)
        //{
        //     numberSensors = 360 / config.verticalFOV;
        // }

        float FOV = 360.f / numberSensors + 6;

        float realFOV = 360.0f / numberSensors;
        int32 singleSensorResolution = fmax((config.horizontalResolution / numberSensors) * 3.9, 100);
        UE_LOG(LogTemp, Warning, TEXT("Single sensor resolution: %d"), singleSensorResolution);

        for (int i = 0; i < numberSensors; i++)
        {
            UE_LOG(LogTemp, Warning, TEXT("Creating sensor %d"), i);
            SensorField sensor;
            sensor.RenderTarget = CreateRenderTarget(singleSensorResolution, singleSensorResolution);

            UE_LOG(LogTemp, Warning, TEXT("Sensor rotation: %f"), i * realFOV + realFOV / 2);
            UE_LOG(LogTemp, Warning, TEXT("Sensor FOV: %f"), FOV);
            sensor.SceneCapture = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, i * realFOV + realFOV / 2.f, 0.0f), sensor.RenderTarget, FOV);
            Sensors.Add(sensor);
        }
    }
}

void UOusterDepthBufferComponent::UpdateBuffer(TObjectPtr<UTextureRenderTarget2D> RenderTarget, TArray<FFloat16Color> &Image)
{
    FTextureRenderTargetResource *RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
    RenderTargetResource->ReadFloat16Pixels(Image);
}

void UOusterDepthBufferComponent::CaptureScene()
{
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Depth buffer capture screen")
    for (SensorField &sensor : Sensors)
    {
        sensor.SceneCapture->CaptureScene();
    }
}

float centerAngleAroundNewZero(float angle)
{
    float centeredAngle = angle;
    if (angle >= 180.0f)
    {
        centeredAngle = angle - 360.0f;
    }

    return centeredAngle;
}

float UOusterDepthBufferComponent::NormalizedAngle(float HorizontalAngle)
{

    return fmod(HorizontalAngle, (360.0f / Sensors.Num())) - 360.0f / Sensors.Num() / 2;
}


// Completely rewrite CaptureDepth to use the pixel cache directly
void UOusterDepthBufferComponent::CaptureDepth(uint32 CurrentBufferIndex)
{
    AActor *ParentActor = GetOwner();
    if (!ParentActor)
    {
        return;
    }

    auto start = high_resolution_clock::now();
    
    // Change parallelization to use one thread per sensor
    int32 NumSensors = Sensors.Num();
    
    // Initialize the 2D PointCloud array with one array per sensor
    for(auto& pointcloudArray : PointCloud)
    {
        pointcloudArray.Reset(PixelCache[0].Num());
    }
    
    // Process in parallel - one thread per sensor
    ParallelFor(NumSensors, [&](int32 SensorIndex)
    {
        // Get the pixel cache and image data for this sensor
        TArray<PixelCache2D>& sensorPixelCache = PixelCache[SensorIndex];
        TArray<FFloat16Color>& sensorImageData = Sensors[SensorIndex].ImageData[CurrentBufferIndex];
        TObjectPtr<UTextureRenderTarget2D> renderTarget = Sensors[SensorIndex].RenderTarget;
        TObjectPtr<USceneCaptureComponent2D> sceneCapture = Sensors[SensorIndex].SceneCapture;
        
        
        // Calculate the offset for this sensor
        float step = 360.0f / NumSensors;
        float offset = 360.0f - step * SensorIndex - step / 2.0f;
        
        // Get render target dimensions
        int32 width = renderTarget->SizeX;
        int32 height = renderTarget->SizeY;
        
        // Process all pixels for this sensor
        for (int32 pixelIndex = 0; pixelIndex < sensorPixelCache.Num(); pixelIndex++)
        {
            // Get the pre-computed pixel coordinates directly from cache
            int32 x = sensorPixelCache[pixelIndex].x;
            int32 y = sensorPixelCache[pixelIndex].y;
            
            // Calculate vertical and horizontal indices for metadata
            int32 horizontalIndex = pixelIndex / config.verticalResolution;
            int32 verticalIndex = pixelIndex % config.verticalResolution;
            
            // Ensure coordinates are valid
            if (x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            
            // Get depth value directly from image data
            float depth = sensorImageData[(y * width) + x].R;
            
            // Skip invalid depths
            if (depth == 0 || depth > 65000)
            {
                continue;
            }
            
            // Calculate 3D position from depth
            std::pair<FVector, FVector> coords = MathToolkitLibrary::CalculateSphericalFromDepth(
                depth, x, y, sceneCapture->FOVAngle, width, height);
                
            FVector point = coords.second.RotateAngleAxis(-offset, FVector(0, 0, 1));
            
            // Create the point with appropriate metadata
            PointXYZI pointData(
                point.X / 100.0f, // Scale down
                point.Y / 100.0f,
                point.Z / 100.0f,
                2.0f,  // intensity
                2,     // laser id
                255,   // reflectivity
                verticalIndex, // ring
                0,     // time
                coords.first.X // range
            );
            
            // Skip invalid ranges as additional check
            if (pointData.range == 0 || pointData.range > 65000)
            {
                continue;
            }
            
            // Add to this sensor's point collection - no locking needed
            PointCloud[SensorIndex].Add(pointData);
        }
    });
    
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    
    // Calculate total points
    int32 totalPoints = 0;
    for (const TArray<PointXYZI>& sensorPoints : PointCloud)
    {
        totalPoints += sensorPoints.Num();
    }
    
    UE_LOG(LogTemp, Warning, TEXT("Time taken by function: %d microseconds"), duration.count());
    UE_LOG(LogTemp, Warning, TEXT("Total point cloud size: %d"), totalPoints);
}

uint32 UOusterDepthBufferComponent::GenerateData(uint8 *data, uint32 size, uint32 timestamp)
{
    uint32 timestamp_sec = timestamp / 1000000;
    uint32 timestamp_nsec = (timestamp % 1000000) * 1000;
    
    // Calculate total points from all sensor arrays
    int32 totalPoints = 0;
    for (const TArray<PointXYZI>& sensorPoints : PointCloud)
    {
        totalPoints += sensorPoints.Num();
    }
    
    // Setup point cloud header
    PointCloud2Reduced *pointCloud = (PointCloud2Reduced *)data;
    pointCloud->height = 1;
    pointCloud->width = totalPoints;
    pointCloud->is_bigendian = false;
    pointCloud->point_step = config.PointStep;
    pointCloud->row_step = pointCloud->width;
    pointCloud->is_dense = true;
    
    // Copy points from each sensor array with multiple memcpy operations
    uint8* destPtr = (uint8*)pointCloud->data;
    for (const TArray<PointXYZI>& sensorPoints : PointCloud)
    {
        if (sensorPoints.Num() > 0)
        {
            size_t bytesToCopy = sensorPoints.Num() * sizeof(PointXYZI);
            std::memcpy(destPtr, (uint8*)sensorPoints.GetData(), bytesToCopy);
            destPtr += bytesToCopy;
        }
    }
    
    return sizeof(PointCloud2Reduced) + totalPoints * pointCloud->point_step;
}

void UOusterDepthBufferComponent::GenerateDataPacket(uint32 TimeStamp)
{
    MemoryPacket *packet = (MemoryPacket *)this->shared_memory->get_ptr();
    uint32 available_size = this->config.MemorySize - sizeof(MemoryPacket);
    pthread_mutex_lock(&(packet->mutex));
    packet->seq++;
    packet->packet_size = GenerateData(packet->data, available_size, TimeStamp);
    pthread_mutex_unlock(&(packet->mutex));
}

uint32 UOusterDepthBufferComponent::GetTimestampMicroseconds()
{
    return (uint32)(fmod(GetWorld()->GetTimeSeconds(), 3600.f) * 1000000); // sec -> microsec
}

void UOusterDepthBufferComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Use std::chrono for more accurate timing

    if (!captureReady.load() && frameIndex >= FramesAvailableForProcessing)
    {
        // Capture the scene for all sensors at once
        CaptureScene();
        frameIndex = 0;
        UE_LOG(LogTemp, Warning, TEXT("Capturing scene"));
        SensorsUpdated = 0;
        captureReady.store(true);
    }

    if (captureReady.load() && SensorsUpdated < Sensors.Num() && frameIndex < ScheduledCaptures.Num())
    {
        // UE_LOG(LogTemp, Warning, TEXT("Updating sensor buffers"));
        TArray<uint32> sensors = ScheduledCaptures[frameIndex];

        for (int i = 0; i < sensors.Num(); i++)
        {
            // UE_LOG(LogTemp, Warning, TEXT("Updating sensor %d"), sensors[i]);
            //  Update one sensor's buffer per frame
            UpdateBuffer(Sensors[sensors[i]].RenderTarget, Sensors[sensors[i]].ImageData[BufferIndex]);
            SensorsUpdated++;
        }
    }
    else
    {
        // UE_LOG(LogTemp, Warning, TEXT("Skipping frame"));
        //  reason for skipping frame
        if (SensorsUpdated >= Sensors.Num())
        {
            // UE_LOG(LogTemp, Warning, TEXT("All sensors updated"));
        }
        if (frameIndex >= ScheduledCaptures.Num())
        {
            // UE_LOG(LogTemp, Warning, TEXT("Frame index exceeded scheduled captures %d"), frameIndex);
        }
    }
    frameIndex++;

    if (SensorsUpdated >= Sensors.Num() && captureReady.load() && !isProcessing.load())
    {
        UE_LOG(LogTemp, Warning, TEXT("All sensors updated, processing data"));
        captureReady.store(false);

        // Track the start time of processing
        auto processStartTime = high_resolution_clock::now();
        isProcessing.store(true); // Set processing flag
        uint32 CurrentBufferIndex = BufferIndex;
        UE_LOG(LogTemp, Warning, TEXT("Current buffer index: %d"), CurrentBufferIndex);
        SwitchBuffer();
        AsyncTask(ENamedThreads::AnyThread, [this, processStartTime, CurrentBufferIndex]()
                  {
                      this->CaptureDepth(CurrentBufferIndex);
                      this->GenerateDataPacket(this->GetTimestampMicroseconds());
                      readySendingData.store(true);
                      ValidityTime++;

                      // Calculate the actual processing time
                      auto processEndTime = high_resolution_clock::now();
                      auto processDuration = duration_cast<microseconds>(processEndTime - processStartTime).count() / 1000000.0f;

                      // Subtract the elapsed frequency delta, accounting for processing time
                      cumulativeTime -= frequncyDelta;

                      // If processing took longer than the frequency delta, adjust accordingly
                      if (processDuration > frequncyDelta)
                      {
                          // Reset to slightly negative to ensure next tick will trigger immediately
                          // but still maintain proper phase for subsequent captures
                          cumulativeTime = -(processDuration - frequncyDelta);
                      }
                      isProcessing.store(false); // Reset processing flag
                  });
    }
}


TObjectPtr<UTextureRenderTarget2D> UOusterDepthBufferComponent::CreateRenderTarget(uint32 Width, uint32 Height)
{
    TObjectPtr<UTextureRenderTarget2D> RenderTarget = NewObject<UTextureRenderTarget2D>();
    RenderTarget->InitAutoFormat(Width, Height);
    RenderTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA32f; // Using 32-bit float format for depth
    return RenderTarget;
}

TObjectPtr<USceneCaptureComponent2D> UOusterDepthBufferComponent::CreateSceneCaptureComponent(FVector RelativeLocation, FRotator RelativeRotation, TObjectPtr<UTextureRenderTarget2D> RenderTarget, float FOV)
{
    if (AActor *Owner = GetOwner())
    {
        TObjectPtr<USceneCaptureComponent2D> SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(Owner);
        SceneCaptureComponent->AttachToComponent(Owner->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        SceneCaptureComponent->RegisterComponent();
        SceneCaptureComponent->TextureTarget = RenderTarget;
        SceneCaptureComponent->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        SceneCaptureComponent->FOVAngle = FOV; // Adjust the FOV
        SceneCaptureComponent->SetRelativeLocation(RelativeLocation);
        SceneCaptureComponent->SetRelativeRotation(RelativeRotation);
        SceneCaptureComponent->bCaptureEveryFrame = false; // Capture on demand
        SceneCaptureComponent->bCaptureOnMovement = false;

        SceneCaptureComponent->ShowFlags.SetDynamicShadows(false);
        SceneCaptureComponent->ShowFlags.SetPostProcessing(false);
        SceneCaptureComponent->ShowFlags.SetLighting(false);
        // SceneCaptureComponent->ShowFlags.SetAtmosphericFog(false);
        SceneCaptureComponent->ShowFlags.SetFog(false);
        SceneCaptureComponent->ShowFlags.SetAntiAliasing(false);
        SceneCaptureComponent->ShowFlags.SetAmbientOcclusion(false);
        SceneCaptureComponent->ShowFlags.SetBloom(false);
        SceneCaptureComponent->ShowFlags.SetTemporalAA(false);
        SceneCaptureComponent->ShowFlags.SetTonemapper(false);
        SceneCaptureComponent->ShowFlags.SetDepthOfField(false);
        SceneCaptureComponent->ShowFlags.SetLensFlares(false);
        SceneCaptureComponent->ShowFlags.SetScreenSpaceReflections(false);
        SceneCaptureComponent->ShowFlags.SetMotionBlur(false);
        // SceneCaptureComponent->ShowFlags.SetReflections(false);
        return SceneCaptureComponent;
    }
    return nullptr;
}
