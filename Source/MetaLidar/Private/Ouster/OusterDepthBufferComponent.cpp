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

    int32 NumThreads = FPlatformMisc::NumberOfWorkerThreadsToSpawn();

    UE_LOG(LogTemp, Warning, TEXT("Number of threads: %d"), NumThreads);
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
            sensor.PointCache[0].Init(Vector3Fast(0, 0, 0, 0), singleSensorResolution * singleSensorResolution);

            sensor.PointCache[1].Init(Vector3Fast(0, 0, 0, 0), singleSensorResolution * singleSensorResolution);

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
FVector UOusterDepthBufferComponent::CalculateSphereCoordinateCached(TArray<FFloat16Color> &frameBuffer,
                                                                     TArray<Vector3Fast> &PointCache,
                                                                     uint32 x, uint32 y, uint32 RenderWidth, uint32 RenderHeight, float FOVH)
{
    FVector spherical;
    Vector3Fast cachedPoint = PointCache[(y * RenderWidth) + x];
    if (cachedPoint.validity == (uint32)ValidityTime)
    {
        spherical = FVector(cachedPoint.r, cachedPoint.horizontalAngle, cachedPoint.verticalAngle);
    }
    else
    {
        FFloat16Color color = frameBuffer[(y * RenderWidth) + x];
        float Depth = color.R;

        std::pair<FVector, FVector> coords = MathToolkitLibrary::CalculateSphericalFromDepth(Depth, x, y, FOVH, RenderWidth, RenderHeight);
        spherical = coords.first;
        PointCache[(y * RenderWidth) + x] = Vector3Fast(ValidityTime, spherical.X, spherical.Y, spherical.Z);
        // UE_LOG(LogTemp, Warning, TEXT("Depth: %f, X: %d, Y: %d, Horizontal Angle: %f, Vertical Angle: %f"), Depth, x, y, FMath::RadiansToDegrees(spherical.Y), FMath::RadiansToDegrees(spherical.Z));
    }
    return spherical;
}

PointXYZI UOusterDepthBufferComponent::GetCoordinateToAngleAccurate(
    TObjectPtr<USceneCaptureComponent2D> SceneCapture,
    TObjectPtr<UTextureRenderTarget2D> RenderTarget,
    TArray<FFloat16Color> &frameBuffer,
    TArray<Vector3Fast> &PointCache,
    float horizontal,
    float vertical,
    uint32 width,
    uint32 height,
    float horizontalOffset,
    uint32 x_offset,
    uint32 y_offset)
{
    int32 RenderWidth = RenderTarget->SizeX;
    int32 RenderHeight = RenderTarget->SizeY;
    float FOVH = SceneCapture->FOVAngle;

    // float FOVV = SceneCapture->FOVAngle * (RenderHeight / RenderWidth);

    // infinity error as beggining
    float error = std::numeric_limits<float>::infinity();
    FVector result(0, 0, 0);
    int x_res = RenderWidth;
    int y_res = RenderHeight;

    int count = 0;
    bool run = true;

    float normal = 0.0;

    for (uint32 i = x_offset; i < x_offset + width && run; i++)
    {
        // UE_LOG(LogTemp, Warning, TEXT("Horizontal angle: %f"), (i - (float)RenderWidth / 2.f)/(float)RenderWidth * (float)FOVH);
        for (uint32 j = y_offset; j < y_offset + height && run; j++)
        {
            count++;
            float r, hCoord, vCoord;
            FVector spherical;

            spherical = CalculateSphereCoordinateCached(frameBuffer, PointCache, i, j, RenderWidth, RenderHeight, FOVH);
            r = spherical.X;
            hCoord = spherical.Y;
            vCoord = spherical.Z;
            float newError = FMath::Sqrt(FMath::Square(vCoord - FMath::DegreesToRadians(vertical)) + FMath::Square(hCoord - FMath::DegreesToRadians(horizontal)));

            // DrawDebugString(GetWorld(), ParentTransform.TransformPosition(result), FString::Printf(TEXT("h: %f V %f \nTh: %f Tv: %f\n E: %f"), FMath::RadiansToDegrees(hCoord), FMath::RadiansToDegrees(vCoord),horizontal, vertical, newError), nullptr, FFloat16Color::Red, (1/config.frequency)*1.5, false);
            if (newError < error)
            {
                // UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f, Target horizontal angle %f, Target vertical angle %f, Theta (h)  %f, Phi (V) %f, Distance: %f, X: %f, Y: %f, Z: %f, Error: %f"), HorizontalAngle, VerticalAngle, horizontal, vertical, FMath::RadiansToDegrees(theta), FMath::RadiansToDegrees(phi), CorrectedDistance, x, y, z, newError);
                error = newError;
                result = spherical;
                x_res = i;
                y_res = j;
            }
        }
    }

    float r = result.X;
    uint32 range = FMath::RoundToInt(r);
    float hCoord = result.Y - FMath::DegreesToRadians(horizontalOffset + angleOffset);
    float vCoord = -result.Z + (PI / 2.f);
    float intensity = 2.0f;
    uint16_t reflectivity = 255;

    uint16_t ring = FMath::RoundToInt(((vertical + config.verticalFOV / 2.f) / config.verticalFOV) * config.verticalResolution);
    // UE_LOG(LogTemp, Warning, TEXT("Ring: %d"), ring);
    PointXYZI point = PointXYZI(
        r * FMath::Sin(vCoord) * FMath::Cos(hCoord),
        r * FMath::Sin(vCoord) * FMath::Sin(hCoord),
        r * FMath::Cos(vCoord) - zoffset,
        intensity,
        2,
        reflectivity,
        ring,
        0,
        range);
    // DrawDebugPoint(GetWorld(), ParentTransform.TransformPosition(point), 5.0f, FFloat16Color::Red, false, (1/config.frequency)*1.5);
    FVector point_coords = FVector(point.x * 100, point.y * 100, point.z * 100);
    point_coords = MathToolkitLibrary::ConvertUEToROS(point_coords);
    point.x = point_coords.X;
    point.y = point_coords.Y;
    point.z = point_coords.Z;

    return point;
}

PointXYZI UOusterDepthBufferComponent::GetCoordinateToAngle(
    TObjectPtr<USceneCaptureComponent2D> SceneCapture,
    TObjectPtr<UTextureRenderTarget2D> RenderTarget,
    TArray<FFloat16Color> &frameBuffer,
    TArray<Vector3Fast> &PointCache,
    float horizontal,
    float vertical,
    uint32 width,
    uint32 height,
    float horizontalOffset,
    uint32 x_offset,
    uint32 y_offset,
    uint32 recursionDepth)
{
    vertical += 90.0f;
    static int recursionCount = 0;
    recursionCount++;
    if(recursionCount%23 == 0)
    {
        //UE_LOG(LogTemp, Warning, TEXT("horizontal: %f, vertical: %f, x_offset: %d, y_offset: %d"), horizontal, vertical, x_offset, y_offset);
    }
    // Compute aspect and FOV values
    float AspectRatio = static_cast<float>(width) / height;
    float FOVH = FMath::DegreesToRadians(SceneCapture->FOVAngle);
    float FOVV = 2.0f * FMath::Atan(FMath::Tan(FOVH / 2.0f) / AspectRatio);

    // Convert horizontal & vertical angles from degrees to radians
    float hcoord = FMath::DegreesToRadians(horizontal);
    float vcoord = FMath::DegreesToRadians(vertical); // vcoord is the polar angle from the X-axis

    // Calculate x using the formula: x = width/2 * (1 + tan(hcoord)/tan(FOVH/2))
    // (Note: Ensure you perform the division on floating-point values before rounding)
    float xFloat = (width / 2) * (1 + (FMath::Tan(hcoord)/FMath::Tan(FOVH / 2.0f)));//(width * (FMath::Tan(FOVH / 2.0f) + FMath::Tan(hcoord))) / (2.0f * FMath::Tan(FOVH / 2.0f));
    int x = FMath::RoundToInt(xFloat);
    // For y, compute the pitch relative to the center of the screen.
    // Since your camera forward is along X, the center vertical angle corresponds to 90° (pi/2)
    // Define pitch as the deviation from the center:
    float pitch = vcoord - (PI / 2.0f);

    // Map the pitch to NDC:
    // When pitch = 0, NDC_Y = 0 (center); when pitch = FOVV/2, NDC_Y = 1 (top); when pitch = -FOVV/2, NDC_Y = -1 (bottom)
    float NDC_Y = pitch / (FOVV / 2.0f);

    // Convert NDC (range -1 to 1) to screen y coordinate (assuming y=0 is top)
    int y = FMath::RoundToInt((height/2) * (1 - (FMath::Cos(vcoord) / (FMath::Sin(vcoord)*FMath::Cos(hcoord)*FMath::Tan(FOVV/2.0f)))));//FMath::RoundToInt((1.0f - NDC_Y) * height / 2.0f);

    // Now use these x and y values to sample the depth buffer
    if (x < 0 || x >= width || y < 0 || y >= height)
    {
        //UE_LOG(LogTemp, Warning, TEXT("Out of bounds: %d, %d"), x, y);
        return PointXYZI(0, 0, 0, 0);
    }

    //UE_LOG(LogTemp, Warning, TEXT("X: %d, Y: %d"), x, y);

    float Depth = frameBuffer[(y * width) + x].R; // Depth in meters
    std::pair<FVector, FVector> coords = MathToolkitLibrary::CalculateSphericalFromDepth(Depth, x, y, SceneCapture->FOVAngle, width, height);
    FVector point = coords.second.RotateAngleAxis(-horizontalOffset, FVector(0, 0, 1));  
    float intensity = 2.0f;
    uint16_t reflectivity = 255;
    uint32 range = 20; // Convert to mm


    uint16_t ring = FMath::RoundToInt(((vertical - (PI/2) + config.verticalFOV / 2.f) / config.verticalFOV) * config.verticalResolution);
    return PointXYZI(point.X, point.Y, point.Z,  intensity,
        2,
        reflectivity,
        ring,
        0,
        range);
}

void UOusterDepthBufferComponent::CaptureDepth(uint32 CurrentBufferIndex)
{
    // UE_LOG(LogTemp, Warning, TEXT("CaptureDepth"));

    AActor *ParentActor = GetOwner();

    if (!ParentActor)
    {
        return;
    }

    FTransform ParentTransform = ParentActor->GetActorTransform();

    float HorizontalAngleStep = 360.0f / config.horizontalResolution;
    float VerticalAngleStep = config.verticalFOV / config.verticalResolution;
    float horizontalFOV = config.horizontalResolution;
    float verticalFOV = config.verticalResolution;
    uint32 HorizontalResolution = config.horizontalResolution;
    uint32 VerticalResolution = config.verticalResolution;

    // UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle Step: %f, Vertical Angle Step: %f Horizontal Resolution: %d, Vertical Resolution: %d"), HorizontalAngleStep, VerticalAngleStep, config.horizontalResolution, config.verticalResolution);
    auto start = high_resolution_clock::now();

    // Determine the number of worker threads to use
    int32 NumThreads = 2;
    if (NumThreads <= 1)
    {
        NumThreads = 2; // Fallback to single-threaded execution if only one worker thread is suggested
    }

    // create array for each thread
    //  Calculate the chunk size for each thread
    int32 ChunkSize = FMath::CeilToInt((float)HorizontalResolution / NumThreads);

    ParallelFor(NumThreads, [&](int32 ThreadIndex)
                {
        // Calculate the start and end indices for the current thread

        int32 StartIndex = ThreadIndex * ChunkSize;

        float HorizontalAngle = StartIndex * HorizontalAngleStep;
        float VerticalAngle = 0;
        int32 EndIndex = fmin(StartIndex + ChunkSize, HorizontalResolution);
        for (int32 i = StartIndex; i < EndIndex; i++)
        {
            for (int32 j = 0; j < VerticalResolution; j++)
            {
                float AdjustedVerticalAngle = VerticalAngle - config.verticalFOV / 2.0f;

                PointXYZI point = GetPixelValueFromMutltipleCaptureComponents(HorizontalAngle, AdjustedVerticalAngle, CurrentBufferIndex);
                if(point.range == 0 || point.range > 65000)
                {
                    UE_LOG(LogTemp, Warning, TEXT("Invalid point: %d"), point.range);
                    continue;
                }
                point.x /= 100.0f;
                point.y /= 100.0f;
                point.z /= 100.0f;

                //UE_LOG(LogTemp, Warning, TEXT("Front: X: %f, Y: %f, Z: %f Horizontal Angle: %f, Vertical Angle: %f"), point.X, point.Y, point.Z, HorizontalAngle, VerticalAngle);
                
                PointCloud.Add(point);

                VerticalAngle += VerticalAngleStep;
            }

            HorizontalAngle += HorizontalAngleStep;
            VerticalAngle = 0;
        } });

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);

    UE_LOG(LogTemp, Warning, TEXT("Time taken by function: %d microseconds"), duration.count());

    UE_LOG(LogTemp, Warning, TEXT("PointCloud size: %d"), PointCloud.Num());
}
uint32 UOusterDepthBufferComponent::GenerateData(uint8 *data, uint32 size, uint32 timestamp)
{
    uint32 timestamp_sec = timestamp / 1000000;
    uint32 timestamp_nsec = (timestamp % 1000000) * 1000;
    size_t packet_size = 0;
    PointCloud2Reduced *pointCloud = (PointCloud2Reduced *)data;
    pointCloud->height = 1;
    pointCloud->width = PointCloud.Num();
    pointCloud->is_bigendian = false;
    pointCloud->point_step = config.PointStep;
    pointCloud->row_step = pointCloud->width;
    pointCloud->is_dense = true;

    std::memcpy(pointCloud->data, (uint8 *)PointCloud.GetPointerUnsafe(), PointCloud.Num() * sizeof(PointXYZI));

    return sizeof(PointCloud2Reduced) + pointCloud->width * pointCloud->point_step;
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
                      PointCloud.Reset();
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

PointXYZI UOusterDepthBufferComponent::GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle, uint32 CurrentBufferIndex)
{

    float calculationHorizontalAngle = NormalizedAngle(HorizontalAngle);

    float step = 360.0f / Sensors.Num();
    int sensorIndex = FMath::Floor(HorizontalAngle / step);
    /*if(sensorIndex != 0)
    {
        return PointXYZI(0,0,0,0);
    }*/
    // UE_LOG(LogTemp, Warning, TEXT("Sensor index: %d"), sensorIndex);
    // UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f"), HorizontalAngle, VerticalAngle);
    // UE_LOG(LogTemp, Warning, TEXT("step %f"), step);

    int32 Width = Sensors[sensorIndex].RenderTarget->SizeX;
    int32 Height = Sensors[sensorIndex].RenderTarget->SizeY;
    float FOVH = Sensors[sensorIndex].SceneCapture->FOVAngle;
    float FOVV = Sensors[sensorIndex].SceneCapture->FOVAngle * (Height / Width);
    float offset = 360 - step * sensorIndex - step / 2;
    // UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Normalized: %f Vertical Angle: %f, Sensor Index: %d, Offset: %f"), calculationHorizontalAngle, HorizontalAngle, VerticalAngle, sensorIndex, offset);

    PointXYZI point = GetCoordinateToAngle(Sensors[sensorIndex].SceneCapture, Sensors[sensorIndex].RenderTarget, Sensors[sensorIndex].ImageData[CurrentBufferIndex], Sensors[sensorIndex].PointCache[CurrentBufferIndex], calculationHorizontalAngle, VerticalAngle, Width, Height, offset);
    // debug line to the point
    // DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FFloat16Color::Red, false, 1/config.frequency, 0, 1.0f);

    return point;
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
