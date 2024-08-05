#include "Ouster/OusterDepthBufferComponent.h"
#include <cstring>
#include <chrono>
using namespace std::chrono;

UOusterDepthBufferComponent::UOusterDepthBufferComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    captureReady = false;
    readySendingData.store(true);

    config.horizontalResolution = 2048;
    config.verticalResolution = 128;
    config.verticalFOV = 45.0f;
    config.frequency = 10;
    config.MemoryLabel = "/t07ySQdKFH_meta_lidar";
    config.MemorySize = 40000000;
    config.PointStep = 4 + 4 + 4 + 4;
    packetSeq = 0;
    zoffset = 30.0f;

    frequncyDelta = 1.f / config.frequency;
    cumulativeTime = 0.0f;
}

FMatrix CalculateInverseProjectionMatrix(const FMatrix &OriginalMatrix)
{
    // Calculate the inverse of the original projection matrix
    return OriginalMatrix.Inverse();
}

void UOusterDepthBufferComponent::BeginPlay()
{
    Super::BeginPlay();
    config.horizontalResolution = 1024;
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
}

void UOusterDepthBufferComponent::InitializeCaptureComponent()
{
    if (AActor *Owner = GetOwner())
    {
        int32 singleSensorResolution = fmax((config.horizontalResolution / 4) * 3.6, 100);
        int32 verticalResolution = singleSensorResolution * (config.verticalFOV / 90.0f) * 2.5;

        RenderTargetFront = CreateRenderTarget(singleSensorResolution, verticalResolution);
        RenderTargetRight = CreateRenderTarget(singleSensorResolution, verticalResolution);
        RenderTargetBack = CreateRenderTarget(singleSensorResolution, verticalResolution);
        RenderTargetLeft = CreateRenderTarget(singleSensorResolution, verticalResolution);

        Vector3Fast point(0, 0, 0, 0);
        PointCacheFront.Init(point, singleSensorResolution * verticalResolution);
        PointCacheRight.Init(point, singleSensorResolution * verticalResolution);
        PointCacheBack.Init(point, singleSensorResolution * verticalResolution);
        PointCacheLeft.Init(point, singleSensorResolution * verticalResolution);

        SceneCaptureFront = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 45.0f, 0.0f), RenderTargetFront, 92.0f);
        SceneCaptureRight = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 135.0f, 0.0f), RenderTargetRight, 92.0f);
        SceneCaptureBack = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 225.0f, 0.0f), RenderTargetBack, 92.0f);
        SceneCaptureLeft = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 315.0f, 0.0f), RenderTargetLeft, 92.0f);
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

    SceneCaptureFront->CaptureScene();
    SceneCaptureRight->CaptureScene();
    SceneCaptureBack->CaptureScene();
    SceneCaptureLeft->CaptureScene();
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
    float calculationHorizontalAngle = 0;
    if (HorizontalAngle < 45.0f || HorizontalAngle >= 315.0f)
    {
        calculationHorizontalAngle = centerAngleAroundNewZero(HorizontalAngle);
    }
    else if (HorizontalAngle < 135.0f)
    {
        calculationHorizontalAngle = HorizontalAngle - 90;
    }
    else if (HorizontalAngle < 225.0f)
    {
        calculationHorizontalAngle = HorizontalAngle - 180;
    }
    else if (HorizontalAngle < 315.0f)
    {
        calculationHorizontalAngle = HorizontalAngle - 270;
    }
    return calculationHorizontalAngle;
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

        std::pair<FVector, FVector> coords = MathToolkit::CalculateSphericalFromDepth(Depth, x, y, FOVH, RenderWidth, RenderHeight);
        spherical = coords.first;
        PointCache[(y * RenderWidth) + x] = Vector3Fast(ValidityTime, spherical.X, spherical.Y, spherical.Z);
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
            float newError = FMath::Abs(vCoord - FMath::DegreesToRadians(vertical)) + FMath::Abs(hCoord - FMath::DegreesToRadians(horizontal));

            // DrawDebugString(GetWorld(), ParentTransform.TransformPosition(result), FString::Printf(TEXT("h: %f V %f \nTh: %f Tv: %f\n E: %f"), FMath::RadiansToDegrees(hCoord), FMath::RadiansToDegrees(vCoord),horizontal, vertical, newError), nullptr, FColor::Red, (1/config.frequency)*1.5, false);
            if (newError < error)
            {
                // UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f, Target horizontal angle %f, Target vertical angle %f, Theta (h)  %f, Phi (V) %f, Distance: %f, X: %f, Y: %f, Z: %f, Error: %f"), HorizontalAngle, VerticalAngle, horizontal, vertical, FMath::RadiansToDegrees(theta), FMath::RadiansToDegrees(phi), CorrectedDistance, x, y, z, newError);
                error = newError;
                result = spherical;
                x_res = i;
                y_res = j;
            }
            else if (i > x_res && j > y_res)
            {
                run = false;
            }
        }
    }

    float r = result.X;
    float hCoord = result.Y - FMath::DegreesToRadians(horizontalOffset);
    float vCoord = -result.Z + (PI / 2.f);
    float intensity = 1.0f;

    PointXYZI point = PointXYZI(
        r * FMath::Sin(vCoord) * FMath::Cos(hCoord),
        r * FMath::Sin(vCoord) * FMath::Sin(hCoord),
        r * FMath::Cos(vCoord)+zoffset,
    intensity);
    // DrawDebugPoint(GetWorld(), ParentTransform.TransformPosition(point), 5.0f, FColor::Red, false, (1/config.frequency)*1.5);

    point = MathToolkit::ConvertUEToROS(point);

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
    if (width <= 10 && height <= 10)
    {
        // UE_LOG(LogTemp, Warning, TEXT("Depth: %d"), recursionDepth);
        // UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f, Width: %d, Height: %d, X Offset: %d, Y Offset: %d"), horizontal, vertical, width, height, x_offset, y_offset);

        return GetCoordinateToAngleAccurate(SceneCapture, RenderTarget, frameBuffer, PointCache, horizontal, vertical, width, height, horizontalOffset, x_offset, y_offset);
    }

    int32 RenderWidth = RenderTarget->SizeX;
    int32 RenderHeight = RenderTarget->SizeY;
    float FOVH = SceneCapture->FOVAngle;
    float FOVV = SceneCapture->FOVAngle * (RenderHeight / RenderWidth);

    recursionDepth++;

    // Calculate The middle of the image
    uint32 middleX = x_offset + width / 2;
    uint32 middleY = y_offset + height / 2;

    FVector spherical = CalculateSphereCoordinateCached(frameBuffer, PointCache, middleX, middleY, RenderWidth, RenderHeight, FOVH);
    float r = spherical.X;
    float hCoord = spherical.Y;
    float vCoord = spherical.Z;

    uint32 sectionCoords[2] = {0, 0};

    sectionCoords[0] = hCoord <= FMath::DegreesToRadians(horizontal);
    sectionCoords[1] = vCoord >= FMath::DegreesToRadians(vertical);

    uint32 widthMargin = width / 8;
    uint32 heightMargin = height / 8;

    uint32 x_offset_new = x_offset + sectionCoords[0] * (width / 2 - widthMargin);
    uint32 y_offset_new = y_offset + sectionCoords[1] * (height / 2 - heightMargin);
    uint32 width_new = width / 2 + widthMargin;
    uint32 height_new = height / 2 + heightMargin;

    return GetCoordinateToAngle(SceneCapture, RenderTarget, frameBuffer, PointCache, horizontal, vertical, width_new, height_new, horizontalOffset, x_offset_new, y_offset_new, recursionDepth);
}

void UOusterDepthBufferComponent::CaptureDepth()
{
    // UE_LOG(LogTemp, Warning, TEXT("CaptureDepth"));
    if (!SceneCaptureFront || !SceneCaptureRight || !SceneCaptureBack || !SceneCaptureLeft)
    {
        return;
    }

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
    int32 NumThreads = FPlatformMisc::NumberOfWorkerThreadsToSpawn();
    if (NumThreads <= 1)
    {
        NumThreads = 1; // Fallback to single-threaded execution if only one worker thread is suggested
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

                PointXYZI point = GetPixelValueFromMutltipleCaptureComponents(HorizontalAngle, AdjustedVerticalAngle);
                //UE_LOG(LogTemp, Warning, TEXT("Front: X: %f, Y: %f, Z: %f Horizontal Angle: %f, Vertical Angle: %f"), point.X, point.Y, point.Z, HorizontalAngle, VerticalAngle);
                
                PointCloud.Add(PointXYZI(point.x / 100, point.y / 100, point.z / 100, point.intensity));

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
    cumulativeTime += DeltaTime;

    if (cumulativeTime < frequncyDelta)
    {
        return;
    }

    if (!captureReady.load())
    {

        CaptureScene();
        captureReady.store(true);
        return;
    }

    if (readySendingData.load())
    {
        TRACE_CPUPROFILER_EVENT_SCOPE_STR("sending data")
        {
            TRACE_CPUPROFILER_EVENT_SCOPE_STR("Updating depth buffers")
            UpdateBuffer(RenderTargetFront, ImageDataFront);
            UpdateBuffer(RenderTargetRight, ImageDataRight);
            UpdateBuffer(RenderTargetBack, ImageDataBack);
            UpdateBuffer(RenderTargetLeft, ImageDataLeft);
        }
        readySendingData.store(false);
        AsyncTask(ENamedThreads::AnyThread, [this]()
                  {
            PointCloud.Reset();
            this->CaptureDepth();
            captureReady.store(false);   
            this->GenerateDataPacket(this->GetTimestampMicroseconds());
            readySendingData.store(true); 
            ValidityTime++; });

        cumulativeTime = 0.0f;
    }
}

PointXYZI UOusterDepthBufferComponent::GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle)
{
    if (!SceneCaptureFront || !SceneCaptureRight || !SceneCaptureBack || !SceneCaptureLeft)
    {
        return PointXYZI(0.0f, 0.0f, 0.0f, 0.0f);
    }

    int32 Width = RenderTargetFront->SizeX;
    int32 Height = RenderTargetFront->SizeY;

    float FOVH = SceneCaptureFront->FOVAngle;
    float FOVV = SceneCaptureFront->FOVAngle * (Height / Width);
    float calculationHorizontalAngle = NormalizedAngle(HorizontalAngle);

    if (HorizontalAngle < 90)
    {

        PointXYZI point = GetCoordinateToAngle(SceneCaptureFront, RenderTargetFront, ImageDataFront, PointCacheFront, calculationHorizontalAngle, VerticalAngle, Width, Height, 135);
        // debug line to the point
        // DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }
    else if (HorizontalAngle < 180.0f)
    {
        PointXYZI point = GetCoordinateToAngle(SceneCaptureRight, RenderTargetRight, ImageDataRight, PointCacheRight, calculationHorizontalAngle, VerticalAngle, Width, Height, 45);
        // debug line to the point
        // DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }
    else if (HorizontalAngle < 270.0f)
    {
        // UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f"), HorizontalAngle);
        PointXYZI point = GetCoordinateToAngle(SceneCaptureBack, RenderTargetBack, ImageDataBack, PointCacheBack, calculationHorizontalAngle, VerticalAngle, Width, Height, 315);
        // debug line to the point
        // DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }
    else
    {
        PointXYZI point = GetCoordinateToAngle(SceneCaptureLeft, RenderTargetLeft, ImageDataLeft, PointCacheLeft, calculationHorizontalAngle, VerticalAngle, Width, Height, 225);
        // debug line to the point
        // DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }

    return PointXYZI(0.0f, 0.0f, 0.0f, 0.0f);
}

float UOusterDepthBufferComponent::GetPixelFromAngle(TObjectPtr<USceneCaptureComponent2D> SceneCapture, TObjectPtr<UTextureRenderTarget2D> RenderTarget, TArray<FFloat16Color> &frameBuffer, float HorizontalAngle, float VerticalAngle)
{
    int32 Width = RenderTarget->SizeX;
    int32 Height = RenderTarget->SizeY;
    float FOVH = SceneCapture->FOVAngle;

    FIntPoint PixelCoords = UAngleToPixelUtility::GetPixelCoordinates(HorizontalAngle, VerticalAngle, FOVH, Width, Height);
    FFloat16Color PixelColor;

    if (frameBuffer.Num() <= Width * PixelCoords.Y + PixelCoords.X)
    {
        UE_LOG(LogTemp, Error, TEXT("FrameBuffer is not large enough! Coords: %d, %d, HorizontalAngle: %f, VerticalAngle: %f Size: %d"), PixelCoords.X, PixelCoords.Y, HorizontalAngle, VerticalAngle, frameBuffer.Num());
    }
    else
    {
        PixelColor = frameBuffer[PixelCoords.Y * Width + PixelCoords.X];
    }

    // Assuming the depth is stored in the red channel
    float DepthValue = PixelColor.R;
    // UE_LOG(LogTemp, Warning, TEXT("Depth value: %f, HorizontalAngle: %f, VerticalAngle: %f, PixelCoords: %d, %d RenderTargetSize: %d, %d"), DepthValue, HorizontalAngle, VerticalAngle, PixelCoords.X, PixelCoords.Y, Width, Height);
    /*if(DepthValue > 1.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("Depth value is greater than 1.0f! Value: %f"), DepthValue);
    }*/
    return DepthValue;
}

TObjectPtr<UTextureRenderTarget2D> UOusterDepthBufferComponent::CreateRenderTarget(uint32 Width, uint32 Height)
{
    TObjectPtr<UTextureRenderTarget2D> RenderTarget = NewObject<UTextureRenderTarget2D>();
    RenderTarget->InitAutoFormat(Width, Height);
    RenderTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f; // Using 32-bit float format for depth
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

        return SceneCaptureComponent;
    }
    return nullptr;
}
