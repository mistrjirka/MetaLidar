#include "Ouster/OusterDepthBufferComponent.h"
#include <cstring>
#include <chrono>
using namespace std::chrono;

UOusterDepthBufferComponent::UOusterDepthBufferComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    captureReady = false;
    readySendingData.store(true);

    config.horizontalResolution = 128;
    config.verticalResolution = 64;
    config.verticalFOV = 45.0f;
    config.frequency = 1;
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
    config.horizontalResolution = 2048;
    config.verticalResolution = 64;
    UE_LOG(LogTemp, Warning, TEXT("OusterDepthBufferComponent BeginPlay"));
    UE_LOG(LogTemp, Warning, TEXT("correction factor for 0,0 case %f"), CalculateDistanceCorrection(0, 0, 90, 90));
    UE_LOG(LogTemp, Warning, TEXT("horizontal resolution %d, vertical resolution %d, vertical fov %f, frequency %d"), config.horizontalResolution, config.verticalResolution, config.verticalFOV, config.frequency);

    this->shared_memory = std::make_unique<SharedMemory>(TCHAR_TO_ANSI(*config.MemoryLabel), config.MemorySize);
    MemoryPacket *packet = (MemoryPacket *)this->shared_memory->get_ptr();
    packet->seq = 0;
    packet->packet_size = 0;
    readySendingData.store(true);

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&(packet->mutex), &attr);

    InitializeCaptureComponent();
}

void UOusterDepthBufferComponent::InitializeCaptureComponent()
{
    if (AActor *Owner = GetOwner())
    {
        int32 singleSensorResolution = (config.horizontalResolution / 4) * 4;
        int32 verticalResolution = singleSensorResolution * (config.verticalFOV / 90.0f)*1.5;
        
        RenderTargetFront = CreateRenderTarget(singleSensorResolution, singleSensorResolution);
        RenderTargetRight = CreateRenderTarget(singleSensorResolution, singleSensorResolution);
        RenderTargetBack = CreateRenderTarget(singleSensorResolution, singleSensorResolution);
        RenderTargetLeft = CreateRenderTarget(singleSensorResolution, singleSensorResolution);

        SceneCaptureFront = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 45.0f, 0.0f), RenderTargetFront, 90.0f);
        SceneCaptureRight = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 135.0f, 0.0f), RenderTargetRight, 90.0f);
        SceneCaptureBack = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 225.0f, 0.0f), RenderTargetBack, 90.0f);
        SceneCaptureLeft = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 315.0f, 0.0f), RenderTargetLeft, 90.0f);
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

float UOusterDepthBufferComponent::CalculateDistanceCorrection(float HorizontalAngle, float VerticalAngle, float FOVH, float FOVV)
{
    // Convert angles to radians
    float HorizontalAngleRad = FMath::DegreesToRadians(HorizontalAngle);
    float VerticalAngleRad = FMath::DegreesToRadians(VerticalAngle);

    // Calculate FOV in radians
    float FOVHRad = FMath::DegreesToRadians(FOVH);
    float FOVVRad = FMath::DegreesToRadians(FOVV);

    // Calculate the correction factor based on the angles
    float CorrectionFactorH = FMath::Cos(HorizontalAngleRad) / FMath::Cos(FOVHRad / 2.0f);
    float CorrectionFactorV = FMath::Cos(VerticalAngleRad) / FMath::Cos(FOVVRad / 2.0f);

    // Combine the horizontal and vertical correction factors
    return CorrectionFactorH * CorrectionFactorV;
}

float CalculateZAxisCompensation(float HorizontalAngle)
{
    // Convert the horizontal angle to radians
    float HorizontalAngleRad = FMath::DegreesToRadians(HorizontalAngle);

    // Calculate the compensation factor
    float CompensationFactor = FMath::Cos(HorizontalAngleRad);

    return CompensationFactor;
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

float UOusterDepthBufferComponent::AdjustVerticalAngleForCircle(float HorizontalAngle, float VerticalAngle)
{
    // Get the normalized horizontal angle

    float nAngle = NormalizedAngle(HorizontalAngle);

    // Calculate the adjustment factor
    float AdjustmentFactor = FMath::Cos(FMath::DegreesToRadians(nAngle));

    // Apply the adjustment to the vertical angle
    float AdjustedVerticalAngle = VerticalAngle * AdjustmentFactor;
    if ((int)HorizontalAngle % 30 == 0 && (int)VerticalAngle % 15 == 0)
    {
        //UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Normalized Angle: %f, Vertical Angle: %f, Adjusted Vertical Angle: %f, Factor %f"), HorizontalAngle, nAngle, VerticalAngle, AdjustedVerticalAngle, AdjustmentFactor);
    }

    return AdjustedVerticalAngle;
}
std::pair<FVector,FVector> UOusterDepthBufferComponent::calculateSphericalFromDepth(
        float distance, 
        float HorizontalAngle, 
        float VerticalAngle, 
        float FOVH, 
        float FOVV
)
{
    float CorrectionFactor = CalculateDistanceCorrection(HorizontalAngle, VerticalAngle, FOVH, FOVV) / 2;

    // Apply the correction factor to the distance
    float CorrectedDistance = distance / CorrectionFactor;

    // Calculate the Z-axis compensation factor
    float ZCompensationFactor = CalculateZAxisCompensation(HorizontalAngle);

    float x = CorrectedDistance * FMath::Cos(FMath::DegreesToRadians(HorizontalAngle)) * FMath::Cos(FMath::DegreesToRadians(VerticalAngle));
    float y = CorrectedDistance * FMath::Sin(FMath::DegreesToRadians(HorizontalAngle)) * FMath::Cos(FMath::DegreesToRadians(VerticalAngle));
    float z = (CorrectedDistance * FMath::Sin(FMath::DegreesToRadians(VerticalAngle))) * ZCompensationFactor;
    


    // Apply inverse matrix to correct distortion
    FVector point(x, y, z);
    //point /= 1000; 

    //convert to spherical coordinates
    float r = FMath::Sqrt(FMath::Square(point.X) + FMath::Square(point.Y) + FMath::Square(point.Z));
    float vCoord = FMath::Acos(point.Z / r);
    float hCoord = FMath::Atan2(point.Y, point.X);
    vCoord = (PI/2.f) - vCoord;
    FVector spherical(r, hCoord, vCoord);
    return std::pair<FVector, FVector>(spherical, point);
}

FVector UOusterDepthBufferComponent::GetCoordinateToAngleAccurate(
    TObjectPtr<USceneCaptureComponent2D> SceneCapture, 
    TObjectPtr<UTextureRenderTarget2D> RenderTarget, 
    TArray<FFloat16Color> &frameBuffer,
    float horizontal, 
    float vertical,
    uint32 width,
    uint32 height,
    float horizontalOffset,
    uint32 x_offset,
    uint32 y_offset
) {
    int32 RenderWidth = RenderTarget->SizeX;
    int32 RenderHeight = RenderTarget->SizeY;
    float FOVH = SceneCapture->FOVAngle;
    float FOVV = SceneCapture->FOVAngle * (RenderHeight / RenderWidth);

    AActor *ParentActor = GetOwner();

    FTransform ParentTransform = ParentActor->GetActorTransform();
    // infinity error as beggining
    float error = std::numeric_limits<float>::infinity();
    FVector result(0, 0, 0);
    int count = 0;

    
    for(uint32 i = x_offset; i < x_offset + width; i++)
    {
        //UE_LOG(LogTemp, Warning, TEXT("Horizontal angle: %f"), (i - (float)RenderWidth / 2.f)/(float)RenderWidth * (float)FOVH);
        for(uint32 j = y_offset; j < y_offset + height; j++)
        {
            count++;
            float HorizontalAngle =(i - (float)RenderWidth / 2.f)/(float)RenderWidth * (float)FOVH;
            float VerticalAngle = -(j - (float)RenderHeight / 2.f)/(float)RenderHeight * (float)FOVV;
            //UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f"), HorizontalAngle, VerticalAngle);
            // Adjust the vertical angle for a circular pattern based on the horizontal angle
            if(j*RenderWidth + i >= frameBuffer.Num())
            {
                UE_LOG(LogTemp, Error, TEXT("FrameBuffer is not large enough! Coords: %d, %d, HorizontalAngle: %f, VerticalAngle: %f Size: %d"), i, j, HorizontalAngle, VerticalAngle, frameBuffer.Num());
                continue;
            }
            FFloat16Color color = frameBuffer[(j * RenderWidth) + i];
            float distance = color.R;

            std::pair<FVector,FVector> coords = calculateSphericalFromDepth(distance, HorizontalAngle, VerticalAngle, FOVH, FOVV);
            FVector spherical = coords.first;
            FVector point = coords.second;
            
            float r = spherical.X;
            float hCoord = spherical.Y;
            float vCoord = spherical.Z;

            float newError = FMath::Abs(vCoord - FMath::DegreesToRadians(vertical)) + FMath::Abs(hCoord - FMath::DegreesToRadians(horizontal));
            //DrawDebugPoint(GetWorld(), ParentTransform.TransformPosition(point), 5.0f, FColor::Red, false, (1/config.frequency)*1.5);

            //DrawDebugString(GetWorld(), ParentTransform.TransformPosition(result), FString::Printf(TEXT("h: %f V %f \nTh: %f Tv: %f\n E: %f"), FMath::RadiansToDegrees(hCoord), FMath::RadiansToDegrees(vCoord),horizontal, vertical, newError), nullptr, FColor::Red, (1/config.frequency)*1.5, false);
            if(newError < error)
            {
                //UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f, Target horizontal angle %f, Target vertical angle %f, Theta (h)  %f, Phi (V) %f, Distance: %f, X: %f, Y: %f, Z: %f, Error: %f"), HorizontalAngle, VerticalAngle, horizontal, vertical, FMath::RadiansToDegrees(theta), FMath::RadiansToDegrees(phi), CorrectedDistance, x, y, z, newError);
                error = newError;
                result = spherical;
            }
        }
    }
    float r = result.X;
    float hCoord = result.Y - FMath::DegreesToRadians(horizontalOffset);
    float vCoord = -result.Z +  (PI/2.f);

    FVector point = FVector(
        r * FMath::Sin(vCoord) * FMath::Cos(hCoord),
        r * FMath::Sin(vCoord) * FMath::Sin(hCoord),
        r * FMath::Cos(vCoord)
    );

    point = MathToolkit::ConvertUEToROS(point);

  /* {
        float r = FMath::Sqrt(FMath::Square(result.X) + FMath::Square(result.Y) + FMath::Square(result.Z));
        float vCoord = FMath::Acos(result.Z / r);
        float hCoord = FMath::Atan2(result.Y, result.X);
        vCoord = (PI/2.f) - vCoord;

        float newError = FMath::Abs(vCoord - FMath::DegreesToRadians(vertical)) + FMath::Abs(hCoord - FMath::DegreesToRadians(horizontal));

        DrawDebugPoint(GetWorld(), ParentTransform.TransformPosition(result), 5.0f, FColor::Red, false, (1/config.frequency)*1.5);

        DrawDebugString(GetWorld(), ParentTransform.TransformPosition(result), FString::Printf(TEXT("h: %f V %f \nTh: %f Tv: %f\n E: %f"), FMath::RadiansToDegrees(hCoord), FMath::RadiansToDegrees(vCoord),horizontal, vertical, newError), nullptr, FColor::Red, (1/config.frequency)*1.5, false);
        //UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f, X: %f, Y: %f, Z: %f"), horizontal, vertical, result.X, result.Y, result.Z);
    }*/
    return point;
}

FVector UOusterDepthBufferComponent::GetCoordinateToAngle(
    TObjectPtr<USceneCaptureComponent2D> SceneCapture, 
    TObjectPtr<UTextureRenderTarget2D> RenderTarget, 
    TArray<FFloat16Color> &frameBuffer,
    float horizontal, 
    float vertical,
    uint32 width,
    uint32 height,
    float horizontalOffset,
    uint32 x_offset,
    uint32 y_offset,
    uint32 recursionDepth
) {
    if(width <= 10  && height <= 10)
    {
        //UE_LOG(LogTemp, Warning, TEXT("Depth: %d"), recursionDepth);
        //UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Vertical Angle: %f, Width: %d, Height: %d, X Offset: %d, Y Offset: %d"), horizontal, vertical, width, height, x_offset, y_offset);

        return GetCoordinateToAngleAccurate(SceneCapture, RenderTarget, frameBuffer, horizontal, vertical, width, height, horizontalOffset, x_offset, y_offset);
    }


    int32 RenderWidth = RenderTarget->SizeX;
    int32 RenderHeight = RenderTarget->SizeY;
    float FOVH = SceneCapture->FOVAngle;
    float FOVV = SceneCapture->FOVAngle * (RenderHeight / RenderWidth);


    recursionDepth++;

    // Calculate The middle of the image
    uint32 middleX = x_offset + width / 2;
    uint32 middleY = y_offset + height / 2;
    float depth = frameBuffer[middleY * width + middleX].R;

    float HorizontalAngle =(middleX- (float)RenderWidth / 2.f)/(float)RenderWidth * (float)FOVH;
    float VerticalAngle = -(middleY- (float)RenderHeight / 2.f)/(float)RenderHeight * (float)FOVV;

    std::pair<FVector,FVector> coords = calculateSphericalFromDepth(depth, HorizontalAngle, VerticalAngle, SceneCapture->FOVAngle, SceneCapture->FOVAngle * (height / width));
    FVector spherical = coords.first;
    
    float r = spherical.X;
    float hCoord = spherical.Y;
    float vCoord = spherical.Z;

    uint32 sectionCoords[2] = {0,0};
    
    if(hCoord > FMath::DegreesToRadians(horizontal))
    {
        //UE_LOG(LogTemp, Warning, TEXT("hCoord: %f, horizontal: %f"), hCoord, FMath::DegreesToRadians(horizontal));
        //UE_LOG(LogTemp, Warning, TEXT("real horizontal: %f hCoord: %f, horizontal: %f Going Left"), FMath::DegreesToRadians(HorizontalAngle), hCoord, FMath::DegreesToRadians(horizontal));
        sectionCoords[0] = 0;
    }
    else
    {
        //UE_LOG(LogTemp, Warning, TEXT("real horizontal: %f hCoord: %f, horizontal: %f Going Right"), FMath::DegreesToRadians(HorizontalAngle), hCoord, FMath::DegreesToRadians(horizontal));
        //UE_LOG(LogTemp, Warning, TEXT("hCoord: %f, horizontal: %f"), hCoord, FMath::DegreesToRadians(horizontal));
        sectionCoords[0] = 1;
    }

    if(vCoord < FMath::DegreesToRadians(vertical))
    {
        //UE_LOG(LogTemp, Warning, TEXT("real vertical: %f vCoord: %f, vertical: %f Going Up"), FMath::RadiansToDegrees(VerticalAngle), FMath::RadiansToDegrees(vCoord), vertical);
        //UE_LOG(LogTemp, Warning, TEXT("vCoord: %f, vertical: %f"), vCoord, FMath::DegreesToRadians(vertical));
        sectionCoords[1] = 0;
    }
    else
    {
        //UE_LOG(LogTemp, Warning, TEXT("real vertical: %f vCoord: %f, vertical: %f Going Down"), FMath::RadiansToDegrees(VerticalAngle), FMath::RadiansToDegrees(vCoord), vertical);
        //UE_LOG(LogTemp, Warning, TEXT("vCoord: %f, vertical: %f"), vCoord, FMath::DegreesToRadians(vertical));
        sectionCoords[1] = 1;
    }

    uint32 widthMargin = width / 8;
    uint32 heightMargin = height / 8;

    uint32 x_offset_new = x_offset + sectionCoords[0] * (width / 2 - widthMargin);
    uint32 y_offset_new = y_offset + sectionCoords[1] * (height / 2 - heightMargin);
    uint32 width_new = width / 2 + widthMargin;
    uint32 height_new = height / 2 + heightMargin;

    return GetCoordinateToAngle(SceneCapture, RenderTarget, frameBuffer, horizontal, vertical, width_new, height_new, horizontalOffset, x_offset_new, y_offset_new, recursionDepth);
}


void UOusterDepthBufferComponent::CaptureDepth()
{
    //UE_LOG(LogTemp, Warning, TEXT("CaptureDepth"));
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

    float HorizontalAngle = 0.0f;
    float VerticalAngle = 0;
    float HorizontalAngleStep = 360.0f / config.horizontalResolution;
    float VerticalAngleStep = config.verticalFOV / config.verticalResolution;

    UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle Step: %f, Vertical Angle Step: %f Horizontal Resolution: %d, Vertical Resolution: %d"), HorizontalAngleStep, VerticalAngleStep, config.horizontalResolution, config.verticalResolution);
    auto start = high_resolution_clock::now();
    for (int32 i = 0; i < config.horizontalResolution; i++)
    {
        for (int32 j = 0; j < config.verticalResolution; j++)
        {
            float AdjustedVerticalAngle = VerticalAngle - config.verticalFOV / 2.0f;

            //

            FVector point = GetPixelValueFromMutltipleCaptureComponents(HorizontalAngle, AdjustedVerticalAngle);
            //UE_LOG(LogTemp, Warning, TEXT("Front: X: %f, Y: %f, Z: %f Horizontal Angle: %f, Vertical Angle: %f"), point.X, point.Y, point.Z, HorizontalAngle, VerticalAngle);
            PointCloud.Add(PointXYZI(point.X/100, point.Y/100, point.Z/100, 1.0f));


            VerticalAngle += VerticalAngleStep;
        }

        HorizontalAngle += HorizontalAngleStep;
        VerticalAngle = 0;
    }

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

    // FPlatformMemory::Memcpy(PointCloud.GetData(), pointCloud->data, PointCloud.Num() * sizeof(PointXYZI));
    for (int i = 0; i < PointCloud.Num(); i++)
    {
        pointCloud->data[i] = PointCloud[i];
    }

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
        }
        );

        cumulativeTime = 0.0f;
    }
}

FVector UOusterDepthBufferComponent::GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle)
{
    if (!SceneCaptureFront || !SceneCaptureRight || !SceneCaptureBack || !SceneCaptureLeft)
    {
        return FVector(0.0f, 0.0f, 0.0f);
    }

    int32 Width = RenderTargetFront->SizeX;
    int32 Height = RenderTargetFront->SizeY;

    float FOVH = SceneCaptureFront->FOVAngle;
    float FOVV = SceneCaptureFront->FOVAngle * (Height / Width);
            float calculationHorizontalAngle = NormalizedAngle(HorizontalAngle);

    if (HorizontalAngle < 90)
    {

        FVector point = GetCoordinateToAngle(SceneCaptureFront, RenderTargetFront, ImageDataFront, calculationHorizontalAngle, VerticalAngle, Width, Height, 45);
        //debug line to the point
        //DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }
    else if (HorizontalAngle < 180.0f)
    {
        FVector point = GetCoordinateToAngle(SceneCaptureRight, RenderTargetRight, ImageDataRight, calculationHorizontalAngle, VerticalAngle, Width, Height, 135);
        //debug line to the point
        //DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }
    else if (HorizontalAngle < 270.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f"), HorizontalAngle);
        float newHorizontalAngle = HorizontalAngle - 180;
        FVector point = GetCoordinateToAngle(SceneCaptureBack, RenderTargetBack, ImageDataBack, calculationHorizontalAngle, VerticalAngle, Width, Height, 225);
        //debug line to the point
        //DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }
    else
    {
        float newHorizontalAngle = HorizontalAngle - 270;
        FVector point = GetCoordinateToAngle(SceneCaptureLeft, RenderTargetLeft, ImageDataLeft, calculationHorizontalAngle, VerticalAngle, Width, Height, 315);
        //debug line to the point
        //DrawDebugLine(GetWorld(), ParentTransform.GetLocation(), point, FColor::Red, false, 1/config.frequency, 0, 1.0f);
        return point;
    }

    return FVector(0.0f, 0.0f, 0.0f);
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

        // Store the original projection matrix
        /*FMatrix ProjectionMatrix;
        const float ClippingPlane = (SceneCaptureComponent->bOverride_CustomNearClippingPlane) ? SceneCaptureComponent->CustomNearClippingPlane : GNearClippingPlane;
        BuildProjectionMatrix(FIntPoint(RenderTarget->SizeX, RenderTarget->SizeY), FOV, ClippingPlane, ProjectionMatrix);
        SceneCaptureComponent->CustomProjectionMatrix = ProjectionMatrix;
        SceneCaptureComponent->bUseCustomProjectionMatrix = true;*/

        return SceneCaptureComponent;
    }
    return nullptr;
}
