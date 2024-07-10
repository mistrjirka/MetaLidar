#include "Ouster/OusterDepthBufferComponent.h"

void BuildProjectionMatrix(FIntPoint InRenderTargetSize, float InFOV, float InNearClippingPlane, FMatrix& OutProjectionMatrix)
{
	float const XAxisMultiplier = 1.0f;
	float const YAxisMultiplier = InRenderTargetSize.X / float(InRenderTargetSize.Y);

	if ((int32)ERHIZBuffer::IsInverted)
	{
		OutProjectionMatrix = FReversedZPerspectiveMatrix(
			InFOV,
			InFOV,
			XAxisMultiplier,
			YAxisMultiplier,
			InNearClippingPlane,
			InNearClippingPlane
			);
	}
	else
	{
		OutProjectionMatrix = FPerspectiveMatrix(
			InFOV,
			InFOV,
			XAxisMultiplier,
			YAxisMultiplier,
			InNearClippingPlane,
			InNearClippingPlane
			);
	}
}

UOusterDepthBufferComponent::UOusterDepthBufferComponent()
{
    PrimaryComponentTick.bCanEverTick = true;

    config.horizontalResolution = 1024;
    config.verticalResolution = 128;
    config.verticalFOV = 45.0f;
    config.frequency = 1;
    config.MemoryLabel = "/t07ySQdKFH_meta_lidar";
    config.MemorySize = 40000000;
    config.PointStep = 4+4+4+4;
    packetSeq = 0;

    frequncyDelta = 1.f/config.frequency;
    cumulativeTime = 0.0f;
}

FMatrix CalculateInverseProjectionMatrix(const FMatrix& OriginalMatrix)
{
    // Calculate the inverse of the original projection matrix
    return OriginalMatrix.Inverse();
}

void UOusterDepthBufferComponent::BeginPlay()
{
    Super::BeginPlay();
    UE_LOG(LogTemp, Warning, TEXT("OusterDepthBufferComponent BeginPlay"));
    
    this->shared_memory = std::make_unique<SharedMemory>(TCHAR_TO_ANSI(*config.MemoryLabel), config.MemorySize);
    MemoryPacket* packet = (MemoryPacket*)this->shared_memory->get_ptr();
    packet->seq = 0;
    packet->packet_size = 0;

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&(packet->mutex), &attr);

    
    InitializeCaptureComponent();
}

void UOusterDepthBufferComponent::InitializeCaptureComponent()
{
    if (AActor* Owner = GetOwner())
    {
        int32 singleSensorResolution = (config.horizontalResolution / 4) * 1.5;

        RenderTargetFront = CreateRenderTarget(singleSensorResolution, singleSensorResolution);
        RenderTargetRight = CreateRenderTarget(singleSensorResolution, singleSensorResolution);
        RenderTargetBack = CreateRenderTarget(singleSensorResolution, singleSensorResolution);
        RenderTargetLeft = CreateRenderTarget(singleSensorResolution, singleSensorResolution);

        SceneCaptureFront = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, 10.0f), FRotator(0.0f, 0.0f, 0.0f), RenderTargetFront, 90.0f);
        SceneCaptureRight = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, 10.0f), FRotator(0.0f, 90.0f, 0.0f), RenderTargetRight, 90.0f);
        SceneCaptureBack = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, 10.0f), FRotator(0.0f, 180.0f, 0.0f), RenderTargetBack, 90.0f);
        SceneCaptureLeft = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, 10.0f), FRotator(0.0f, 270.0f, 0.0f), RenderTargetLeft, 90.0f);
    }
}



void UOusterDepthBufferComponent::UpdateBuffer(UTextureRenderTarget2D* RenderTarget, TArray<FFloat16Color>& Image)
{
    FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
    RenderTargetResource->ReadFloat16Pixels(Image);
}

void UOusterDepthBufferComponent::CaptureScene()
{

    SceneCaptureFront->CaptureScene();
    SceneCaptureRight->CaptureScene();
    SceneCaptureBack->CaptureScene();
    SceneCaptureLeft->CaptureScene();

    UpdateBuffer(RenderTargetFront, ImageDataFront);
    UpdateBuffer(RenderTargetRight, ImageDataRight);
    UpdateBuffer(RenderTargetBack, ImageDataBack);
    UpdateBuffer(RenderTargetLeft, ImageDataLeft);
}

float CalculateDistanceCorrection(float HorizontalAngle, float VerticalAngle, float FOVH, float FOVV)
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

void UOusterDepthBufferComponent::CaptureDepth()
{
    if (!SceneCaptureFront || !SceneCaptureRight || !SceneCaptureBack || !SceneCaptureLeft)
    {
        return;
    }
    CaptureScene();

    PointCloud.Empty();

    float HorizontalAngle = 0.0f;
    float VerticalAngle = -config.verticalFOV / 2.0f;
    float HorizontalAngleStep = 360.0f / config.horizontalResolution;
    float VerticalAngleStep = config.verticalFOV / config.verticalResolution;
    AActor* ParentActor = GetOwner();
    if (!ParentActor)
    {
        return;
    }

    FTransform ParentTransform = ParentActor->GetActorTransform();

    // Get the inverse projection matrices
    FMatrix InverseFrontMatrix = CalculateInverseProjectionMatrix(SceneCaptureFront->CustomProjectionMatrix);
    FMatrix InverseRightMatrix = CalculateInverseProjectionMatrix(SceneCaptureRight->CustomProjectionMatrix);
    FMatrix InverseBackMatrix = CalculateInverseProjectionMatrix(SceneCaptureBack->CustomProjectionMatrix);
    FMatrix InverseLeftMatrix = CalculateInverseProjectionMatrix(SceneCaptureLeft->CustomProjectionMatrix);

    for (int32 i = 0; i < config.horizontalResolution; i++)
    {
        float distance;
        for (int32 j = 0; j < config.verticalResolution; j++)
        {
            distance = GetPixelValueFromMutltipleCaptureComponents(HorizontalAngle, VerticalAngle);
            if (distance > 0 && distance < 6000)
            {
                float CorrectionFactor = CalculateDistanceCorrection(HorizontalAngle, VerticalAngle, SceneCaptureFront->FOVAngle, SceneCaptureFront->FOVAngle);
                
                // Apply the correction factor to the distance
                float CorrectedDistance = distance / CorrectionFactor;

                float x = CorrectedDistance * FMath::Cos(FMath::DegreesToRadians(HorizontalAngle)) * FMath::Cos(FMath::DegreesToRadians(VerticalAngle));
                float y = CorrectedDistance * FMath::Sin(FMath::DegreesToRadians(HorizontalAngle)) * FMath::Cos(FMath::DegreesToRadians(VerticalAngle));
                float z = CorrectedDistance * FMath::Sin(FMath::DegreesToRadians(VerticalAngle));
                
                // Apply inverse matrix to correct distortion
                FVector4 OriginalPoint(x, y, z, 1.0f);
                //FVector4 CorrectedPoint;

                /*if (HorizontalAngle <= 45.0f || HorizontalAngle > 315.0f)
                {
                    CorrectedPoint = InverseFrontMatrix.TransformFVector4(OriginalPoint);
                }
                else if (HorizontalAngle < 180.0f)
                {
                    CorrectedPoint = InverseRightMatrix.TransformFVector4(OriginalPoint);
                }
                else if (HorizontalAngle < 270.0f)
                {
                    CorrectedPoint = InverseBackMatrix.TransformFVector4(OriginalPoint);
                }
                else
                {
                    CorrectedPoint = InverseLeftMatrix.TransformFVector4(OriginalPoint);
                }*/

                PointXYZI point;
                point.x = OriginalPoint.X;
                point.y = OriginalPoint.Y;
                point.z = OriginalPoint.Z;
                point.intensity = 1.0f;
                PointCloud.Add(point);

                FVector LocalPoint(OriginalPoint.X * 100.0f, OriginalPoint.Y * 100.0f, OriginalPoint.Z * 100.0f);
                FVector WorldPoint = ParentTransform.TransformPosition(LocalPoint);

                DrawDebugPoint(GetWorld(), WorldPoint, 5.0f, FColor::Red, false, 1.0f);
            }
            VerticalAngle += VerticalAngleStep;
        }

        HorizontalAngle += HorizontalAngleStep;
        VerticalAngle = -config.verticalFOV / 2.0f;
    }
}
uint32 UOusterDepthBufferComponent::GenerateData(uint8* data, uint32 size, uint32 timestamp)
{
    uint32 timestamp_sec = timestamp / 1000000;
    uint32 timestamp_nsec = (timestamp % 1000000) * 1000;
    size_t packet_size = 0;
    PointCloud2Reduced* pointCloud = (PointCloud2Reduced*)data;
    pointCloud->height = 1;
    pointCloud->width = PointCloud.Num();
    pointCloud->is_bigendian = false;
    pointCloud->point_step = config.PointStep;
    pointCloud->row_step = pointCloud->width;
    pointCloud->is_dense = true;
    for(int i = 0; i < PointCloud.Num(); i++)
    {
        pointCloud->data[i] = PointCloud[i];
    }

    return  sizeof(PointCloud2Reduced) + pointCloud->width * pointCloud->point_step;
}

void UOusterDepthBufferComponent::GenerateDataPacket(uint32 TimeStamp)
{
    MemoryPacket* packet = (MemoryPacket*)this->shared_memory->get_ptr();
    uint32 available_size = this->config.MemorySize - sizeof(MemoryPacket);
    pthread_mutex_lock(&(packet->mutex));
    packet->seq++;
    packet->packet_size = GenerateData(packet->data, available_size, TimeStamp);
    pthread_mutex_unlock(&(packet->mutex));
}

uint32 UOusterDepthBufferComponent::GetTimestampMicroseconds()
{
  return (uint32)(fmod(GetWorld()->GetTimeSeconds(), 3600.f) * 1000000);  // sec -> microsec
}

void UOusterDepthBufferComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    cumulativeTime += DeltaTime;
    
    if(packetReady)
    {
        packetReady = false;
        GenerateDataPacket(this->GetTimestampMicroseconds());
    }


    if(cumulativeTime < frequncyDelta)
    { 
       return;
    }

    cumulativeTime = 0.0f;
    CaptureDepth();
    packetReady = true;
}

float UOusterDepthBufferComponent::GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle)
{
    if (!SceneCaptureFront || !SceneCaptureRight || !SceneCaptureBack || !SceneCaptureLeft)
    {
        return 0.0f;
    }

    if (HorizontalAngle <= 45.0f || HorizontalAngle > 315.0f)
    {
        if(HorizontalAngle > 315.0f)
        {
            HorizontalAngle = -(360.0f - HorizontalAngle);
        }

        return GetPixelFromAngle(SceneCaptureFront, RenderTargetFront, ImageDataFront, HorizontalAngle, VerticalAngle);
    }
    /*else if (HorizontalAngle < 180.0f)
    {
        return GetPixelFromAngle(SceneCaptureRight, RenderTargetRight, ImageDataRight, HorizontalAngle - 90.0f, VerticalAngle);
    }
    else if (HorizontalAngle < 270.0f)
    {
        return GetPixelFromAngle(SceneCaptureBack, RenderTargetBack, ImageDataBack, HorizontalAngle - 180.0f, VerticalAngle);
    }
    else
    {
        return GetPixelFromAngle(SceneCaptureLeft, RenderTargetLeft, ImageDataLeft, HorizontalAngle - 270.0f, VerticalAngle);
    }*/

   return 0.0f; 
}

float UOusterDepthBufferComponent::GetPixelFromAngle(USceneCaptureComponent2D* SceneCapture, UTextureRenderTarget2D* RenderTarget, TArray<FFloat16Color>& frameBuffer, float HorizontalAngle, float VerticalAngle)
{
    int32 Width = RenderTarget->SizeX;
    int32 Height = RenderTarget->SizeY;
    float FOVH = SceneCapture->FOVAngle;

    FIntPoint PixelCoords = UAngleToPixelUtility::GetPixelCoordinates(HorizontalAngle, VerticalAngle, FOVH, Width, Height);
    FFloat16Color PixelColor;

    if(frameBuffer.Num() <= Width * PixelCoords.Y + PixelCoords.X)
    {
        UE_LOG(LogTemp, Error, TEXT("FrameBuffer is not large enough! Coords: %d, %d, HorizontalAngle: %f, VerticalAngle: %f Size: %d"), PixelCoords.X, PixelCoords.Y, HorizontalAngle, VerticalAngle, frameBuffer.Num());
    } else {
        PixelColor = frameBuffer[PixelCoords.Y * Width + PixelCoords.X];
    }

    // Assuming the depth is stored in the red channel
    float DepthValue = PixelColor.R;
    //UE_LOG(LogTemp, Warning, TEXT("Depth value: %f, HorizontalAngle: %f, VerticalAngle: %f, PixelCoords: %d, %d RenderTargetSize: %d, %d"), DepthValue, HorizontalAngle, VerticalAngle, PixelCoords.X, PixelCoords.Y, Width, Height);
    /*if(DepthValue > 1.0f)
    {
        UE_LOG(LogTemp, Warning, TEXT("Depth value is greater than 1.0f! Value: %f"), DepthValue);
    }*/
    return DepthValue/100;
}

UTextureRenderTarget2D* UOusterDepthBufferComponent::CreateRenderTarget(uint32 Width, uint32 Height)
{
    UTextureRenderTarget2D* RenderTarget = NewObject<UTextureRenderTarget2D>();
    RenderTarget->InitAutoFormat(Width, Height);
    RenderTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f; // Using 32-bit float format for depth
    return RenderTarget;
}

USceneCaptureComponent2D* UOusterDepthBufferComponent::CreateSceneCaptureComponent(FVector RelativeLocation, FRotator RelativeRotation, UTextureRenderTarget2D* RenderTarget, float FOV)
{
    if (AActor* Owner = GetOwner())
    {
        USceneCaptureComponent2D* SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(Owner);
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
        FMatrix ProjectionMatrix;
        const float ClippingPlane = (SceneCaptureComponent->bOverride_CustomNearClippingPlane) ? SceneCaptureComponent->CustomNearClippingPlane : GNearClippingPlane;
        BuildProjectionMatrix(FIntPoint(RenderTarget->SizeX, RenderTarget->SizeY), FOV, ClippingPlane, ProjectionMatrix);
        SceneCaptureComponent->CustomProjectionMatrix = ProjectionMatrix;
        
        return SceneCaptureComponent;
    }
    return nullptr;
}

