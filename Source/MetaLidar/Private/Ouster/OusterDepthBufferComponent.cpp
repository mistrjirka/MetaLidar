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
    zoffset = 30.0f;

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
    UE_LOG(LogTemp, Warning, TEXT("correction factor for 0,0 case %f"), CalculateDistanceCorrection(0, 0, 90, 90));
    
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

        SceneCaptureFront = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 0.0f, 0.0f), RenderTargetFront, 90.0f);
        SceneCaptureRight = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 90.0f, 0.0f), RenderTargetRight, 90.0f);
        SceneCaptureBack = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 180.0f, 0.0f), RenderTargetBack, 90.0f);
        SceneCaptureLeft = CreateSceneCaptureComponent(FVector(0.0f, 0.0f, zoffset), FRotator(0.0f, 270.0f, 0.0f), RenderTargetLeft, 90.0f);
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
    if(angle >= 180.0f)
    {
        centeredAngle = angle - 360.0f;
    }

    return centeredAngle;
}


float UOusterDepthBufferComponent::NormalizedAngle(float HorizontalAngle)
{
    float calculationHorizontalAngle = 0;
    if(HorizontalAngle < 45.0f || HorizontalAngle >= 315.0f)
    {
        calculationHorizontalAngle = centerAngleAroundNewZero(HorizontalAngle);
    }
    else if(HorizontalAngle < 135.0f)
    {
        calculationHorizontalAngle = HorizontalAngle - 90;
    }
    else if(HorizontalAngle < 225.0f)
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
    if((int)HorizontalAngle % 30 ==0 && (int)VerticalAngle % 15 == 0){
        UE_LOG(LogTemp, Warning, TEXT("Horizontal Angle: %f, Normalized Angle: %f, Vertical Angle: %f, Adjusted Vertical Angle: %f, Factor %f"), HorizontalAngle, nAngle, VerticalAngle, AdjustedVerticalAngle, AdjustmentFactor);
    }

    return AdjustedVerticalAngle;
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
    float VerticalAngle = 0;
    float HorizontalAngleStep = 360.0f / config.horizontalResolution;
    float VerticalAngleStep = config.verticalFOV / config.verticalResolution;
    AActor* ParentActor = GetOwner();
    
    if (!ParentActor)
    {
        return;
    }

    FTransform ParentTransform = ParentActor->GetActorTransform();

    for (int32 i = 0; i < config.horizontalResolution; i++)
    {
        float distance;
        for (int32 j = 0; j < config.verticalResolution; j++)
        {
            // Adjust the vertical angle for a circular pattern based on the horizontal angle
            float AdjustedVerticalAngle = VerticalAngle/* AdjustVerticalAngleForCircle(HorizontalAngle, VerticalAngle)*/;
            AdjustedVerticalAngle -= config.verticalFOV / 2.0f;
            distance = GetPixelValueFromMutltipleCaptureComponents(HorizontalAngle, AdjustedVerticalAngle);
            if (distance > 0 && distance < 6000)
            {
                
                float calculationHorizontalAngle = NormalizedAngle(HorizontalAngle);


                float CorrectionFactor = CalculateDistanceCorrection(calculationHorizontalAngle, AdjustedVerticalAngle, SceneCaptureFront->FOVAngle, SceneCaptureFront->FOVAngle)/2;
                
                // Apply the correction factor to the distance
                float CorrectedDistance = distance / CorrectionFactor;

                // Calculate the Z-axis compensation factor
                float ZCompensationFactor = CalculateZAxisCompensation(calculationHorizontalAngle);

                float x = CorrectedDistance * FMath::Cos(FMath::DegreesToRadians(HorizontalAngle)) * FMath::Cos(FMath::DegreesToRadians(AdjustedVerticalAngle));
                float y = CorrectedDistance * FMath::Sin(FMath::DegreesToRadians(HorizontalAngle)) * FMath::Cos(FMath::DegreesToRadians(AdjustedVerticalAngle));
                float z = (CorrectedDistance * FMath::Sin(FMath::DegreesToRadians(AdjustedVerticalAngle))) * ZCompensationFactor + zoffset/100.f;

                
                // Apply inverse matrix to correct distortion
                FVector4 OriginalPoint(x, y, z, 1.0f);
                

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
        VerticalAngle = 0;
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

    float calculationHorizontalAngle = NormalizedAngle(HorizontalAngle);


    if (HorizontalAngle < 45.0f || HorizontalAngle >= 315.0f)
    {
        return GetPixelFromAngle(SceneCaptureFront, RenderTargetFront, ImageDataFront, calculationHorizontalAngle, VerticalAngle);
    }
    else if (HorizontalAngle < 135.0f)
    {
        return GetPixelFromAngle(SceneCaptureRight, RenderTargetRight, ImageDataRight, calculationHorizontalAngle, VerticalAngle);
    }
    else if (HorizontalAngle < 225.0f)
    {
        return GetPixelFromAngle(SceneCaptureBack, RenderTargetBack, ImageDataBack, calculationHorizontalAngle, VerticalAngle);
    }
    else if (HorizontalAngle < 315.0f)
    {
        return GetPixelFromAngle(SceneCaptureLeft, RenderTargetLeft, ImageDataLeft, calculationHorizontalAngle, VerticalAngle);
    }

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
    return DepthValue/100.f;
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
        /*FMatrix ProjectionMatrix;
        const float ClippingPlane = (SceneCaptureComponent->bOverride_CustomNearClippingPlane) ? SceneCaptureComponent->CustomNearClippingPlane : GNearClippingPlane;
        BuildProjectionMatrix(FIntPoint(RenderTarget->SizeX, RenderTarget->SizeY), FOV, ClippingPlane, ProjectionMatrix);
        SceneCaptureComponent->CustomProjectionMatrix = ProjectionMatrix;
        SceneCaptureComponent->bUseCustomProjectionMatrix = true;*/
        
        return SceneCaptureComponent;
    }
    return nullptr;
}

