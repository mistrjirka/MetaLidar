#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "RenderUtils.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "ImageUtils.h"
#include "Components/SceneCaptureComponent2D.h"
#include "SharedStructure.h"
#include <cmath>
#include <tuple>
#include <pthread.h>
#include "AngelToPixelUtility/AngleToPixelUtility.h"
#include "SharedMemory/SharedMemory.h"
#include "Math/Float16Color.h"


#include "OusterDepthBufferComponent.generated.h"

USTRUCT()
struct FSensorConfig {
    GENERATED_BODY()
    public:
        uint32 horizontalResolution;
        uint32 verticalResolution;
        uint32 frequency;
        float verticalFOV;
        FString MemoryLabel;
        uint32 MemorySize;
        uint32 PointStep;
        uint32 RowStep;
};


UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class METALIDAR_API UOusterDepthBufferComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UOusterDepthBufferComponent();

protected:
    virtual void BeginPlay() override;

public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    UPROPERTY()
    FSensorConfig config;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureFront;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTargetFront;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureRight;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTargetRight;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureBack;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTargetBack;

    UPROPERTY()
    USceneCaptureComponent2D* SceneCaptureLeft;

    UPROPERTY()
    UTextureRenderTarget2D* RenderTargetLeft;

    TArray<FFloat16Color> ImageDataFront;

    TArray<FFloat16Color> ImageDataRight;

    TArray<FFloat16Color> ImageDataBack;

    TArray<FFloat16Color> ImageDataLeft;

    UPROPERTY()
    float frequncyDelta;

    UPROPERTY()
    float cumulativeTime;

    UPROPERTY()
    bool packetReady;

    UPROPERTY()
    uint64 packetSeq;

    FMatrix inverseProjectionMatrix;

    FMatrix projectionMatrix;
    
    TArray<PointXYZI> PointCloud;
    
    std::unique_ptr<SharedMemory> shared_memory;

    void InitializeCaptureComponent();

    void CaptureDepth();
    
    void CaptureScene();

    void UpdateBuffer(UTextureRenderTarget2D*, TArray<FFloat16Color>&);

    float GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle);

    void GenerateDataPacket(uint64 timestamp);

    float GetPixelFromAngle(USceneCaptureComponent2D* SceneCapture, UTextureRenderTarget2D* RenderTarget, TArray<FFloat16Color>& frameBuffer, float HorizontalAngle, float VerticalAngle);

    float CalculateDistanceCorrection(float HorizontalAngle, float VerticalAngle, float FOVH, float FOVV);

    uint32 CalculatePointStep(const TArray<PointField> &fields);

    uint32 GenerateData(uint8* data, uint32 size, uint32 timestamp);

    void GenerateDataPacket(uint32 TimeStamp);

    uint32 GetTimestampMicroseconds();
 
    UTextureRenderTarget2D* CreateRenderTarget(uint32 Width, uint32 Height);
 
    USceneCaptureComponent2D* CreateSceneCaptureComponent(FVector RelativeLocation, FRotator RelativeRotation, UTextureRenderTarget2D* RenderTarget, float FOV);
};