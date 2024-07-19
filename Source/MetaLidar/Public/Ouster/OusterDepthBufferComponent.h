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
#include <atomic>
#include "AngelToPixelUtility/AngleToPixelUtility.h"
#include "SharedMemory/SharedMemory.h"
#include "Math/Float16Color.h"

#include "OusterDepthBufferComponent.generated.h"

USTRUCT()
struct FSensorConfig
{
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
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;

private:
    UPROPERTY()
    FSensorConfig config;

    TObjectPtr<USceneCaptureComponent2D> SceneCaptureFront;

    TObjectPtr<UTextureRenderTarget2D> RenderTargetFront;

    TObjectPtr<USceneCaptureComponent2D> SceneCaptureRight;

    TObjectPtr<UTextureRenderTarget2D> RenderTargetRight;

    TObjectPtr<USceneCaptureComponent2D> SceneCaptureBack;

    TObjectPtr<UTextureRenderTarget2D> RenderTargetBack;

    TObjectPtr<USceneCaptureComponent2D> SceneCaptureLeft;

    TObjectPtr<UTextureRenderTarget2D> RenderTargetLeft;

    TArray<FFloat16Color> ImageDataFront;

    TArray<FFloat16Color> ImageDataRight;

    TArray<FFloat16Color> ImageDataBack;

    TArray<FFloat16Color> ImageDataLeft;


    UPROPERTY()
    float frequncyDelta;

    UPROPERTY()
    float cumulativeTime;

    std::atomic<bool> packetReady;

    UPROPERTY()
    uint64 packetSeq;

    UPROPERTY()
    float zoffset;

    std::atomic<bool> readySendingData;

    std::atomic<bool> captureReady;

    FMatrix inverseProjectionMatrix;

    FMatrix projectionMatrix;

    TArray<PointXYZI> PointCloud;

    std::unique_ptr<SharedMemory> shared_memory;

    void InitializeCaptureComponent();

    float AdjustVerticalAngleForCircle(float HorizontalAngle, float VerticalAngle);

    void CaptureDepth();

    void CaptureScene();

    float NormalizedAngle(float HorizontalAngle);

    void UpdateBuffer(TObjectPtr<UTextureRenderTarget2D> , TArray<FFloat16Color> &);

    float GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle);

    void GenerateDataPacket(uint64 timestamp);

    float GetPixelFromAngle(TObjectPtr<USceneCaptureComponent2D> SceneCapture, TObjectPtr<UTextureRenderTarget2D> RenderTarget, TArray<FFloat16Color> &frameBuffer, float HorizontalAngle, float VerticalAngle);

    float CalculateDistanceCorrection(float HorizontalAngle, float VerticalAngle, float FOVH, float FOVV);

    uint32 CalculatePointStep(const TArray<PointField> &fields);

    uint32 GenerateData(uint8 *data, uint32 size, uint32 timestamp);

    void GenerateDataPacket(uint32 TimeStamp);

    uint32 GetTimestampMicroseconds();

    TObjectPtr<UTextureRenderTarget2D> CreateRenderTarget(uint32 Width, uint32 Height);

    TObjectPtr<USceneCaptureComponent2D> CreateSceneCaptureComponent(FVector RelativeLocation, FRotator RelativeRotation, TObjectPtr<UTextureRenderTarget2D> RenderTarget, float FOV);
};