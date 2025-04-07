#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "RenderUtils.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "Kismet/KismetMathLibrary.h"
#include "ImageUtils.h"
#include "Components/SceneCaptureComponent2D.h"
#include "SharedStructure.h"
#include <cmath>
#include <tuple>
#include <pthread.h>
#include <atomic>
#include "Engine/Engine.h"
#include "AngelToPixelUtility/AngleToPixelUtility.h"
#include "SharedMemory/SharedMemory.h"
#include "Math/Float16Color.h"
#include "ThreadSafeArray/TThreadSafeArray.h"
#include "MathToolkitLibrary.h"

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

typedef struct Vector3Fast
{
    uint32 validity;
    float r;
    float horizontalAngle;
    float verticalAngle;
} Vector3Fast;

typedef struct SensorField
{
    TObjectPtr<USceneCaptureComponent2D> SceneCapture;
    TObjectPtr<UTextureRenderTarget2D> RenderTarget;
    TArray<FFloat16Color> ImageData[2];
} SensorField;

typedef struct PixelCache2D
{
    uint32 x;
    uint32 y;
} PixelCache2D;

enum STATE : uint8
{
    IDLE,
    CAPTURING,
    PROCESSING
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

    TArray<SensorField> Sensors;

    uint64 ValidityTime;

    UPROPERTY()
    float frequncyDelta;

    UPROPERTY()
    float cumulativeTime;

    std::atomic<bool> packetReady;

    UPROPERTY()
    uint64 packetSeq;

    UPROPERTY()
    float zoffset;

    UPROPERTY()
    float angleOffset;

    std::atomic<bool> readySendingData;

    std::atomic<bool> readyCapturingData;

    std::atomic<bool> captureReady;

    FMatrix inverseProjectionMatrix;

    FMatrix projectionMatrix;

    TThreadSafeArray<PointXYZI> PointCloud;

    TArray<TArray<PixelCache2D>> PixelCache;

    std::atomic<bool> isProcessing;
    std::unique_ptr<SharedMemory> shared_memory;
    float TanHalfFOVVRad;
    float TanHalfFOVHRad;
    float FOVV;
    float FOVH;
    int numberOfThreads;

    void InitializeCaptureComponent();

    void CaptureDepth(uint32 BufferIndex);

    void CaptureScene();

    PointXYZI GetCoordinateToAngle(
        TObjectPtr<USceneCaptureComponent2D> SceneCapture,
        TObjectPtr<UTextureRenderTarget2D> RenderTarget,
        TArray<FFloat16Color> &frameBuffer,
        float horizontal,
        float vertical,
        uint32 width,
        uint32 height,
        float horizontalOffset = 0.0f
    );

    float NormalizedAngle(float HorizontalAngle);

    void UpdateBuffer(TObjectPtr<UTextureRenderTarget2D>, TArray<FFloat16Color> &);

    void InitializeCache();

    PointXYZI GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle, uint32 CurrentBufferIndex);

    float GetPixelFromAngle(TObjectPtr<USceneCaptureComponent2D> SceneCapture, TObjectPtr<UTextureRenderTarget2D> RenderTarget, TArray<FFloat16Color> &frameBuffer, float HorizontalAngle, float VerticalAngle);

    uint32 CalculatePointStep(const TArray<PointField> &fields);
    uint32 GenerateData(uint8 *data, uint32 size, uint32 timestamp);

    void GenerateDataPacket(uint32 TimeStamp);

    uint32 GetTimestampMicroseconds();

    TObjectPtr<UTextureRenderTarget2D> CreateRenderTarget(uint32 Width, uint32 Height);

    TObjectPtr<USceneCaptureComponent2D> CreateSceneCaptureComponent(FVector RelativeLocation, FRotator RelativeRotation, TObjectPtr<UTextureRenderTarget2D> RenderTarget, float FOV);

    int ScheduleCaptures(); // returns the number of capture components

    TArray<TArray<uint32>> ScheduledCaptures;

    int32 frameIndex;
    int32 SensorsUpdated;
    int32 FramesAvailableForProcessing;

    uint32 BufferIndex;
    
    void SwitchBuffer();
};