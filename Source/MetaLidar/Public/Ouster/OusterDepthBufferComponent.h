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
#include "ThreadSafeArray/TThreadSafeArray.h"

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
    TArray<FFloat16Color> ImageData;
    TArray<Vector3Fast> PointCache;
} SensorField;

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

    FVector CalculateSphereCoordinateCached(TArray<FFloat16Color> &frameBuffer,
                                            TArray<Vector3Fast> &PointCache,
                                            uint32 x, uint32 y, uint32 width, uint32 height, float FOVH);

    std::unique_ptr<SharedMemory> shared_memory;

    void InitializeCaptureComponent();

    void CaptureDepth();

    void CaptureScene();

    PointXYZI GetCoordinateToAngleAccurate(
        TObjectPtr<USceneCaptureComponent2D> SceneCapture,
        TObjectPtr<UTextureRenderTarget2D> RenderTarget,
        TArray<FFloat16Color> &frameBuffer,
        TArray<Vector3Fast> &PointCache,
        float horizontal,
        float vertical,
        uint32 width,
        uint32 height,
        float horizontalOffset,
        uint32 x_offset = 0,
        uint32 y_offset = 0);

    PointXYZI GetCoordinateToAngle(
        TObjectPtr<USceneCaptureComponent2D> SceneCapture,
        TObjectPtr<UTextureRenderTarget2D> RenderTarget,
        TArray<FFloat16Color> &frameBuffer,
        TArray<Vector3Fast> &PointCache,
        float horizontal,
        float vertical,
        uint32 width,
        uint32 height,
        float horizontalOffset,
        uint32 x_offset = 0,
        uint32 y_offset = 0,
        uint32 step = 0);

    float NormalizedAngle(float HorizontalAngle);

    void UpdateBuffer(TObjectPtr<UTextureRenderTarget2D>, TArray<FFloat16Color> &);

    PointXYZI GetPixelValueFromMutltipleCaptureComponents(float HorizontalAngle, float VerticalAngle);

    float GetPixelFromAngle(TObjectPtr<USceneCaptureComponent2D> SceneCapture, TObjectPtr<UTextureRenderTarget2D> RenderTarget, TArray<FFloat16Color> &frameBuffer, float HorizontalAngle, float VerticalAngle);

    uint32 CalculatePointStep(const TArray<PointField> &fields);
    int32 SensorUpdateIndex;
    uint32 GenerateData(uint8 *data, uint32 size, uint32 timestamp);

    void GenerateDataPacket(uint32 TimeStamp);

    uint32 GetTimestampMicroseconds();

    TObjectPtr<UTextureRenderTarget2D> CreateRenderTarget(uint32 Width, uint32 Height);

    TObjectPtr<USceneCaptureComponent2D> CreateSceneCaptureComponent(FVector RelativeLocation, FRotator RelativeRotation, TObjectPtr<UTextureRenderTarget2D> RenderTarget, float FOV);
};