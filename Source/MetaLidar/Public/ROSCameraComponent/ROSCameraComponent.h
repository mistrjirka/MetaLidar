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
#include "Math/Color.h"

#include "ROSCameraComponent.generated.h"
#define SHARED_MEMORY_NAME_CAMERA "MetaLidarCamera5465AS6"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class METALIDAR_API UROSCameraComponent : public UActorComponent
{
    GENERATED_BODY()

    protected:
        virtual void BeginPlay() override;

    public:
        virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
    private:
        uint32 seq;
        uint32 resolutionX;
        uint32 resolutionY;
        float focalLength;
        float sensorSizeX;
        float iris;
        float accumulatedTime;
        uint32 FPS;
        std::unique_ptr<SharedMemory> shared_memory;
        
        UPROPERTY(EditAnywhere)
        USceneCaptureComponent2D* SceneCapture;

        UPROPERTY(EditAnywhere)
        UTextureRenderTarget2D* RenderTarget;

        UPROPERTY(EditAnywhere)
        TArray<FColor> ImageData;

        void InitializeCameraComponent();
        void SendImageToROS(uint32);
        uint32 GetTimestampMicroseconds();

    public:
        UROSCameraComponent();
};