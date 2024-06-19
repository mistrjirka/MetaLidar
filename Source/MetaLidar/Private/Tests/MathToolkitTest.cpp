#include "Misc/AutomationTest.h"

#if WITH_AUTOMATION_TESTS


#include "MathToolkit/MathToolkit.h"
#include "CircularBuffer/CircularBuffer.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(MathToolkitTest, "MetaLidar.MathToolkit", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool MathToolkitTest             ::RunTest(const FString &Parameters)
{
    CircularBuffer<std::pair<FVector, uint32>, 3> testBuffer;
    FVector vector_fit_a, vector_fit_b;
    testBuffer.put(std::make_pair(FVector(1, 1, 1), 1));
    testBuffer.put(std::make_pair(FVector(2, 2, 2), 2));
    testBuffer.put(std::make_pair(FVector(3, 3, 3), 3));
    MathToolkit::calculateLinearFit(testBuffer, 3, vector_fit_a, vector_fit_b);
    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector a: %f, %f, %f"), vector_fit_a.X, vector_fit_a.Y, vector_fit_a.Z);
    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector b: %f, %f, %f"), vector_fit_b.X, vector_fit_b.Y, vector_fit_b.Z);

    if (vector_fit_a.X != 1 || vector_fit_a.Y != 1 || vector_fit_a.Z != 1)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit a is not correct!"));
        return false;
    }
    if (vector_fit_b.X != 0 || vector_fit_b.Y != 0 || vector_fit_b.Z != 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit b is not correct!"));
        return false;
    }

    testBuffer.put(std::make_pair(FVector(4, 4, 4), 4));

    MathToolkit::calculateLinearFit(testBuffer, 3, vector_fit_a, vector_fit_b);
    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector a: %f, %f, %f"), vector_fit_a.X, vector_fit_a.Y, vector_fit_a.Z);
    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector b: %f, %f, %f"), vector_fit_b.X, vector_fit_b.Y, vector_fit_b.Z);

    if(vector_fit_a.X != 1 || vector_fit_a.Y != 1 || vector_fit_a.Z != 1)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit a is not correct!"));
        return false;
    }
    if(vector_fit_b.X != 0 || vector_fit_b.Y != 0 || vector_fit_b.Z != 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit b is not correct!"));
        return false;
    }

    testBuffer.put(std::make_pair(FVector(0, 0, 0), 6));
    testBuffer.put(std::make_pair(FVector(0, 0, 0), 7));
    testBuffer.put(std::make_pair(FVector(0, 0, 0), 8));

    MathToolkit::calculateLinearFit(testBuffer, 3, vector_fit_a, vector_fit_b);

    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector a: %f, %f, %f"), vector_fit_a.X, vector_fit_a.Y, vector_fit_a.Z);
    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector b: %f, %f, %f"), vector_fit_b.X, vector_fit_b.Y, vector_fit_b.Z);
    if(vector_fit_a.X != 0 || vector_fit_a.Y != 0 || vector_fit_a.Z != 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit a is not correct!"));
        return false;
    }
    if(vector_fit_b.X != 0 || vector_fit_b.Y != 0 || vector_fit_b.Z != 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit b is not correct!"));
        return false;
    }

    testBuffer.put(std::make_pair(FVector(-1, -1, -1), 9));
    testBuffer.put(std::make_pair(FVector(-1, -1, -1), 10));
    testBuffer.put(std::make_pair(FVector(-1, -1, -1), 11));
    
    MathToolkit::calculateLinearFit(testBuffer, 3, vector_fit_a, vector_fit_b);

    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector a: %f, %f, %f"), vector_fit_a.X, vector_fit_a.Y, vector_fit_a.Z);
    //UE_LOG(LogTemp, Warning, TEXT("Resulting direction Vector b: %f, %f, %f"), vector_fit_b.X, vector_fit_b.Y, vector_fit_b.Z);

    if(vector_fit_a.X != 0 || vector_fit_a.Y != 0 || vector_fit_a.Z != 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit a is not correct!"));
        return false;
    }
    if(vector_fit_b.X != -1 || vector_fit_b.Y != -1 || vector_fit_b.Z != -1)
    {
        UE_LOG(LogTemp, Error, TEXT("Linear fit b is not correct!"));
        return false;
    }
    

    return true;
}

#endif // WITH_AUTOMATION_TESTS