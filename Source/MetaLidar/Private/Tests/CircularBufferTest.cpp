#include "Misc/AutomationTest.h"

#if WITH_AUTOMATION_TESTS

#ifdef UNIT_TEST_FRIEND
#undef UNIT_TEST_FRIEND
#endif
#define UNIT_TEST_FRIEND friend class CircularBufferTest;

#include "CircularBuffer/CircularBuffer.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(CircularBufferTest, "MetaLidar.CircularBuffer", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool CircularBufferTest::RunTest(const FString &Parameters)
{
    CircularBuffer<uint32, 3> testBuffer;

    testBuffer.put(1);
    testBuffer.put(2);
    testBuffer.put(3);
    //UE_LOG(LogTemp, Warning, TEXT("Put 1, 2, 3"));
    if (testBuffer.size() != 3)
    {
        UE_LOG(LogTemp, Error, TEXT("Buffer size is not 3!"));
        return false;
    }
    std::array<uint32, 3> buffer = testBuffer.get_all();
    for (int i = 0; i < 3; i++)
    {
        if (buffer[i] != i + 1)
        {
            UE_LOG(LogTemp, Error, TEXT("Buffer is not correct! on index %d value should be %d actually %d"), i, i + 1, buffer[i]);
            return false;
        }
    }

    testBuffer.put(4);
    //UE_LOG(LogTemp, Warning, TEXT("Put 4"));
    if (testBuffer.size() != 3)
    {
        UE_LOG(LogTemp, Error, TEXT("Buffer size is not 3!"));
        return false;
    }
    buffer = testBuffer.get_all();

    for (int i = 0; i < 3; i++)
    {
        if (buffer[i] != i + 2)
        {
            UE_LOG(LogTemp, Error, TEXT("Buffer is not correct! on index %d value should be %d actually %d"), i, i + 2, buffer[i]);
            return false;
        }
    }

    return true;
}

#endif // WITH_AUTOMATION_TESTS