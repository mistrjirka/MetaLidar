#pragma once

#include "CoreMinimal.h"
#ifndef UNIT_TEST_FRIEND
#define UNIT_TEST_FRIEND
#endif

class CircularBufferTest : public CircularBuffer<int, 3>
{
    public:
        CircularBufferTest();
    private:
        UNIT_TEST_FRIEND;

};