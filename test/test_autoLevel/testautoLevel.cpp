#include <unity.h>
#include "mixer.hpp"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_1(void)
{
    controlMode = 0;
    Arming = 2000;
    Throttle = 1500;
    Mode = 2000;
    Roll = 1500;
    Pitch = 1500;
    gyroX = gyroY = gyroZ = 0;
    for(int32_t i = -9000; i <= 9000; i++)
    {
        anglePitch = i/100.0;
        float ref = (Pitch - 1500) - anglePitch * 15;
        berechnen();
        TEST_ASSERT_EQUAL(ref,pid_pitch_setpoint);
    }
}

void setup()
{
    setUp();

    UNITY_BEGIN();

    RUN_TEST(test_1);

    UNITY_END();
}
void loop()
{
}

#ifdef NATIVE
int main()
{
    setup();
}
#endif