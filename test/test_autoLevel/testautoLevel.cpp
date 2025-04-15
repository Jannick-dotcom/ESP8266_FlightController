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
    for(int32_t i = -9000; i < 9000; i++)
    {
        anglePitch = i/100.0;
        berechnen();
        TEST_ASSERT_EQUAL(i,pid_pitch_setpoint);
    }

}

void test_2(void)
{
    double last = 10;
    double retVal = 0;
    StallardosPID testPID(0, 1, 0);
    for (uint8_t i = 0; i < 10; i++)
    {
        retVal = testPID.calculate_pid(10, 0, 1);
        TEST_ASSERT_EQUAL(last, retVal);
        last += 10;
    }
}

void test_3(void)
{
    double retVal = 0;
    StallardosPID testPID(0, 0, 1);
    retVal = testPID.calculate_pid(10, 0, 1);
    TEST_ASSERT_EQUAL(10, retVal);
    retVal = testPID.calculate_pid(0, 0, 1);
    TEST_ASSERT_EQUAL(-10, retVal);
    retVal = testPID.calculate_pid(0,0, 1);
    TEST_ASSERT_EQUAL(0,retVal);
}

void setup()
{
    setUp();

    UNITY_BEGIN();

    RUN_TEST(test_1);
    // RUN_TEST(test_PID_i);
    // RUN_TEST(test_PID_d);

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