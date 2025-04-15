#include <unity.h>
#include "StallardOSPID.hpp"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_PID_p(void)
{
    StallardosPID testPID(1, 0, 0);
    double retVal = testPID.calculate_pid(10, 0, 1);
    TEST_ASSERT_EQUAL(10, retVal);
}

void test_PID_i(void)
{
    double last = 10;
    double retVal = 0;
    StallardosPID testPID(0, 1, 0);
    for (uint8_t i = 0; i < 10; i++)
    {
        retVal = testPID.calculate_pid(10, 0,1);
        TEST_ASSERT_EQUAL(last, retVal);
        last += 10;
    }
}

void test_PID_d(void)
{
    double retVal = 0;
    StallardosPID testPID(0, 0, 1);
    retVal = testPID.calculate_pid(10, 0,1);
    TEST_ASSERT_EQUAL(10, retVal);
    retVal = testPID.calculate_pid(0, 0,1);
    TEST_ASSERT_EQUAL(-10, retVal);
    retVal = testPID.calculate_pid(0,0,1);
    TEST_ASSERT_EQUAL(0,retVal);
}

void setup()
{
    setUp();

    UNITY_BEGIN();

    RUN_TEST(test_PID_p);
    RUN_TEST(test_PID_i);
    RUN_TEST(test_PID_d);

    UNITY_END();
}
void loop()
{
    return;
}

#ifdef NATIVE
int main()
{
    setup();
}
#endif