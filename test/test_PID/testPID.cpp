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
    double last = 0;
    double retVal = 0;
    StallardosPID testPID(0, 1, 0);
    for (uint8_t i = 0; i < 100; i++)
    {
        last += 10;
        retVal = testPID.calculate_pid(10, 0,1);
        if(last > 200)
        {
            TEST_ASSERT_EQUAL(200, retVal);
        }
        else
        {
            TEST_ASSERT_EQUAL(last, retVal);
        }
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

void test_default_constructor() {
    StallardosPID pid;
    double output = pid.calculate_pid(0.0, 0.0, 0.1);
    TEST_ASSERT_EQUAL(0.0, output);
}

void test_parameterized_constructor() {
    StallardosPID pid(1.0, 0.0, 0.0);
    double output = pid.calculate_pid(10.0, 0.0, 0.1);
    TEST_ASSERT_EQUAL(10.0, output);
}

void test_integral_action() {
    StallardosPID pid(0.0, 1.0, 0.0);
    double output1 = pid.calculate_pid(1.0, 0.0, 1.0);
    double output2 = pid.calculate_pid(1.0, 0.0, 1.0);
    TEST_ASSERT_TRUE(output2 > output1);
}

void test_derivative_action() {
    StallardosPID pid(0.0, 0.0, 1.0);
    double output1 = pid.calculate_pid(1.0, 0.0, 1.0);
    double output2 = pid.calculate_pid(1.0, 1.0, 1.0); // error change should be 0
    TEST_ASSERT_TRUE(output2 < output1);
}

void test_reset_function() {
    StallardosPID pid(0.0, 1.0, 0.0);
    pid.calculate_pid(1.0, 0.0, 1.0);
    pid.reset();
    double output = pid.calculate_pid(1.0, 0.0, 1.0);
    TEST_ASSERT_EQUAL(output, 1);  // Output should be small after reset
}

void setup()
{
    setUp();

    UNITY_BEGIN();

    RUN_TEST(test_PID_p);
    RUN_TEST(test_PID_i);
    RUN_TEST(test_PID_d);
    RUN_TEST(test_default_constructor);
    RUN_TEST(test_parameterized_constructor);
    RUN_TEST(test_integral_action);
    RUN_TEST(test_derivative_action);
    RUN_TEST(test_reset_function);

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