#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"

static void test_status_initial_off_inj1(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule1.Status);
}

static void test_status_initial_off_inj2(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule2.Status);
}

static void test_status_initial_off_inj3(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule3.Status);
}

static void test_status_initial_off_inj4(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule4.Status);
}

#if INJ_CHANNELS >= 5
static void test_status_initial_off_inj5(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule5.Status);
}
#endif

#if INJ_CHANNELS >= 6
static void test_status_initial_off_inj6(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule6.Status);
}
#endif

#if INJ_CHANNELS >= 7
static void test_status_initial_off_inj7(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule7.Status);
}
#endif

#if INJ_CHANNELS >= 8
static void test_status_initial_off_inj8(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule8.Status);
}
#endif
void test_status_initial_off(void)
{
    RUN_TEST(test_status_initial_off_inj1);
    RUN_TEST(test_status_initial_off_inj2);
    RUN_TEST(test_status_initial_off_inj3);
    RUN_TEST(test_status_initial_off_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_status_initial_off_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_status_initial_off_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_status_initial_off_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_status_initial_off_inj8);
#endif
}