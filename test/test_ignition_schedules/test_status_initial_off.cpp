#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"

static void test_status_initial_off_ign1(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel1].Status);
}

static void test_status_initial_off_ign2(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel2].Status);
}

static void test_status_initial_off_ign3(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel3].Status);
}

static void test_status_initial_off_ign4(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel4].Status);
}

#if IGN_CHANNELS >= 5
static void test_status_initial_off_ign5(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel5].Status);
}
#endif

#if IGN_CHANNELS >= 6
static void test_status_initial_off_ign6(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel6].Status);
}
#endif

#if IGN_CHANNELS >= 7
static void test_status_initial_off_ign7(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel7].Status);
}
#endif

#if IGN_CHANNELS >= 8
static void test_status_initial_off_ign8(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedules[ignChannel8].Status);
}
#endif

void test_status_initial_off(void)
{
    RUN_TEST(test_status_initial_off_ign1);
    RUN_TEST(test_status_initial_off_ign2);
    RUN_TEST(test_status_initial_off_ign3);
    RUN_TEST(test_status_initial_off_ign4);
#if IGN_CHANNELS >= 5
    RUN_TEST(test_status_initial_off_ign5);
#endif
#if IGN_CHANNELS >= 6
    RUN_TEST(test_status_initial_off_ign6);
#endif
#if IGN_CHANNELS >= 7
    RUN_TEST(test_status_initial_off_ign7);
#endif
#if IGN_CHANNELS >= 8
    RUN_TEST(test_status_initial_off_ign8);
#endif
}