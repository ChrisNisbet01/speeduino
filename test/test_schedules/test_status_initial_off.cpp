#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"

void test_status_initial_off_inj1(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule1.Status);
}

void test_status_initial_off_inj2(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule2.Status);
}

void test_status_initial_off_inj3(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule3.Status);
}

void test_status_initial_off_inj4(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule4.Status);
}

#if INJ_CHANNELS >= 5
void test_status_initial_off_inj5(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule5.Status);
}
#endif

#if INJ_CHANNELS >= 6
void test_status_initial_off_inj6(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule6.Status);
}
#endif

#if INJ_CHANNELS >= 7
void test_status_initial_off_inj7(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule7.Status);
}
#endif

#if INJ_CHANNELS >= 8
void test_status_initial_off_inj8(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule8.Status);
}
#endif


void test_status_initial_off_ign1(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule1.Status);
}

void test_status_initial_off_ign2(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule2.Status);
}

void test_status_initial_off_ign3(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule3.Status);
}

void test_status_initial_off_ign4(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule4.Status);
}

#if IGN_CHANNELS >= 5
void test_status_initial_off_ign5(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule5.Status);
}
#endif

#if IGN_CHANNELS >= 6
void test_status_initial_off_ign6(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule6.Status);
}
#endif

#if IGN_CHANNELS >= 7
void test_status_initial_off_ign7(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule7.Status);
}
#endif

#if IGN_CHANNELS >= 8
void test_status_initial_off_ign8(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule8.Status);
}
#endif

void test_status_initial_off(void)
{
    RUN_TEST(test_status_initial_off_inj1);
    delay(250);
    RUN_TEST(test_status_initial_off_inj2);
    delay(250);
    RUN_TEST(test_status_initial_off_inj3);
    delay(250);
    RUN_TEST(test_status_initial_off_inj4);
    delay(250);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_status_initial_off_inj5);
    delay(250);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_status_initial_off_inj6);
    delay(250);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_status_initial_off_inj7);
    delay(250);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_status_initial_off_inj8);
    delay(250);
#endif

    RUN_TEST(test_status_initial_off_ign1);
    delay(250);
    RUN_TEST(test_status_initial_off_ign2);
    delay(250);
    RUN_TEST(test_status_initial_off_ign3);
    delay(250);
    RUN_TEST(test_status_initial_off_ign4);
    delay(250);
#if IGN_CHANNELS >= 5
    RUN_TEST(test_status_initial_off_ign5);
    delay(250);
#endif
#if IGN_CHANNELS >= 6
    RUN_TEST(test_status_initial_off_ign6);
    delay(250);
#endif
#if IGN_CHANNELS >= 7
    RUN_TEST(test_status_initial_off_ign7);
    delay(250);
#endif
#if IGN_CHANNELS >= 8
    RUN_TEST(test_status_initial_off_ign8);
    delay(250);
#endif
}