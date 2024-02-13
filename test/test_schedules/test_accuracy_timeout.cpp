
#include <Arduino.h>
#include <unity.h>

#include "injector_contexts.h"
#include "scheduler.h"
#include "ignition_control.h"

#define TIMEOUT 1000
#define DURATION 1000
#define DELTA 24

static uint32_t start_time, end_time;

static void startCallback(ignition_id_t coil_id1, ignition_id_t coil_id2)
{
  end_time = micros();
}

static void endCallback(ignition_id_t coil_id1, ignition_id_t coil_id2)
{
  /*Empty*/
}

static void injStartCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    start_time = micros();
}

static void injEndCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    end_time = micros();
}

void test_accuracy_timeout_inj(FuelSchedule &schedule)
{
    initialiseSchedulers();
    schedule.start.pCallback = injStartCallback;
    schedule.end.pCallback = injEndCallback;
    start_time = micros();
    setFuelSchedule(schedule, TIMEOUT, DURATION);
    while(schedule.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}

void test_accuracy_timeout_inj1(void)
{
    test_accuracy_timeout_inj(fuelSchedule1);
}

void test_accuracy_timeout_inj2(void)
{
    test_accuracy_timeout_inj(fuelSchedule2);
}

void test_accuracy_timeout_inj3(void)
{
    test_accuracy_timeout_inj(fuelSchedule3);
}

void test_accuracy_timeout_inj4(void)
{
    test_accuracy_timeout_inj(fuelSchedule4);
}

#if INJ_CHANNELS >= 5
void test_accuracy_timeout_inj5(void)
{
    test_accuracy_timeout_inj(fuelSchedule5);
}
#endif

#if INJ_CHANNELS >= 6
void test_accuracy_timeout_inj6(void)
{
    test_accuracy_timeout_inj(fuelSchedule6);
}
#endif

#if INJ_CHANNELS >= 7
void test_accuracy_timeout_inj7(void)
{
    test_accuracy_timeout_inj(fuelSchedule7);
}
#endif

#if INJ_CHANNELS >= 8
void test_accuracy_timeout_inj8(void)
{
    test_accuracy_timeout_inj(fuelSchedule8);
}
#endif

void test_accuracy_timeout_ign(IgnitionSchedule &schedule)
{
    initialiseSchedulers();
    schedule.start.pCallback = startCallback;
    schedule.end.pCallback = endCallback;
    start_time = micros();
    setIgnitionSchedule(schedule, TIMEOUT, DURATION);
    while(schedule.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}

void test_accuracy_timeout_ign1(void)
{
    test_accuracy_timeout_ign(ignitionSchedule1);
}

void test_accuracy_timeout_ign2(void)
{
    test_accuracy_timeout_ign(ignitionSchedule2);
}

void test_accuracy_timeout_ign3(void)
{
    test_accuracy_timeout_ign(ignitionSchedule3);
}

void test_accuracy_timeout_ign4(void)
{
    test_accuracy_timeout_ign(ignitionSchedule4);
}

#if IGN_CHANNELS >= 5
void test_accuracy_timeout_ign5(void)
{
    test_accuracy_timeout_ign(ignitionSchedule5);
}
#endif

#if IGN_CHANNELS >= 6
void test_accuracy_timeout_ign6(void)
{
    test_accuracy_timeout_ign(ignitionSchedule6);
}
#endif

#if IGN_CHANNELS >= 7
void test_accuracy_timeout_ign7(void)
{
    test_accuracy_timeout_ign(ignitionSchedule7);
}
#endif

#if IGN_CHANNELS >= 8
void test_accuracy_timeout_ign8(void)
{
    test_accuracy_timeout_ign(ignitionSchedule8);
}
#endif

void test_accuracy_timeout(void)
{
    RUN_TEST(test_accuracy_timeout_inj1);
    RUN_TEST(test_accuracy_timeout_inj2);
    RUN_TEST(test_accuracy_timeout_inj3);
    RUN_TEST(test_accuracy_timeout_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_accuracy_timeout_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_accuracy_timeout_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_accuracy_timeout_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_accuracy_timeout_inj8);
#endif

    RUN_TEST(test_accuracy_timeout_ign1);
    RUN_TEST(test_accuracy_timeout_ign2);
    RUN_TEST(test_accuracy_timeout_ign3);
    RUN_TEST(test_accuracy_timeout_ign4);
#if IGN_CHANNELS >= 5
    RUN_TEST(test_accuracy_timeout_ign5);
#endif
#if IGN_CHANNELS >= 6
    RUN_TEST(test_accuracy_timeout_ign6);
#endif
#if IGN_CHANNELS >= 7
    RUN_TEST(test_accuracy_timeout_ign7);
#endif
#if IGN_CHANNELS >= 8
    RUN_TEST(test_accuracy_timeout_ign8);
#endif
}
