
#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"
#include "injector_contexts.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000
#define DELTA 20

static volatile uint32_t start_time;
static volatile uint32_t end_time;

static void injStartCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    UNUSED(inj_id1);
    UNUSED(inj_id2);

    start_time = micros();
}

static void injEndCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    UNUSED(inj_id1);
    UNUSED(inj_id2);

    end_time = micros();
}

static void test_accuracy_duration_inj(FuelSchedule &schedule)
{
    initialiseSchedulers();
    schedule.start.pCallback = injStartCallback;
    schedule.end.pCallback = injEndCallback;
    setFuelSchedule(schedule, TIMEOUT, DURATION);
    uint32_t const loop_start_time = micros();

    while(schedule.Status != OFF)
    {
      /*
       * Ensure the test doesn't get stuck in this loop by waiting for a maximum
       * of twice the expected delay before the schedule starts.
       */
      int32_t const now = micros();

      if ((now - ((int32_t)loop_start_time + 2 * (TIMEOUT + DURATION))) > 0)
      {
        break;
      }
      /* Wait */
    }

    TEST_ASSERT_EQUAL(OFF, schedule.Status);
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}

static void test_accuracy_duration_inj1(void)
{
    test_accuracy_duration_inj(fuelSchedule1);
}

static void test_accuracy_duration_inj2(void)
{
    test_accuracy_duration_inj(fuelSchedule2);
}

static void test_accuracy_duration_inj3(void)
{
    test_accuracy_duration_inj(fuelSchedule3);
}

static void test_accuracy_duration_inj4(void)
{
    test_accuracy_duration_inj(fuelSchedule4);
}

#if INJ_CHANNELS >= 5
static void test_accuracy_duration_inj5(void)
{
    test_accuracy_duration_inj(fuelSchedule5);
}
#endif

#if INJ_CHANNELS >= 6
static void test_accuracy_duration_inj6(void)
{
    test_accuracy_duration_inj(fuelSchedule6);
}
#endif

#if INJ_CHANNELS >= 7
static void test_accuracy_duration_inj7(void)
{
    test_accuracy_duration_inj(fuelSchedule7);
}
#endif

#if INJ_CHANNELS >= 8
static void test_accuracy_duration_inj8(void)
{
    test_accuracy_duration_inj(fuelSchedule8);
}
#endif

void test_accuracy_duration(void)
{
    RUN_TEST(test_accuracy_duration_inj1);
    RUN_TEST(test_accuracy_duration_inj2);
    RUN_TEST(test_accuracy_duration_inj3);
    RUN_TEST(test_accuracy_duration_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_accuracy_duration_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_accuracy_duration_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_accuracy_duration_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_accuracy_duration_inj8);
#endif
}
