
#include <Arduino.h>
#include <unity.h>

#include "ignition_scheduler.h"
#include "scheduler.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000
#define DELTA 20

static volatile uint32_t start_time;
static volatile uint32_t end_time;

static void ignStartCallback(void)
{
    start_time = micros();
}

static void ignEndCallback()
{
    end_time = micros();
}

static void test_accuracy_duration_ign(IgnitionSchedule &schedule)
{
    initialiseSchedulers();
    schedule.start.pCallback = ignStartCallback;
    schedule.end.pCallback = ignEndCallback;
    setIgnitionSchedule(schedule, TIMEOUT, DURATION);
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

static void test_accuracy_duration_ign1(void)
{
  test_accuracy_duration_ign(ignitionSchedules[ignChannel1]);
}

static void test_accuracy_duration_ign2(void)
{
    test_accuracy_duration_ign(ignitionSchedules[ignChannel2]);
}

static void test_accuracy_duration_ign3(void)
{
    test_accuracy_duration_ign(ignitionSchedules[ignChannel3]);
}

static void test_accuracy_duration_ign4(void)
{
    test_accuracy_duration_ign(ignitionSchedules[ignChannel4]);
}

#if IGN_CHANNELS >= 5
static void test_accuracy_duration_ign5(void)
{
    test_accuracy_duration_ign(ignitionSchedules[ignChannel5]);
}
#endif

#if IGN_CHANNELS >= 6
static void test_accuracy_duration_ign6(void)
{
    test_accuracy_duration_ign(ignitionSchedules[ignChannel6]);
}
#endif

#if IGN_CHANNELS >= 7
static void test_accuracy_duration_ign7(void)
{
    test_accuracy_duration_ign(ignitionSchedules[ignChannel7]);
}
#endif

#if IGN_CHANNELS >= 8
static void test_accuracy_duration_ign8(void)
{
    test_accuracy_duration_ign(ignitionSchedules[ignChannel8]);
}
#endif

void test_accuracy_duration(void)
{
    RUN_TEST(test_accuracy_duration_ign1);
    RUN_TEST(test_accuracy_duration_ign2);
    RUN_TEST(test_accuracy_duration_ign3);
    RUN_TEST(test_accuracy_duration_ign4);
#if IGN_CHANNELS >= 5
    RUN_TEST(test_accuracy_duration_ign5);
#endif
#if IGN_CHANNELS >= 6
    RUN_TEST(test_accuracy_duration_ign6);
#endif
#if IGN_CHANNELS >= 7
    RUN_TEST(test_accuracy_duration_ign7);
#endif
#if IGN_CHANNELS >= 8
    RUN_TEST(test_accuracy_duration_ign8);
#endif
}

