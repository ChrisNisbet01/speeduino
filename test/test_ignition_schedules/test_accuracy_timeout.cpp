
#include <Arduino.h>
#include <unity.h>
#include "globals.h"

#include "scheduler.h"
#include "ignition_control.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000
#define DELTA 40

static volatile uint32_t start_time;
static volatile uint32_t end_time;

static void ignStartCallback(ignition_id_t coil_id1, ignition_id_t coil_id2)
{
  UNUSED(coil_id1);
  UNUSED(coil_id2);

  end_time = micros();
}

static void ignEndCallback(ignition_id_t coil_id1, ignition_id_t coil_id2)
{
    UNUSED(coil_id1);
    UNUSED(coil_id2);

    /* Do nothing. */
}

static void test_accuracy_timeout_ign(IgnitionSchedule &schedule)
{
  initialiseSchedulers();

  schedule.start.pCallback = ignStartCallback;
  schedule.end.pCallback = ignEndCallback;
  start_time = micros();
  end_time = start_time;
  setIgnitionSchedule(schedule, TIMEOUT, DURATION);

  while(schedule.Status != RUNNING)
  {
    /*
     * Ensure the test doesn't get stuck in this loop by waiting for a maximum
     * of twice the expected delay before the schedule starts.
     */
    int32_t const now = micros();

    if ((now - ((int32_t)start_time + 2 * TIMEOUT)) > 0)
    {
      end_time = now;
      break;
    }
    /* Wait */
  }

  TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}

static void test_accuracy_timeout_ign1(void)
{
    test_accuracy_timeout_ign(ignitionSchedule1);
}

static void test_accuracy_timeout_ign2(void)
{
    test_accuracy_timeout_ign(ignitionSchedule2);
}

static void test_accuracy_timeout_ign3(void)
{
    test_accuracy_timeout_ign(ignitionSchedule3);
}

static void test_accuracy_timeout_ign4(void)
{
    test_accuracy_timeout_ign(ignitionSchedule4);
}

#if IGN_CHANNELS >= 5
static void test_accuracy_timeout_ign5(void)
{
    test_accuracy_timeout_ign(ignitionSchedule5);
}
#endif

#if IGN_CHANNELS >= 6
static void test_accuracy_timeout_ign6(void)
{
    test_accuracy_timeout_ign(ignitionSchedule6);
}
#endif

#if IGN_CHANNELS >= 7
static void test_accuracy_timeout_ign7(void)
{
    test_accuracy_timeout_ign(ignitionSchedule7);
}
#endif

#if IGN_CHANNELS >= 8
static void test_accuracy_timeout_ign8(void)
{
    test_accuracy_timeout_ign(ignitionSchedule8);
}
#endif

void test_accuracy_timeout(void)
{
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
