
#include <Arduino.h>
#include <unity.h>
#include "globals.h"

#include "injector_contexts.h"
#include "scheduler.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000
#define DELTA 40

static volatile uint32_t start_time;
static volatile uint32_t end_time;

static void injStartCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    UNUSED(inj_id1);
    UNUSED(inj_id2);

    end_time = micros();
}

static void injEndCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    UNUSED(inj_id1);
    UNUSED(inj_id2);

    /* Do nothing. */
}

static void test_accuracy_timeout_inj(FuelSchedule &schedule)
{
  initialiseSchedulers();

  schedule.start.pCallback = injStartCallback;
  schedule.end.pCallback = injEndCallback;
  start_time = micros();
  end_time = start_time;
  setFuelSchedule(schedule, TIMEOUT, DURATION);

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

static void test_accuracy_timeout_inj1(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel1]);
}

static void test_accuracy_timeout_inj2(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel2]);
}

static void test_accuracy_timeout_inj3(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel3]);
}

static void test_accuracy_timeout_inj4(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel4]);
}

#if INJ_CHANNELS >= 5
static void test_accuracy_timeout_inj5(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel5]);
}
#endif

#if INJ_CHANNELS >= 6
static void test_accuracy_timeout_inj6(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel6]);
}
#endif

#if INJ_CHANNELS >= 7
static void test_accuracy_timeout_inj7(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel7]);
}
#endif

#if INJ_CHANNELS >= 8
static void test_accuracy_timeout_inj8(void)
{
    test_accuracy_timeout_inj(fuelSchedules[injChannel8]);
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
}
