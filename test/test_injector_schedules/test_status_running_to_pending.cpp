
#include <Arduino.h>
#include <unity.h>

#include "fuel_scheduler.h"
#include "scheduler.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000

static void injEmptyCallback(void)
{
  /* Do nothing. */
}

static void test_status_running_to_pending_inj(FuelSchedule &fuelSchedule)
{
  initialiseSchedulers();
  fuelSchedule.start.pCallback = injEmptyCallback;
  fuelSchedule.end.pCallback = injEmptyCallback;
  setFuelSchedule(fuelSchedule, TIMEOUT, DURATION);
  uint32_t start_time;

  start_time = micros();
  while (fuelSchedule.Status != RUNNING)
  {
    if (micros() - start_time > TIMEOUT * 2)
    {
      break;
    }
    /* Wait. */
  }
  TEST_ASSERT_EQUAL(RUNNING, fuelSchedule.Status);
  /*
   * Quickly insert another scheduled event before the current one ends.
   * This second event should get placed into the pending queue.
   */
  setFuelSchedule(fuelSchedule, DURATION + TIMEOUT, DURATION);

  start_time = micros();
  while (fuelSchedule.Status == RUNNING)
  {
    if (micros() - start_time > DURATION * 2)
    {
      break;
    }
  }
  TEST_ASSERT_EQUAL(PENDING, fuelSchedule.Status);
}

static void test_status_running_to_pending_inj1(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel1]);
}

static void test_status_running_to_pending_inj2(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel2]);
}

static void test_status_running_to_pending_inj3(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel3]);
}

static void test_status_running_to_pending_inj4(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel4]);
}

#if INJ_CHANNELS >= 5
static void test_status_running_to_pending_inj5(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel5]);
}
#endif

#if INJ_CHANNELS >= 6
static void test_status_running_to_pending_inj6(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel6]);
}
#endif

#if INJ_CHANNELS >= 7
static void test_status_running_to_pending_inj7(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel7]);
}
#endif

#if INJ_CHANNELS >= 8
static void test_status_running_to_pending_inj8(void)
{
  test_status_running_to_pending_inj(fuelSchedules[injChannel8]);
}
#endif

void test_status_running_to_pending(void)
{
    RUN_TEST(test_status_running_to_pending_inj1);
    RUN_TEST(test_status_running_to_pending_inj2);
    RUN_TEST(test_status_running_to_pending_inj3);
    RUN_TEST(test_status_running_to_pending_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_status_running_to_pending_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_status_running_to_pending_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_status_running_to_pending_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_status_running_to_pending_inj8);
#endif
}

