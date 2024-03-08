
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

static void test_status_pending_to_running_inj(FuelSchedule &fuelSchedule)
{
  initialiseSchedulers();
  fuelSchedule.start.pCallback = injEmptyCallback;
  fuelSchedule.end.pCallback = injEmptyCallback;
  uint32_t const start_time = micros();

  setFuelSchedule(fuelSchedule, TIMEOUT, DURATION);

  while (fuelSchedule.Status == PENDING)
  {
    /*
     * Ensure the test doesn't get stuck in this loop by waiting for a maximum
     * of twice the expected delay before the schedule starts.
     */
    int32_t const now = micros();

    if ((now - ((int32_t)start_time + 2 * TIMEOUT)) > 0)
    {
      break;
    }
    /*Wait*/
  }

  TEST_ASSERT_EQUAL(RUNNING, fuelSchedule.Status);
}

static void test_status_pending_to_running_inj1(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel1]);
}

static void test_status_pending_to_running_inj2(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel2]);
}

static void test_status_pending_to_running_inj3(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel3]);
}

static void test_status_pending_to_running_inj4(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel4]);
}

#if INJ_CHANNELS >= 5
static void test_status_pending_to_running_inj5(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel5]);
}
#endif

#if INJ_CHANNELS >= 6
static void test_status_pending_to_running_inj6(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel6]);
}
#endif

#if INJ_CHANNELS >= 7
static void test_status_pending_to_running_inj7(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel7]);
}
#endif

#if INJ_CHANNELS >= 8
static void test_status_pending_to_running_inj8(void)
{
  test_status_pending_to_running_inj(fuelSchedules[injChannel8]);
}
#endif
void test_status_pending_to_running(void)
{
    RUN_TEST(test_status_pending_to_running_inj1);
    RUN_TEST(test_status_pending_to_running_inj2);
    RUN_TEST(test_status_pending_to_running_inj3);
    RUN_TEST(test_status_pending_to_running_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_status_pending_to_running_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_status_pending_to_running_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_status_pending_to_running_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_status_pending_to_running_inj8);
#endif
}
