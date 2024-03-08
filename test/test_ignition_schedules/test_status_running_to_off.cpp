
#include <Arduino.h>
#include <unity.h>

#include "ignition_scheduler.h"
#include "scheduler.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000

static void ignEmptyCallback(void)
{
  /* Do nothing. */
}

static void test_status_running_to_off_ign(IgnitionSchedule &ignitionSchedule)
{
  initialiseSchedulers();
  ignitionSchedule.start.pCallback = ignEmptyCallback;
  ignitionSchedule.end.pCallback = ignEmptyCallback;
  uint32_t const start_time = micros();

  setIgnitionSchedule(ignitionSchedule, TIMEOUT, DURATION);
  while (ignitionSchedule.Status == PENDING || ignitionSchedule.Status == RUNNING)
  {
    /*
     * Ensure the test doesn't get stuck in this loop by waiting for a maximum
     * of twice the expected delay before the schedule starts.
     */
    int32_t const now = micros();

    if ((now - ((int32_t)start_time + 2 * (TIMEOUT + DURATION))) > 0)
    {
      break;
    }
    /*Wait*/
  }
  TEST_ASSERT_EQUAL(OFF, ignitionSchedule.Status);
}

static void test_status_running_to_off_ign1(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel1]);
}

static void test_status_running_to_off_ign2(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel2]);
}

static void test_status_running_to_off_ign3(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel3]);
}

static void test_status_running_to_off_ign4(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel4]);
}

#if IGN_CHANNELS >= 5
static void test_status_running_to_off_ign5(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel5]);
}
#endif

#if IGN_CHANNELS >= 6
static void test_status_running_to_off_ign6(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel6]);
}
#endif

#if IGN_CHANNELS >= 7
static void test_status_running_to_off_ign7(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel7]);
}
#endif

#if IGN_CHANNELS >= 8
static void test_status_running_to_off_ign8(void)
{
  test_status_running_to_off_ign(ignitionSchedules[ignChannel8]);
}
#endif

void test_status_running_to_off(void)
{
  RUN_TEST(test_status_running_to_off_ign1);
  RUN_TEST(test_status_running_to_off_ign2);
  RUN_TEST(test_status_running_to_off_ign3);
  RUN_TEST(test_status_running_to_off_ign4);
#if IGN_CHANNELS >= 5
  RUN_TEST(test_status_running_to_off_ign5);
#endif
#if IGN_CHANNELS >= 6
  RUN_TEST(test_status_running_to_off_ign6);
#endif
#if IGN_CHANNELS >= 7
  RUN_TEST(test_status_running_to_off_ign7);
#endif
#if IGN_CHANNELS >= 8
  RUN_TEST(test_status_running_to_off_ign8);
#endif
}
