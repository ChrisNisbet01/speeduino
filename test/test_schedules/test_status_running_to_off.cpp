
#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000

static void injEmptyCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    UNUSED(inj_id1);
    UNUSED(inj_id2);
}

void test_status_running_to_off_inj(FuelSchedule &fuelSchedule)
{
    initialiseSchedulers();
    uint32_t const start_time = micros();
    fuelSchedule.start.pCallback = injEmptyCallback;
    fuelSchedule.end.pCallback = injEmptyCallback;
    setFuelSchedule(fuelSchedule, TIMEOUT, DURATION);
    while (fuelSchedule.Status == PENDING || fuelSchedule.Status == RUNNING)
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
    TEST_ASSERT_EQUAL(OFF, fuelSchedule1.Status);
}

void test_status_running_to_off_inj1(void)
{
    test_status_running_to_off_inj(fuelSchedule1);
}

void test_status_running_to_off_inj2(void)
{
    test_status_running_to_off_inj(fuelSchedule2);
}

void test_status_running_to_off_inj3(void)
{
    test_status_running_to_off_inj(fuelSchedule3);
}

void test_status_running_to_off_inj4(void)
{
    test_status_running_to_off_inj(fuelSchedule4);
}

#if INJ_CHANNELS >= 5
void test_status_running_to_off_inj5(void)
{
    test_status_running_to_off_inj(fuelSchedule5);
}
#endif

#if INJ_CHANNELS >= 6
void test_status_running_to_off_inj6(void)
{
    test_status_running_to_off_inj(fuelSchedule6);
}
#endif

#if INJ_CHANNELS >= 7
void test_status_running_to_off_inj7(void)
{
    test_status_running_to_off_inj(fuelSchedule7);
}
#endif

#if INJ_CHANNELS >= 8
void test_status_running_to_off_inj8(void)
{
    test_status_running_to_off_inj(fuelSchedule8);
}
#endif


static void ignEmptyCallback(ignition_id_t coil_id1, ignition_id_t coil_id2)
{
    UNUSED(coil_id1);
    UNUSED(coil_id2);
}

void test_status_running_to_off_ign(IgnitionSchedule &ignitionSchedule)
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

void test_status_running_to_off_ign1(void)
{
    test_status_running_to_off_ign(ignitionSchedule1);
}

void test_status_running_to_off_ign2(void)
{
    test_status_running_to_off_ign(ignitionSchedule2);
}

void test_status_running_to_off_ign3(void)
{
    test_status_running_to_off_ign(ignitionSchedule3);
}

void test_status_running_to_off_ign4(void)
{
    test_status_running_to_off_ign(ignitionSchedule4);
}

#if IGN_CHANNELS >= 5
void test_status_running_to_off_ign5(void)
{
    test_status_running_to_off_ign(ignitionSchedule5);
}
#endif

#if IGN_CHANNELS >= 6
void test_status_running_to_off_ign6(void)
{
    test_status_running_to_off_ign(ignitionSchedule6);
}
#endif

#if IGN_CHANNELS >= 7
void test_status_running_to_off_ign7(void)
{
    test_status_running_to_off_ign(ignitionSchedule7);
}
#endif

#if IGN_CHANNELS >= 8
void test_status_running_to_off_ign8(void)
{
    test_status_running_to_off_ign(ignitionSchedule8);
}
#endif

void test_status_running_to_off(void)
{
    RUN_TEST(test_status_running_to_off_inj1);
    RUN_TEST(test_status_running_to_off_inj2);
    RUN_TEST(test_status_running_to_off_inj3);
    RUN_TEST(test_status_running_to_off_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_status_running_to_off_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_status_running_to_off_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_status_running_to_off_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_status_running_to_off_inj8);
#endif

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
