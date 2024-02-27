
#include <Arduino.h>
#include <unity.h>

#include "ignition_contexts.h"
#include "scheduler.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000

static void injEmptyCallback(injector_id_t inj_id1, injector_id_t inj_id2)
{
    UNUSED(inj_id1);
    UNUSED(inj_id2);
}

static void test_status_running_to_pending_inj(FuelSchedule &fuelSchedule)
{
  initialiseSchedulers();
  fuelSchedule.start.pCallback = injEmptyCallback;
  fuelSchedule.end.pCallback = injEmptyCallback;
  setFuelSchedule(fuelSchedule, TIMEOUT, DURATION);
  while(fuelSchedule.Status == PENDING) /*Wait*/ ;
  /*
   * Quickly insert another scheduled event before the current one ends.
   * This second event should get placed into the pending queue.
   */
  setFuelSchedule(fuelSchedule, 2*TIMEOUT, DURATION);
  while(fuelSchedule.Status == RUNNING) /*Wait*/ ;
  TEST_ASSERT_EQUAL(PENDING, fuelSchedule.Status);
}

static void test_status_running_to_pending_inj1(void)
{
  test_status_running_to_pending_inj(fuelSchedule1);
}

static void test_status_running_to_pending_inj2(void)
{
  test_status_running_to_pending_inj(fuelSchedule2);
}

static void test_status_running_to_pending_inj3(void)
{
  test_status_running_to_pending_inj(fuelSchedule3);
}

static void test_status_running_to_pending_inj4(void)
{
  test_status_running_to_pending_inj(fuelSchedule4);
}

#if INJ_CHANNELS >= 5
static void test_status_running_to_pending_inj5(void)
{
  test_status_running_to_pending_inj(fuelSchedule5);
}
#endif

#if INJ_CHANNELS >= 6
static void test_status_running_to_pending_inj6(void)
{
  test_status_running_to_pending_inj(fuelSchedule6);
}
#endif

#if INJ_CHANNELS >= 7
static void test_status_running_to_pending_inj7(void)
{
  test_status_running_to_pending_inj(fuelSchedule7);
}
#endif

#if INJ_CHANNELS >= 8
static void test_status_running_to_pending_inj8(void)
{
  test_status_running_to_pending_inj(fuelSchedule8);
}
#endif

static void ignEmptyCallback(ignition_id_t coil_id1, ignition_id_t coil_id2)
{
    UNUSED(coil_id1);
    UNUSED(coil_id2);
}

static void test_status_running_to_pending_ign(IgnitionSchedule &ignitionSchedule)
{
    initialiseSchedulers();
    ignitionSchedule.start.pCallback = ignEmptyCallback;
    ignitionSchedule.end.pCallback = ignEmptyCallback;
    setIgnitionSchedule(ignitionSchedule, TIMEOUT, DURATION);
    while(ignitionSchedule.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule, 2*TIMEOUT, DURATION);
    while(ignitionSchedule.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule.Status);
}

static void test_status_running_to_pending_ign1(void)
{
  test_status_running_to_pending_ign(ignitionSchedule1);
}

static void test_status_running_to_pending_ign2(void)
{
  test_status_running_to_pending_ign(ignitionSchedule2);
}

static void test_status_running_to_pending_ign3(void)
{
  test_status_running_to_pending_ign(ignitionSchedule3);
}

static void test_status_running_to_pending_ign4(void)
{
  test_status_running_to_pending_ign(ignitionSchedule4);
}

#if IGN_CHANNELS >= 5
static void test_status_running_to_pending_ign5(void)
{
  test_status_running_to_pending_ign(ignitionSchedule5);
}
#endif

#if IGN_CHANNELS >= 6
static void test_status_running_to_pending_ign6(void)
{
  test_status_running_to_pending_ign(ignitionSchedule6);
}
#endif

#if IGN_CHANNELS >= 7
static void test_status_running_to_pending_ign7(void)
{
  test_status_running_to_pending_ign(ignitionSchedule7);
}
#endif

#if IGN_CHANNELS >= 8
static void test_status_running_to_pending_ign8(void)
{
  test_status_running_to_pending_ign(ignitionSchedule8);
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

    RUN_TEST(test_status_running_to_pending_ign1);
    RUN_TEST(test_status_running_to_pending_ign2);
    RUN_TEST(test_status_running_to_pending_ign3);
    RUN_TEST(test_status_running_to_pending_ign4);
#if IGN_CHANNELS >= 5
    RUN_TEST(test_status_running_to_pending_ign5);
#endif
#if IGN_CHANNELS >= 6
    RUN_TEST(test_status_running_to_pending_ign6);
#endif
#if IGN_CHANNELS >= 7
    RUN_TEST(test_status_running_to_pending_ign7);
#endif
#if IGN_CHANNELS >= 8
    RUN_TEST(test_status_running_to_pending_ign8);
#endif
}
