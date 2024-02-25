
#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"
#include "utilities.h"

#define TIMEOUT 1000
#define DURATION 1000

static void emptyCallback(ignition_id_t coil_id1, ignition_id_t coil_id2)
{
    UNUSED(coil_id1);
    UNUSED(coil_id2);
}

void test_status_running_to_off_inj1(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule1, TIMEOUT, DURATION);

    uint32_t const micros_start = micros();

    while (fuelSchedule1.Status == PENDING || fuelSchedule1.Status == RUNNING)
    {
      if (micros() - (micros_start + 1000000) > 0)
      {
        break;
      }
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule1.Status);
}

void test_status_running_to_off_inj2(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule2, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((fuelSchedule2.Status == PENDING) || (fuelSchedule2.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule2.Status);
}

void test_status_running_to_off_inj3(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule3, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((fuelSchedule3.Status == PENDING) || (fuelSchedule3.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule3.Status);
}

void test_status_running_to_off_inj4(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule4, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((fuelSchedule4.Status == PENDING) || (fuelSchedule4.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule4.Status);
}

void test_status_running_to_off_inj5(void)
{
#if INJ_CHANNELS >= 5
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule5, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((fuelSchedule5.Status == PENDING) || (fuelSchedule5.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule5.Status);
#endif
}

void test_status_running_to_off_inj6(void)
{
#if INJ_CHANNELS >= 6
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule6, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((fuelSchedule6.Status == PENDING) || (fuelSchedule6.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule6.Status);
#endif
}

void test_status_running_to_off_inj7(void)
{
#if INJ_CHANNELS >= 7
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule7, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((fuelSchedule7.Status == PENDING) || (fuelSchedule7.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule7.Status);
#endif
}

void test_status_running_to_off_inj8(void)
{
#if INJ_CHANNELS >= 8
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule8, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((fuelSchedule8.Status == PENDING) || (fuelSchedule8.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, fuelSchedule8.Status);
#endif
}


void test_status_running_to_off_ign1(void)
{
    initialiseSchedulers();
    ignitionSchedule1.start.pCallback = emptyCallback;
    ignitionSchedule1.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule1, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule1.Status == PENDING) || (ignitionSchedule1.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule1.Status);
}

void test_status_running_to_off_ign2(void)
{
    initialiseSchedulers();
    ignitionSchedule2.start.pCallback = emptyCallback;
    ignitionSchedule2.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule2, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule2.Status == PENDING) || (ignitionSchedule2.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule2.Status);
}

void test_status_running_to_off_ign3(void)
{
    initialiseSchedulers();
    ignitionSchedule3.start.pCallback = emptyCallback;
    ignitionSchedule3.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule3, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule3.Status == PENDING) || (ignitionSchedule3.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule3.Status);
}

void test_status_running_to_off_ign4(void)
{
    initialiseSchedulers();
    ignitionSchedule4.start.pCallback = emptyCallback;
    ignitionSchedule4.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule4, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule4.Status == PENDING) || (ignitionSchedule4.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule4.Status);
}

void test_status_running_to_off_ign5(void)
{
#if IGN_CHANNELS >= 5
    initialiseSchedulers();
    ignitionSchedule5.start.pCallback = emptyCallback;
    ignitionSchedule5.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule5, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule5.Status == PENDING) || (ignitionSchedule5.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule5.Status);
#endif
}

void test_status_running_to_off_ign6(void)
{
#if IGN_CHANNELS >= 6
    initialiseSchedulers();
    ignitionSchedule6.start.pCallback = emptyCallback;
    ignitionSchedule6.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule6, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule6.Status == PENDING) || (ignitionSchedule6.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule6.Status);
#endif
}

void test_status_running_to_off_ign7(void)
{
#if IGN_CHANNELS >= 7
    initialiseSchedulers();
    ignitionSchedule7.start.pCallback = emptyCallback;
    ignitionSchedule7.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule7, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule7.Status == PENDING) || (ignitionSchedule7.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule7.Status);
#endif
}

void test_status_running_to_off_ign8(void)
{
#if IGN_CHANNELS >= 8
    initialiseSchedulers();
    ignitionSchedule8.start.pCallback = emptyCallback;
    ignitionSchedule8.end.pCallback = emptyCallback;
    setIgnitionSchedule(ignitionSchedule8, TIMEOUT, DURATION);
    uint32_t const micros_start = micros();

    while ((ignitionSchedule8.Status == PENDING) || (ignitionSchedule8.Status == RUNNING))
    {
      if (micros() > micros_start + 1000000)
      {
        break;
      }
      /*Wait*/;
    }
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule8.Status);
#endif
}

void test_status_running_to_off(void)
{
    RUN_TEST(test_status_running_to_off_inj1);
#if 0
    RUN_TEST(test_status_running_to_off_inj2);
    RUN_TEST(test_status_running_to_off_inj3);
    RUN_TEST(test_status_running_to_off_inj4);
    RUN_TEST(test_status_running_to_off_inj5);
    RUN_TEST(test_status_running_to_off_inj6);
    RUN_TEST(test_status_running_to_off_inj7);
    RUN_TEST(test_status_running_to_off_inj8);

    RUN_TEST(test_status_running_to_off_ign1);
    RUN_TEST(test_status_running_to_off_ign2);
    RUN_TEST(test_status_running_to_off_ign3);
    RUN_TEST(test_status_running_to_off_ign4);
    RUN_TEST(test_status_running_to_off_ign5);
    RUN_TEST(test_status_running_to_off_ign6);
    RUN_TEST(test_status_running_to_off_ign7);
    RUN_TEST(test_status_running_to_off_ign8);
#endif
}
