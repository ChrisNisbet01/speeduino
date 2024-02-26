#include <Arduino.h>
#include <unity.h>
#include <init.h>
#include "globals.h"
#include "test_schedules.h"
#include "scheduler.h"
#include "injector_contexts.h"
#include "injector_schedule.h"
#include "src/decoders/missing_tooth.h"

static void prepareForInitialiseAll(void)
{
    /*
       Avoid some divide-by-zero operations that would occur if the following
       variables are not initialised.
    */
    configPage6.boostFreq = 10;
    configPage6.vvtFreq = 10;
    configPage6.idleFreq = 10;
#if defined(PWM_FAN_AVAILABLE)
    configPage6.fanFreq = 10;
#endif
}

int trigger_pattern;
volatile uint32_t interrupt_running;

void test_debug(void)
{
  //TEST_ASSERT_EQUAL(100, trigger_pattern);
  TEST_ASSERT_EQUAL(0, interrupt_running);
}

void test_debug2(void)
{
    initialiseSchedulers();
    //fuelSchedule1.pTimerEnable =  FUEL1_TIMER_ENABLE;
    //fuelSchedule1.pTimerDisable =  FUEL1_TIMER_DISABLE;
    //TEST_ASSERT_EQUAL(0, memcmp_P(&decoder.handler, &trigger_missing_tooth, sizeof(decoder.handler)));
    //TEST_ASSERT_EQUAL(fuelSchedule1.pTimerEnable, FUEL1_TIMER_ENABLE);
    //TEST_ASSERT_EQUAL(1003, fuelSchedule1.startCompare);
    //TEST_ASSERT_EQUAL(1003, fuelSchedule1.endCompare);
    TEST_ASSERT_TRUE(injectors.injector(injChannel1).fuelSchedule != nullptr);
    TEST_ASSERT_EQUAL(injectors.injector(injChannel1).fuelSchedule->pTimerEnable, FUEL1_TIMER_ENABLE);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);

  UNITY_BEGIN(); // start unit testing

  prepareForInitialiseAll();

  initialiseAll(); // Run the main initialise function

  RUN_TEST(test_debug);
  RUN_TEST(test_debug2);
  test_status_initial_off();
  test_status_off_to_pending();
  test_status_pending_to_running();
  test_status_running_to_pending();
  test_status_running_to_off();
  test_accuracy_timeout();
  test_accuracy_duration();

  UNITY_END(); // stop unit testing
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}