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

static void flash(void)
{
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);

  UNITY_BEGIN(); // start unit testing

  prepareForInitialiseAll();

  initialiseAll(); // Run the main initialise function

  test_status_initial_off();
  flash();
  test_status_off_to_pending();
  flash();
  //test_status_pending_to_running();
  //flash();
  //test_status_running_to_pending();
  //flash();
  //test_status_running_to_off();
  //flash();
  //test_accuracy_timeout();
  //flash();
  test_accuracy_duration();
  flash();

  UNITY_END(); // stop unit testing
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}