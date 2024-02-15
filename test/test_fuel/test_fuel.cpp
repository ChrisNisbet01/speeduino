#include <globals.h>
#include <init.h>
#include <Arduino.h>
#include <unity.h>

#include "test_corrections.h"
#include "test_PW.h"
#include "test_staging.h"

#define UNITY_EXCLUDE_DETAILS

int init_ok = 0;

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

#if 0
static void testInitialiseAll(void)
{
    prepareForInitialiseAll();
    initialiseAll();

    TEST_ASSERT_EQUAL(init_ok, 1);
}
#endif

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();

    prepareForInitialiseAll();
    initialiseAll(); // Run the main initialise function
    testCorrections();
    //testPW();
    //testStaging();

    UNITY_END();
}

void loop()
{
    // Blink to indicate end of test
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}