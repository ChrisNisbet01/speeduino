#include <Arduino.h>
#include <unity.h>
#include "test_calcs_common.h"
#include "schedule_calcs.h"
#include "crankMaths.h"
#include "decoders.h"

#define _countof(x) (sizeof(x) / sizeof (x[0]))

// void printFreeRam()
// {
//     char msg[128];
//     sprintf(msg, "freeRam: %u", freeRam());
//     TEST_MESSAGE(msg);
// }

struct inj_test_parameters
{
    uint16_t channelAngle;  // deg
    uint16_t pw;            // uS
    uint16_t crankAngle;    // deg
    uint32_t pending;       // Expected delay when channel status is PENDING
    uint32_t running;       // Expected delay when channel status is RUNNING
};

static void nullInjCallback(void) { }

static void test_calc_inj_timeout(const inj_test_parameters &parameters)
{
    static constexpr uint16_t injAngle = 355;
    char msg[150];
    uint16_t PWdivTimerPerDegree = timeToAngleDegPerMicroSec(parameters.pw, degreesPerMicro);

    FuelSchedule schedule(FUEL2_COUNTER, FUEL2_COMPARE, nullInjCallback, nullInjCallback);

    schedule.Status = PENDING;
    uint16_t startAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, parameters.channelAngle, injAngle);
    sprintf_P(msg, PSTR("PENDING channelAngle: %" PRIu16 ", pw: %" PRIu16 ", crankAngle: %" PRIu16 ", startAngle: %" PRIu16), parameters.channelAngle, parameters.pw, parameters.crankAngle, startAngle);
    TEST_ASSERT_INT32_WITHIN_MESSAGE(1, parameters.pending, calculateInjectorTimeout(schedule, parameters.channelAngle, startAngle, parameters.crankAngle), msg);

    schedule.Status = RUNNING;
    startAngle = calculateInjectorStartAngle( PWdivTimerPerDegree, parameters.channelAngle, injAngle);
    sprintf_P(msg, PSTR("RUNNING channelAngle: %" PRIu16 ", pw: %" PRIu16 ", crankAngle: %" PRIu16 ", startAngle: %" PRIu16), parameters.channelAngle, parameters.pw, parameters.crankAngle, startAngle);
    TEST_ASSERT_INT32_WITHIN_MESSAGE(1, parameters.running, calculateInjectorTimeout(schedule, parameters.channelAngle, startAngle, parameters.crankAngle), msg);
}


static void test_calc_inj_timeout(const inj_test_parameters *pStart, const inj_test_parameters *pEnd)
{
    inj_test_parameters local;
    while (pStart!=pEnd)
    {
        memcpy_P(&local, pStart, sizeof(local));
        test_calc_inj_timeout(local);
        ++pStart;
    }
}

static void test_calc_inj_timeout_360()
{
    setEngineSpeed(4000, 360);

    // Expected test values were generated using floating point calculations (in Excel)
    static const inj_test_parameters test_data[] PROGMEM = {
        { 0, 3000, 0, 11792, 11792 },
        { 0, 3000, 45, 9917, 9917 },
        { 0, 3000, 90, 8042, 8042 },
        { 0, 3000, 135, 6167, 6167 },
        { 0, 3000, 180, 4292, 4292 },
        { 0, 3000, 215, 2833, 2833 },
        { 0, 3000, 270, 542, 542 },
        { 0, 3000, 315, 0, 13667 },
        { 0, 3000, 360, 0, 11792 },
        { 72, 3000, 0, 0, 14792 },
        { 72, 3000, 45, 0, 12917 },
        { 72, 3000, 90, 11042, 11042 },
        { 72, 3000, 135, 9167, 9167 },
        { 72, 3000, 180, 7292, 7292 },
        { 72, 3000, 215, 5833, 5833 },
        { 72, 3000, 270, 3542, 3542 },
        { 72, 3000, 315, 1667, 1667 },
        { 72, 3000, 360, 0, 14792 },
        { 80, 3000, 0, 125, 125 },
        { 80, 3000, 45, 0, 13250 },
        { 80, 3000, 90, 11375, 11375 },
        { 80, 3000, 135, 9500, 9500 },
        { 80, 3000, 180, 7625, 7625 },
        { 80, 3000, 215, 6167, 6167 },
        { 80, 3000, 270, 3875, 3875 },
        { 80, 3000, 315, 2000, 2000 },
        { 80, 3000, 360, 125, 125 },
        { 90, 3000, 0, 542, 542 },
        { 90, 3000, 45, 0, 13667 },
        { 90, 3000, 90, 11792, 11792 },
        { 90, 3000, 135, 9917, 9917 },
        { 90, 3000, 180, 8042, 8042 },
        { 90, 3000, 215, 6583, 6583 },
        { 90, 3000, 270, 4292, 4292 },
        { 90, 3000, 315, 2417, 2417 },
        { 90, 3000, 360, 542, 542 },
        { 144, 3000, 0, 2792, 2792 },
        { 144, 3000, 45, 917, 917 },
        { 144, 3000, 90, 0, 14042 },
        { 144, 3000, 135, 0, 12167 },
        { 144, 3000, 180, 10292, 10292 },
        { 144, 3000, 215, 8833, 8833 },
        { 144, 3000, 270, 6542, 6542 },
        { 144, 3000, 315, 4667, 4667 },
        { 144, 3000, 360, 2792, 2792 },
        { 180, 3000, 0, 4292, 4292 },
        { 180, 3000, 45, 2417, 2417 },
        { 180, 3000, 90, 542, 542 },
        { 180, 3000, 135, 0, 13667 },
        { 180, 3000, 180, 11792, 11792 },
        { 180, 3000, 215, 10333, 10333 },
        { 180, 3000, 270, 8042, 8042 },
        { 180, 3000, 315, 6167, 6167 },
        { 180, 3000, 360, 4292, 4292 },
        { 240, 3000, 0, 6792, 6792 },
        { 240, 3000, 45, 4917, 4917 },
        { 240, 3000, 90, 3042, 3042 },
        { 240, 3000, 135, 1167, 1167 },
        { 240, 3000, 180, 0, 14292 },
        { 240, 3000, 215, 0, 12833 },
        { 240, 3000, 270, 10542, 10542 },
        { 240, 3000, 315, 8667, 8667 },
        { 240, 3000, 360, 6792, 6792 },
        { 270, 3000, 0, 8042, 8042 },
        { 270, 3000, 45, 6167, 6167 },
        { 270, 3000, 90, 4292, 4292 },
        { 270, 3000, 135, 2417, 2417 },
        { 270, 3000, 180, 542, 542 },
        { 270, 3000, 215, 0, 14083 },
        { 270, 3000, 270, 11792, 11792 },
        { 270, 3000, 315, 9917, 9917 },
        { 270, 3000, 360, 8042, 8042 },
        { 360, 3000, 0, 11792, 11792 },
        { 360, 3000, 45, 9917, 9917 },
        { 360, 3000, 90, 8042, 8042 },
        { 360, 3000, 135, 6167, 6167 },
        { 360, 3000, 180, 4292, 4292 },
        { 360, 3000, 215, 2833, 2833 },
        { 360, 3000, 270, 542, 542 },
        { 360, 3000, 315, 0, 13667 },
        { 360, 3000, 360, 11792, 11792 },
    };

    test_calc_inj_timeout(&test_data[0], &test_data[0]+_countof(test_data));
}

static void test_calc_inj_timeout_720()
{
    setEngineSpeed(4000, 720);

    // Expected test values were generated using floating point calculations (in Excel)
    static const inj_test_parameters test_data[] PROGMEM = {
        // ChannelAngle (deg), PW (uS), Crank (deg), Expected Pending (uS), Expected Running (uS)
        { 0, 3000, 0, 11792, 11792 },
        { 0, 3000, 45, 9917, 9917 },
        { 0, 3000, 90, 8042, 8042 },
        { 0, 3000, 135, 6167, 6167 },
        { 0, 3000, 180, 4292, 4292 },
        { 0, 3000, 215, 2833, 2833 },
        { 0, 3000, 270, 542, 542 },
        { 0, 3000, 315, 0, 28667 },
        { 0, 3000, 360, 0, 26792 },
        { 72, 3000, 0, 0, 14792 },
        { 72, 3000, 45, 0, 12917 },
        { 72, 3000, 90, 11042, 11042 },
        { 72, 3000, 135, 9167, 9167 },
        { 72, 3000, 180, 7292, 7292 },
        { 72, 3000, 215, 5833, 5833 },
        { 72, 3000, 270, 3542, 3542 },
        { 72, 3000, 315, 1667, 1667 },
        { 72, 3000, 360, 0, 29792 },
        { 80, 3000, 0, 0, 15125 },
        { 80, 3000, 45, 0, 13250 },
        { 80, 3000, 90, 11375, 11375 },
        { 80, 3000, 135, 9500, 9500 },
        { 80, 3000, 180, 7625, 7625 },
        { 80, 3000, 215, 6167, 6167 },
        { 80, 3000, 270, 3875, 3875 },
        { 80, 3000, 315, 2000, 2000 },
        { 80, 3000, 360, 125, 125 },
        { 90, 3000, 0, 0, 15542 },
        { 90, 3000, 45, 0, 13667 },
        { 90, 3000, 90, 11792, 11792 },
        { 90, 3000, 135, 9917, 9917 },
        { 90, 3000, 180, 8042, 8042 },
        { 90, 3000, 215, 6583, 6583 },
        { 90, 3000, 270, 4292, 4292 },
        { 90, 3000, 315, 2417, 2417 },
        { 90, 3000, 360, 542, 542 },
        { 144, 3000, 0, 0, 17792 },
        { 144, 3000, 45, 0, 15917 },
        { 144, 3000, 90, 0, 14042 },
        { 144, 3000, 135, 0, 12167 },
        { 144, 3000, 180, 10292, 10292 },
        { 144, 3000, 215, 8833, 8833 },
        { 144, 3000, 270, 6542, 6542 },
        { 144, 3000, 315, 4667, 4667 },
        { 144, 3000, 360, 2792, 2792 },
        { 180, 3000, 0, 0, 19292 },
        { 180, 3000, 45, 0, 17417 },
        { 180, 3000, 90, 0, 15542 },
        { 180, 3000, 135, 0, 13667 },
        { 180, 3000, 180, 11792, 11792 },
        { 180, 3000, 215, 10333, 10333 },
        { 180, 3000, 270, 8042, 8042 },
        { 180, 3000, 315, 6167, 6167 },
        { 180, 3000, 360, 4292, 4292 },
        { 240, 3000, 0, 0, 21792 },
        { 240, 3000, 45, 0, 19917 },
        { 240, 3000, 90, 0, 18042 },
        { 240, 3000, 135, 0, 16167 },
        { 240, 3000, 180, 0, 14292 },
        { 240, 3000, 215, 0, 12833 },
        { 240, 3000, 270, 10542, 10542 },
        { 240, 3000, 315, 8667, 8667 },
        { 240, 3000, 360, 6792, 6792 },
        { 270, 3000, 0, 0, 23042 },
        { 270, 3000, 45, 0, 21167 },
        { 270, 3000, 90, 0, 19292 },
        { 270, 3000, 135, 0, 17417 },
        { 270, 3000, 180, 0, 15542 },
        { 270, 3000, 215, 0, 14083 },
        { 270, 3000, 270, 11792, 11792 },
        { 270, 3000, 315, 9917, 9917 },
        { 270, 3000, 360, 8042, 8042 },
        { 360, 3000, 0, 0, 26792 },
        { 360, 3000, 45, 0, 24917 },
        { 360, 3000, 90, 0, 23042 },
        { 360, 3000, 135, 0, 21167 },
        { 360, 3000, 180, 0, 19292 },
        { 360, 3000, 215, 0, 17833 },
        { 360, 3000, 270, 0, 15542 },
        { 360, 3000, 315, 0, 13667 },
        { 360, 3000, 360, 11792, 11792 },
        { 480, 3000, 0, 1792, 1792 },
        { 480, 3000, 45, 0, 29917 },
        { 480, 3000, 90, 0, 28042 },
        { 480, 3000, 135, 0, 26167 },
        { 480, 3000, 180, 0, 24292 },
        { 480, 3000, 215, 0, 22833 },
        { 480, 3000, 270, 0, 20542 },
        { 480, 3000, 315, 0, 18667 },
        { 480, 3000, 360, 0, 16792 },
        { 540, 3000, 0, 4292, 4292 },
        { 540, 3000, 45, 2417, 2417 },
        { 540, 3000, 90, 542, 542 },
        { 540, 3000, 135, 0, 28667 },
        { 540, 3000, 180, 0, 26792 },
        { 540, 3000, 215, 0, 25333 },
        { 540, 3000, 270, 0, 23042 },
        { 540, 3000, 315, 0, 21167 },
        { 540, 3000, 360, 0, 19292 },
        { 600, 3000, 0, 6792, 6792 },
        { 600, 3000, 45, 4917, 4917 },
        { 600, 3000, 90, 3042, 3042 },
        { 600, 3000, 135, 1167, 1167 },
        { 600, 3000, 180, 0, 29292 },
        { 600, 3000, 215, 0, 27833 },
        { 600, 3000, 270, 0, 25542 },
        { 600, 3000, 315, 0, 23667 },
        { 600, 3000, 360, 0, 21792 },
        { 630, 3000, 0, 8042, 8042 },
        { 630, 3000, 45, 6167, 6167 },
        { 630, 3000, 90, 4292, 4292 },
        { 630, 3000, 135, 2417, 2417 },
        { 630, 3000, 180, 542, 542 },
        { 630, 3000, 215, 0, 29083 },
        { 630, 3000, 270, 0, 26792 },
        { 630, 3000, 315, 0, 24917 },
        { 630, 3000, 360, 0, 23042 },
  };

    test_calc_inj_timeout(&test_data[0], &test_data[0]+_countof(test_data));
}

//
void test_calc_inj_timeout(void)
{
    RUN_TEST(test_calc_inj_timeout_360);
    RUN_TEST(test_calc_inj_timeout_720);
}