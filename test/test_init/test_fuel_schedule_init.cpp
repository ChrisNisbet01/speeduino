#include <Arduino.h>
#include <unity.h>
#include "globals.h"
#include "init.h"
#include "schedule_calcs.h"
#include "utilities.h"
#include "../test_utils.h"
#include "storage.h"
#include "injector_control.h"
#include "injector_contexts.h"

extern uint16_t req_fuel_uS;
void prepareForInitialiseAll(uint8_t boardId);

static constexpr uint16_t reqFuel = 86; // ms * 10

static void __attribute__((noinline))
assert_fuel_channel(
    bool enabled,
    uint16_t angle,
    uint8_t cmdBit,
    int channelInjDegrees,
    injectorCallback_fn startFunction,
    injectorCallback_fn endFunction)
{
  char msg[39];

  sprintf_P(msg, PSTR("channel%" PRIu8 ".InjChannelIsEnabled. Max:%" PRIu8), cmdBit + 1, injectors.maxOutputs);
  TEST_ASSERT_TRUE_MESSAGE(!enabled || (cmdBit + 1) <= injectors.maxOutputs, msg);
  sprintf_P(msg, PSTR("channel%" PRIu8 ".InjDegrees"), cmdBit + 1);
  TEST_ASSERT_EQUAL_MESSAGE(angle, channelInjDegrees, msg);
  sprintf_P(msg, PSTR("inj%" PRIu8 ".StartFunction"), cmdBit + 1);
  TEST_ASSERT_TRUE_MESSAGE(!enabled || (startFunction!=nullInjCallback), msg);
  sprintf_P(msg, PSTR("inj%" PRIu8 ".EndFunction"), cmdBit + 1);
  TEST_ASSERT_TRUE_MESSAGE(!enabled || (endFunction!=nullInjCallback), msg);
}

static void __attribute__((noinline))
assert_fuel_schedules(
  uint16_t crankAngle,
  uint16_t reqFuel,
  const bool enabled[],
  const uint16_t angle[])
{
  char msg[32];

  strcpy_P(msg, PSTR("CRANK_ANGLE_MAX_INJ"));
  TEST_ASSERT_EQUAL_INT16_MESSAGE(crankAngle, CRANK_ANGLE_MAX_INJ, msg);
  strcpy_P(msg, PSTR("req_fuel_uS"));
  TEST_ASSERT_EQUAL_UINT16_MESSAGE(reqFuel, req_fuel_uS, msg);

  assert_fuel_channel(enabled[0], angle[0], INJ1_CMD_BIT, injector_contexts[injChannel1].channelInjDegrees, fuelSchedules[injChannel1].start.pCallback, fuelSchedule1.end.pCallback);
  assert_fuel_channel(enabled[1], angle[1], INJ2_CMD_BIT, injector_contexts[injChannel2].channelInjDegrees, fuelSchedules[injChannel2].start.pCallback, fuelSchedule2.end.pCallback);
  assert_fuel_channel(enabled[2], angle[2], INJ3_CMD_BIT, injector_contexts[injChannel3].channelInjDegrees, fuelSchedules[injChannel3].start.pCallback, fuelSchedule3.end.pCallback);
  assert_fuel_channel(enabled[3], angle[3], INJ4_CMD_BIT, injector_contexts[injChannel4].channelInjDegrees, fuelSchedules[injChannel4].start.pCallback, fuelSchedule4.end.pCallback);

#if INJ_CHANNELS>=5
  assert_fuel_channel(enabled[4], angle[4], INJ5_CMD_BIT, injector_contexts[injChannel5].channelInjDegrees, fuelSchedules[injChannel5].start.pCallback, fuelSchedule5.start.pCallback);
#endif

#if INJ_CHANNELS>=6
  assert_fuel_channel(enabled[5], angle[5], INJ6_CMD_BIT, injector_contexts[injChannel6].channelInjDegrees, fuelSchedules[injChannel6].start.pCallback, fuelSchedule6.start.pCallback);
#endif

#if INJ_CHANNELS>=7
  assert_fuel_channel(enabled[6], angle[6], INJ7_CMD_BIT, injector_contexts[injChannel7].channelInjDegrees, fuelSchedules[injChannel7].start.pCallback, fuelSchedule7.start.pCallback);
#endif

#if INJ_CHANNELS>=8
  assert_fuel_channel(enabled[7], angle[7], INJ8_CMD_BIT, injector_contexts[injChannel8].channelInjDegrees, fuelSchedules[injChannel8].start.pCallback, fuelSchedule8.start.pCallback);
#endif
}

static void cylinder1_stroke4_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, false, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
}

static void cylinder1_stroke4_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, false, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
}

static void cylinder1_stroke4_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
  }

static void cylinder1_stroke4_semiseq_staged(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
}

static void run_1_cylinder_4stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 1;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder1_stroke4_seq_nostage);
  RUN_TEST_P(cylinder1_stroke4_semiseq_nostage);
  RUN_TEST_P(cylinder1_stroke4_seq_staged);
  RUN_TEST_P(cylinder1_stroke4_semiseq_staged);
}

static void cylinder1_stroke2_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
  const bool enabled[] = {true, false, false, false, false, false, false, false};
  const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
}

static void cylinder1_stroke2_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, false, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
}

static void cylinder1_stroke2_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
}

static void cylinder1_stroke2_semiseq_staged(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
  const bool enabled[] = {true, true, false, false, false, false, false, false};
  const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
}

static void run_1_cylinder_2stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 1;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder1_stroke2_seq_nostage);
  RUN_TEST_P(cylinder1_stroke2_semiseq_nostage);
  RUN_TEST_P(cylinder1_stroke2_seq_staged);
  RUN_TEST_P(cylinder1_stroke2_semiseq_staged);
}

static void cylinder2_stroke4_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
}

static void cylinder2_stroke4_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
}

static void cylinder2_stroke4_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
}

static void cylinder2_stroke4_semiseq_staged(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
}

static void run_2_cylinder_4stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 2;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder2_stroke4_seq_nostage);
  RUN_TEST_P(cylinder2_stroke4_semiseq_nostage);
  RUN_TEST_P(cylinder2_stroke4_seq_staged);
  RUN_TEST_P(cylinder2_stroke4_semiseq_staged);
}


static void cylinder2_stroke2_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};

  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

static void cylinder2_stroke2_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

static void cylinder2_stroke2_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

static void cylinder2_stroke2_semiseq_staged(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

static void run_2_cylinder_2stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 2;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder2_stroke2_seq_nostage);
  RUN_TEST_P(cylinder2_stroke2_semiseq_nostage);
  RUN_TEST_P(cylinder2_stroke2_seq_staged);
  RUN_TEST_P(cylinder2_stroke2_semiseq_staged);
}

static void cylinder3_stroke4_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,240,480,0,0,0,0,0};

  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
}

static void cylinder3_stroke4_semiseq_nostage_tb(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  configPage2.injType = INJ_TYPE_TBODY;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
  assert_fuel_schedules(720U/3U, reqFuel * 50U, enabled, angle);
}

static void cylinder3_stroke4_semiseq_nostage_port(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = false;
  configPage2.injType = INJ_TYPE_PORT;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
  //Special case as 3 squirts per cycle MUST be over 720 degrees
  assert_fuel_schedules(720U/2U, reqFuel * 50U, enabled, angle);
}

static void cylinder3_stroke4_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage2.injTiming = true;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,240,480,0,240,480,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,240,480,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#endif
}

static void cylinder3_stroke4_semiseq_staged_tb(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  configPage2.injType = INJ_TYPE_TBODY;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const uint16_t angle[] = {0,80,160,0,80,160,0,0};
#else
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
#endif
	const bool enabled[] = {true, true, true, true, false, false, false, false};
  //Special case as 3 squirts per cycle MUST be over 720 degrees
  assert_fuel_schedules(720U/3U, reqFuel * 50U, enabled, angle);
}

static void cylinder3_stroke4_semiseq_staged_port(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  configPage2.injType = INJ_TYPE_PORT;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const uint16_t angle[] = {0,120,240,0,120,240,0,0};
#else
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
#endif
	const bool enabled[] = {true, true, true, true, false, false, false, false};
  //Special case as 3 squirts per cycle MUST be over 720 degrees
  assert_fuel_schedules(720U/2U, reqFuel * 50U, enabled, angle);
}

static void run_3_cylinder_4stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 3;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1; //3 squirts per cycle for a 3 cylinder

  RUN_TEST_P(cylinder3_stroke4_seq_nostage);
  RUN_TEST_P(cylinder3_stroke4_semiseq_nostage_tb);
  RUN_TEST_P(cylinder3_stroke4_semiseq_nostage_port);
  RUN_TEST_P(cylinder3_stroke4_seq_staged);
  RUN_TEST_P(cylinder3_stroke4_semiseq_staged_tb);
  RUN_TEST_P(cylinder3_stroke4_semiseq_staged_port);
}

static void cylinder3_stroke2_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
  }

static void cylinder3_stroke2_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder3_stroke2_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,120,240,0,120,240,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
#endif
  }

static void cylinder3_stroke2_semiseq_staged(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const uint16_t angle[] = {0,120,240,0,120,240,0,0};
#else
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
#endif
	const bool enabled[] = {true, true, true, true, false, false, false, false};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

static void run_3_cylinder_2stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 3;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder3_stroke2_seq_nostage);
  RUN_TEST_P(cylinder3_stroke2_semiseq_nostage);
  RUN_TEST_P(cylinder3_stroke2_seq_staged);
  RUN_TEST_P(cylinder3_stroke2_semiseq_staged);
}

static void assert_4cylinder_4stroke_seq_nostage(void)
{
    const bool enabled[] = {true, true, true, true, false, false, false, false};
    const uint16_t angle[] = {0,180,360,540,0,0,0,0};
    assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
}

static void cylinder4_stroke4_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
  assert_4cylinder_4stroke_seq_nostage();
}

static void cylinder4_stroke4_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
  }


static void cylinder4_stroke4_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=8
	const bool enabled[] = {true, true, true, true, true, true, true, true};
	const uint16_t angle[] = {0,180,360,540,0,180,360,540};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#elif INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,180,360,540,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
  assert_4cylinder_4stroke_seq_nostage();
#endif
}

static void cylinder4_stroke4_semiseq_staged(void)
{
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
}

void run_4_cylinder_4stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 4;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 2;

  RUN_TEST_P(cylinder4_stroke4_seq_nostage);
  RUN_TEST_P(cylinder4_stroke4_semiseq_nostage);
  RUN_TEST_P(cylinder4_stroke4_seq_staged);
  RUN_TEST_P(cylinder4_stroke4_semiseq_staged);
}

static void cylinder4_stroke2_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder4_stroke2_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder4_stroke2_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=8
	const bool enabled[] = {true, true, true, true, true, true, true, true};
	const uint16_t angle[] = {0,180,0,0,0,180,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
#elif INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
#endif
  }

static void cylinder4_stroke2_semiseq_staged(void)
{
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

void run_4_cylinder_2stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 4;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 2;

  RUN_TEST_P(cylinder4_stroke2_seq_nostage);
  RUN_TEST_P(cylinder4_stroke2_semiseq_nostage);
  RUN_TEST_P(cylinder4_stroke2_seq_staged);
  RUN_TEST_P(cylinder4_stroke2_semiseq_staged);
}

static void cylinder5_stroke4_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,144,288,432,576,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }


static void cylinder5_stroke4_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,72,144,216,288,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
  }

static void cylinder5_stroke4_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,144,288,432,576,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }

static void cylinder5_stroke4_semiseq_staged(void)
{
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,72,144,216,288,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,72,144,216,288,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
}

void run_5_cylinder_4stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 5;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 5;

  RUN_TEST_P(cylinder5_stroke4_seq_nostage);
  RUN_TEST_P(cylinder5_stroke4_semiseq_nostage);
  RUN_TEST_P(cylinder5_stroke4_seq_staged);
  RUN_TEST_P(cylinder5_stroke4_semiseq_staged);
}

static void cylinder6_stroke4_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,120,240,360,480,600,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }

static void cylinder6_stroke4_semiseq_nostage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
  }

static void cylinder6_stroke4_seq_staged(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 8
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,120,240,360,480,600,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }


static void cylinder6_stroke4_semiseq_staged(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 8
	const uint16_t angle[] = {0,120,240,0,0,120,240,0};
#else
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
#endif
	const bool enabled[] = {true, true, true, false, false, false, false, false};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
}

void run_6_cylinder_4stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 6;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 6;

  RUN_TEST_P(cylinder6_stroke4_seq_nostage);
  RUN_TEST_P(cylinder6_stroke4_semiseq_nostage);
  RUN_TEST_P(cylinder6_stroke4_seq_staged);
  RUN_TEST_P(cylinder6_stroke4_semiseq_staged);
}

static void cylinder8_stroke4_seq_nostage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 8
	const bool enabled[] = {true, true, true, true, true, true, true, true};
	const uint16_t angle[] = {0,90,180,270,360,450,540,630};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }

void run_8_cylinder_4stroke_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 8;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 8;

  // Staging not supported on 8 cylinders

  RUN_TEST_P(cylinder8_stroke4_seq_nostage);
}

static constexpr uint16_t zeroAngles[] = {0,0,0,0,0,0,0,0};

static void cylinder_1_NoinjTiming_paired(void) {
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 1;
  configPage2.divider = 1;

  initialiseAll(); //Run the main initialise function

  const bool enabled[] = {true, false, false, false, false, false, false, false};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, zeroAngles);
}

static void cylinder_2_NoinjTiming_paired(void) {
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 2;
  configPage2.divider = 2;

  initialiseAll(); //Run the main initialise function

  const bool enabled[] = {true, true, false, false, false, false, false, false};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, zeroAngles);
}

static void cylinder_3_NoinjTiming_paired(void) {
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 3;
  configPage2.divider = 3;

  initialiseAll(); //Run the main initialise function

  const bool enabled[] = {true, true, true, false, false, false, false, false};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, zeroAngles);
}

static void cylinder_4_NoinjTiming_paired(void) {
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 4;
  configPage2.divider = 4;

  initialiseAll(); //Run the main initialise function

  const bool enabled[] = {true, true, false, false, false, false, false, false};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, zeroAngles);
}

static void cylinder_5_NoinjTiming_paired(void) {
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 5;
  configPage2.divider = 5;

  initialiseAll(); //Run the main initialise function

  const bool enabled[] = {true, true, true, true, false, false, false, false};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, zeroAngles);
}

static void cylinder_6_NoinjTiming_paired(void) {
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 6;
  configPage2.divider = 6;

  initialiseAll(); //Run the main initialise function

  const bool enabled[] = {true, true, true, false, false, false, false, false};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, zeroAngles);
}

static void cylinder_8_NoinjTiming_paired(void) {
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 8;
  configPage2.divider = 8;

  initialiseAll(); //Run the main initialise function

  const bool enabled[] = {true, true, true, true, false, false, false, false};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, zeroAngles);
}

static void run_no_inj_timing_tests(void)
{
  prepareForInitialiseAll(3U);
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = false;
  configPage2.reqFuel = reqFuel;
  configPage10.stagingEnabled = false;

  RUN_TEST_P(cylinder_1_NoinjTiming_paired);
  RUN_TEST_P(cylinder_2_NoinjTiming_paired);
  RUN_TEST_P(cylinder_3_NoinjTiming_paired);
  RUN_TEST_P(cylinder_4_NoinjTiming_paired);
  RUN_TEST_P(cylinder_5_NoinjTiming_paired);
  RUN_TEST_P(cylinder_6_NoinjTiming_paired);
  RUN_TEST_P(cylinder_8_NoinjTiming_paired);
}

static void cylinder_2_oddfire(void)
{
  configPage2.injLayout = INJ_PAIRED;
  configPage2.nCylinders = 2;
  configPage2.divider = 2;

  initialiseAll(); //Run the main initialise function

	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,13,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
}

static void run_oddfire_tests()
{
  prepareForInitialiseAll(3U);
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = ODD_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage10.stagingEnabled = false;
  configPage2.oddfire2 = 13;
  configPage2.oddfire3 = 111;
  configPage2.oddfire4 = 217;

  // Oddfire only affects 2 cylinder configurations
  configPage2.nCylinders = 1;
  configPage2.divider = 1;
  RUN_TEST_P(cylinder1_stroke4_seq_nostage);

  RUN_TEST_P(cylinder_2_oddfire);

  configPage2.nCylinders = 3;
  configPage2.divider = 1;
  RUN_TEST_P(cylinder3_stroke4_seq_nostage);
  configPage2.nCylinders = 4;
  configPage2.divider = 2;
  RUN_TEST_P(cylinder4_stroke4_seq_nostage);
  configPage2.nCylinders = 5;
  configPage2.divider = 5;
  RUN_TEST_P(cylinder5_stroke4_seq_nostage);
  configPage2.nCylinders = 6;
  configPage2.divider = 6;
  RUN_TEST_P(cylinder6_stroke4_seq_nostage);
  configPage2.nCylinders = 8;
  configPage2.divider = 8;
  RUN_TEST_P(cylinder8_stroke4_seq_nostage);
}

static void test_partial_sync(void)
{
  prepareForInitialiseAll(3U);
  configPage2.nCylinders = 4;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function

  // Confirm initial state
  assert_4cylinder_4stroke_seq_nostage();

  changeFullToHalfSync();
  {
	  const bool enabled[] = {true, true, false, false, false, false, false, false};
	  const uint16_t angle[] = {0,180,360,540,0,0,0,0};
    assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
  }

  changeHalfToFullSync();
  assert_4cylinder_4stroke_seq_nostage();
}


void testFuelScheduleInit()
{
  run_1_cylinder_4stroke_tests();
  run_1_cylinder_2stroke_tests();
  run_2_cylinder_4stroke_tests();
  run_2_cylinder_2stroke_tests();
  run_3_cylinder_4stroke_tests();
  run_3_cylinder_2stroke_tests();
  run_4_cylinder_4stroke_tests();
  run_4_cylinder_2stroke_tests();
  run_5_cylinder_4stroke_tests();
  run_6_cylinder_4stroke_tests();
  run_8_cylinder_4stroke_tests();

  run_no_inj_timing_tests();

  run_oddfire_tests();

  RUN_TEST_P(test_partial_sync);
}