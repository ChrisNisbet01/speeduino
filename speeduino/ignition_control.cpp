#include "ignition_control.h"
#include "ignition_schedule_direct.h"
#include "ignition_schedule_mc33810.h"
#include "globals.h"
#include "timers.h"
#include "utilities.h"

/** @file
 * Coil (toggle/open/close) control (under various situations, eg with particular cylinder count, rotary engine type or wasted spark ign, etc.).
 * Also accounts for presence of MC33810 injector/ignition (dwell, etc.) control circuit.
 * Functions here are typically assigned (at initialisation) to callback function variables (e.g. inj1StartFunction or inj1EndFunction)
 * from where they are called (by scheduler.ino).
 */
static ignition_st const * ignition = &ignition_direct;

static void tachoOutputOn(void)
{
  if (configPage6.tachoMode)
  {
    TachOut.off();
  }
  else
  {
    tachoOutputFlag = READY;
  }
}

static void tachoOutputOff(void)
{
  if (configPage6.tachoMode)
  {
    TachOut.on();
  }
}

static void coil_toggle(ignition_id_t coil)
{
  ignition->toggle(coil);
}

static void singleCoilBeginCharge(ignition_id_t coil_id)
{
  ignition->begin_charge(coil_id);
  tachoOutputOn();
}

static void singleCoilEndCharge(ignition_id_t coil_id)
{
  ignition->end_charge(coil_id);
  tachoOutputOff();
}

//The below 3 calls are all part of the rotary ignition mode
void beginTrailingCoilCharge(void)
{
  singleCoilBeginCharge(ignition_id_2);
}

void endTrailingCoilCharge1(void) //Sets ign3 (Trailing select) high
{
  singleCoilEndCharge(ignition_id_2);
  singleCoilBeginCharge(ignition_id_3);
}

void endTrailingCoilCharge2(void) //sets ign3 (Trailing select) low
{
  singleCoilEndCharge(ignition_id_2);
  singleCoilEndCharge(ignition_id_3);
  tachoOutputOff();
}

void beginCoil1Charge(void) { singleCoilBeginCharge(ignition_id_1); }
void endCoil1Charge(void) { singleCoilEndCharge(ignition_id_1);}
void coil1Toggle(void) { coil_toggle(ignition_id_1); }

void beginCoil2Charge(void) { singleCoilBeginCharge(ignition_id_2); }
void endCoil2Charge(void) { singleCoilEndCharge(ignition_id_2);}
void coil2Toggle(void) { coil_toggle(ignition_id_2); }

void beginCoil3Charge(void) { singleCoilBeginCharge(ignition_id_3); }
void endCoil3Charge(void) { singleCoilEndCharge(ignition_id_3); }
void coil3Toggle(void) { coil_toggle(ignition_id_3); }

void beginCoil4Charge(void) { singleCoilBeginCharge(ignition_id_4); }
void endCoil4Charge(void) { singleCoilEndCharge(ignition_id_4); }
void coil4Toggle(void) { coil_toggle(ignition_id_4); }

#if IGN_CHANNELS >= 5
void beginCoil5Charge(void) { singleCoilBeginCharge(ignition_id_5); }
void endCoil5Charge(void) { singleCoilEndCharge(ignition_id_5); }
void coil5Toggle(void) { coil_toggle(ignition_id_5); }
#endif

#if IGN_CHANNELS >= 6
void beginCoil6Charge(void) { singleCoilBeginCharge(ignition_id_6); }
void endCoil6Charge(void) { singleCoilEndCharge(ignition_id_6); }
void coil6Toggle(void) { coil_toggle(ignition_id_6); }
#endif

#if IGN_CHANNELS >= 7
void beginCoil7Charge(void) { singleCoilBeginCharge(ignition_id_7); }
void endCoil7Charge(void) { singleCoilEndCharge(ignition_id_7); }
void coil7Toggle(void) { coil_toggle(ignition_id_7); }
#endif

#if IGN_CHANNELS >= 8
void beginCoil8Charge(void) { singleCoilBeginCharge(ignition_id_8); }
void endCoil8Charge(void) { singleCoilEndCharge(ignition_id_8); }
void coil8Toggle(void) { coil_toggle(ignition_id_8); }
#endif

//For 6cyl wasted COP mode)
void beginCoil1and4Charge(void) { beginCoil1Charge(); beginCoil4Charge(); }
void endCoil1and4Charge(void)   { endCoil1Charge();  endCoil4Charge(); }
void beginCoil2and5Charge(void) { beginCoil2Charge(); beginCoil5Charge(); }
void endCoil2and5Charge(void)   { endCoil2Charge();  endCoil5Charge(); }
void beginCoil3and6Charge(void) { beginCoil3Charge(); beginCoil6Charge(); }
void endCoil3and6Charge(void)   { endCoil3Charge(); endCoil6Charge(); }

//For 8cyl wasted COP mode)
void beginCoil1and5Charge(void) { beginCoil1Charge(); beginCoil5Charge(); }
void endCoil1and5Charge(void)   { endCoil1Charge();  endCoil5Charge(); }
void beginCoil2and6Charge(void) { beginCoil2Charge(); beginCoil6Charge(); }
void endCoil2and6Charge(void)   { endCoil2Charge();  endCoil6Charge(); }
void beginCoil3and7Charge(void) { beginCoil3Charge(); beginCoil7Charge(); }
void endCoil3and7Charge(void)   { endCoil3Charge(); endCoil7Charge(); }
void beginCoil4and8Charge(void) { beginCoil4Charge(); beginCoil8Charge(); }
void endCoil4and8Charge(void)   { endCoil4Charge();  endCoil8Charge(); }

void nullIgnCallback(void)
{
  /* Do nothing. */
}

static void ignition_control_update(OUTPUT_CONTROL_TYPE const control_method)
{
  if (control_method == OUTPUT_CONTROL_MC33810)
  {
    ignition = &ignition_mc33810;
  }
  else
  {
    ignition = &ignition_direct;
  }
}

void ignitionControlMethodAssign(OUTPUT_CONTROL_TYPE const control_method)
{
  ignitionOutputControl = control_method;
  ignition_control_update(control_method);
}

void ignition_pins_init(void)
{
  ignition->init();
}

