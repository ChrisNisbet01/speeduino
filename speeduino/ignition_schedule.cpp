#include "ignition_schedule.h"
#include "ignition_schedule_direct.h"
#include "ignition_schedule_mc33810.h"
#include "globals.h"
#include "timers.h"

/** @file
 * Coil (toggle/open/close) control (under various situations, eg with particular cylinder count, rotary engine type or wasted spark ign, etc.).
 * Also accounts for presence of MC33810 injector/ignition (dwell, etc.) control circuit.
 * Functions here are typically assigned (at initialisation) to callback function variables (e.g. inj1StartFunction or inj1EndFunction)
 * from where they are called (by scheduler.ino).
 */
static ignition_st const * ignition = &ignition_direct;

static void tachoOutputOn(void)
{
  if( configPage6.tachoMode ) {
    TACHO_PULSE_LOW();
  }
  else {
    tachoOutputFlag = READY;
  }
}

static void tachoOutputOff(void)
{
  if( configPage6.tachoMode ) {
    TACHO_PULSE_HIGH();
  }
}

static void coil_begin_charge(ignition_id_t coil)
{
  ignition->begin_charge(coil);
}

static void coil_end_charge(ignition_id_t coil)
{
  ignition->end_charge(coil);
}

static void coil_toggle(ignition_id_t coil)
{
  ignition->toggle(coil);
}

void coil1Toggle(void)
{
  coil_toggle(ignition_id_1);
}
void coil2Toggle(void)
{
  coil_toggle(ignition_id_2);
}
void coil3Toggle(void)
{
  coil_toggle(ignition_id_3);
}
void coil4Toggle(void)
{
  coil_toggle(ignition_id_4);
}
void coil5Toggle(void)
{
  coil_toggle(ignition_id_5);
}
void coil6Toggle(void)
{
  coil_toggle(ignition_id_6);
}
void coil7Toggle(void)
{
  coil_toggle(ignition_id_7);
}
void coil8Toggle(void)
{
  coil_toggle(ignition_id_8);
}

void beginCoil1Charge(void)
{
  coil_begin_charge(ignition_id_1);
  tachoOutputOn();
}
void endCoil1Charge(void)
{
  coil_end_charge(ignition_id_1);
  tachoOutputOff();
}

void beginCoil2Charge(void)
{
  coil_begin_charge(ignition_id_2);
  tachoOutputOn();
}
void endCoil2Charge(void)
{
  coil_end_charge(ignition_id_2);
  tachoOutputOff();
}

void beginCoil3Charge(void)
{
  coil_begin_charge(ignition_id_3);
  tachoOutputOn();
}
void endCoil3Charge(void)
{
  coil_end_charge(ignition_id_3);
  tachoOutputOff();
}

void beginCoil4Charge(void)
{
  coil_begin_charge(ignition_id_4);
  tachoOutputOn();
}
void endCoil4Charge(void)
{
  coil_end_charge(ignition_id_4);
  tachoOutputOff();
}

void beginCoil5Charge(void)
{
  coil_begin_charge(ignition_id_5);
  tachoOutputOn();
}
void endCoil5Charge(void)
{
  coil_end_charge(ignition_id_5);
  tachoOutputOff();
}

void beginCoil6Charge(void)
{
  coil_begin_charge(ignition_id_6);
  tachoOutputOn();
}
void endCoil6Charge(void)
{
  coil_end_charge(ignition_id_6);
  tachoOutputOff();
}

void beginCoil7Charge(void)
{
  coil_begin_charge(ignition_id_7);
  tachoOutputOn();
}
void endCoil7Charge(void)
{
  coil_end_charge(ignition_id_7);
  tachoOutputOff();
}

void beginCoil8Charge(void)
{
  coil_begin_charge(ignition_id_8);
  tachoOutputOn();
}
void endCoil8Charge(void)
{
  coil_end_charge(ignition_id_8);
  tachoOutputOff();
}

//The below 3 calls are all part of the rotary ignition mode
void beginTrailingCoilCharge(void)
{
  beginCoil2Charge();
}
void endTrailingCoilCharge1(void) //Sets ign3 (Trailing select) high
{
  endCoil2Charge(); beginCoil3Charge();
}
void endTrailingCoilCharge2(void) //sets ign3 (Trailing select) low
{
  endCoil2Charge(); endCoil3Charge();
}

//As above but for ignition (Wasted COP mode)
void beginCoil1and3Charge(void)
{
  beginCoil1Charge(); beginCoil3Charge();
}
void endCoil1and3Charge(void)
{
  endCoil1Charge();  endCoil3Charge();
}
void beginCoil2and4Charge(void)
{
  beginCoil2Charge(); beginCoil4Charge();
}
void endCoil2and4Charge(void)
{
  endCoil2Charge();  endCoil4Charge();
}

//For 6cyl wasted COP mode)
void beginCoil1and4Charge(void)
{
  beginCoil1Charge(); beginCoil4Charge();
}
void endCoil1and4Charge(void)
{
  endCoil1Charge();  endCoil4Charge();
}
void beginCoil2and5Charge(void)
{
  beginCoil2Charge(); beginCoil5Charge();
}
void endCoil2and5Charge(void)
{
  endCoil2Charge();  endCoil5Charge();
}
void beginCoil3and6Charge(void)
{
  beginCoil3Charge(); beginCoil6Charge();
}
void endCoil3and6Charge(void)
{
  endCoil3Charge(); endCoil6Charge();
}

//For 8cyl wasted COP mode)
void beginCoil1and5Charge(void)
{
  beginCoil1Charge(); beginCoil5Charge();
}
void endCoil1and5Charge(void)
{
  endCoil1Charge();  endCoil5Charge();
}
void beginCoil2and6Charge(void)
{
  beginCoil2Charge(); beginCoil6Charge();
}
void endCoil2and6Charge(void)
{
  endCoil2Charge();  endCoil6Charge();
}
void beginCoil3and7Charge(void)
{
  beginCoil3Charge(); beginCoil7Charge();
}
void endCoil3and7Charge(void)
{
  endCoil3Charge(); endCoil7Charge();
}
void beginCoil4and8Charge(void)
{
  beginCoil4Charge(); beginCoil8Charge();
}
void endCoil4and8Charge(void)
{
  endCoil4Charge();  endCoil8Charge();
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

