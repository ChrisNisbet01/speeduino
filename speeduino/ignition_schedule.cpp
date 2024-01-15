#include "ignition_schedule.h"
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

void singleCoilBeginCharge(uint8_t coil_id, uint8_t unused)
{
  UNUSED(unused);
  ignition->begin_charge((ignition_id_t)coil_id);
  tachoOutputOn();
}

void singleCoilBeginCharge(uint8_t coil_id)
{
  ignition->begin_charge((ignition_id_t)coil_id);
  tachoOutputOn();
}

void singleCoilEndCharge(uint8_t coil_id, uint8_t unused)
{
  UNUSED(unused);
  ignition->end_charge((ignition_id_t)coil_id);
  tachoOutputOff();
}

void singleCoilEndCharge(uint8_t coil_id)
{
  ignition->end_charge((ignition_id_t)coil_id);
  tachoOutputOff();
}

void twoCoilsBeginCharge(uint8_t coil_id1, uint8_t coil_id2)
{
  ignition->begin_charge((ignition_id_t)coil_id1);
  ignition->begin_charge((ignition_id_t)coil_id2);
  tachoOutputOn();
}

void twoCoilsEndCharge(uint8_t coil_id1, uint8_t coil_id2)
{
  ignition->end_charge((ignition_id_t)coil_id1);
  ignition->end_charge((ignition_id_t)coil_id2);
  tachoOutputOff();
}

//The below 3 calls are all part of the rotary ignition mode
void beginTrailingCoilCharge(uint8_t unused1, uint8_t unused2)
{
  UNUSED(unused1);
  UNUSED(unused2);
  singleCoilBeginCharge(ignition_id_2);
}
void endTrailingCoilCharge1(uint8_t unused1, uint8_t unused2) //Sets ign3 (Trailing select) high
{
  UNUSED(unused1);
  UNUSED(unused2);
  singleCoilEndCharge(ignition_id_2);
  singleCoilBeginCharge(ignition_id_3);
}
void endTrailingCoilCharge2(uint8_t unused1, uint8_t unused2) //sets ign3 (Trailing select) low
{
  UNUSED(unused1);
  UNUSED(unused2);
  twoCoilsEndCharge(ignition_id_2, ignition_id_3);
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

