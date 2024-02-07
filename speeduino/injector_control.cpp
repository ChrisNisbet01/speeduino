#include "injector_control.h"
#include "injectors.h"
#include "injector_schedule_direct.h"
#include "injector_schedule_mc33810.h"
#include "globals.h"
#include "utilities.h"

/** @file
 * Injector (toggle/open/close) control (under various situations, eg with particular cylinder count, rotary engine type or wasted spark ign, etc.).
 * Also accounts for presence of MC33810 injector/ignition (dwell, etc.) control circuit.
 * Functions here are typically assigned (at initialisation) to callback function variables (e.g. inj1StartFunction or inj1EndFunction)
 * from where they are called (by scheduler.ino).
 */

static injectors_st const * injectors = &injectors_direct;

static void openInjector(injector_id_t injector)
{
  injectors->open(injector);
}

static void closeInjector(injector_id_t injector)
{
  injectors->close(injector);
}

static void toggleInjector(injector_id_t injector)
{
  injectors->toggle(injector);
}

void openSingleInjector(uint8_t injector_id, uint8_t unused)
{
  UNUSED(unused);
  openInjector((injector_id_t)injector_id);
}

void openSingleInjector(uint8_t injector_id)
{
  openInjector((injector_id_t)injector_id);
}

void closeSingleInjector(uint8_t injector_id, uint8_t unused)
{
  UNUSED(unused);
  closeInjector((injector_id_t)injector_id);
}

void closeSingleInjector(uint8_t injector_id)
{
  closeInjector((injector_id_t)injector_id);
}

void openTwoInjectors(uint8_t arg1, uint8_t arg2)
{
  openInjector((injector_id_t)arg1);
  openInjector((injector_id_t)arg2);
}

void closeTwoInjectors(uint8_t arg1, uint8_t arg2)
{
  closeInjector((injector_id_t)arg1);
  closeInjector((injector_id_t)arg2);
}

void openInjector1(void)
{
  openInjector(injector_id_1);
}

void injector1Toggle(void)
{
  toggleInjector(injector_id_1);
}

void injector2Toggle(void)
{
  toggleInjector(injector_id_2);
}

void injector3Toggle(void)
{
  toggleInjector(injector_id_3);
}

void injector4Toggle(void)
{
  toggleInjector(injector_id_4);
}

void injector5Toggle(void)
{
  toggleInjector(injector_id_5);
}

void injector6Toggle(void)
{
  toggleInjector(injector_id_6);
}

void injector7Toggle(void)
{
  toggleInjector(injector_id_7);
}

void injector8Toggle(void)
{
  toggleInjector(injector_id_8);
}

static void injector_control_update(OUTPUT_CONTROL_TYPE const control_method)
{
  if (control_method == OUTPUT_CONTROL_MC33810)
  {
    injectors = &injectors_mc33810;
  }
  else
  {
    injectors = &injectors_direct;
  }
}

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE const control_method)
{
  injectorOutputControl = control_method;
  injector_control_update(control_method);
}

void injector_pins_init(void)
{
  injectors->init();
}

