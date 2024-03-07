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

void nullInjCallback(void)
{
  /* Do nothing. */
}

void openInjector1(void) { openInjector(injector_id_1); }
void closeInjector1(void) {  closeInjector(injector_id_1); }
void injector1Toggle(void) { toggleInjector(injector_id_1); }

void openInjector2(void) { openInjector(injector_id_2); }
void closeInjector2(void) {  closeInjector(injector_id_2); }
void injector2Toggle(void) { toggleInjector(injector_id_2); }

void openInjector3(void) { openInjector(injector_id_3); }
void closeInjector3(void) {  closeInjector(injector_id_3); }
void injector3Toggle(void) { toggleInjector(injector_id_3); }

void openInjector4(void) { openInjector(injector_id_4); }
void closeInjector4(void) {  closeInjector(injector_id_4); }
void injector4Toggle(void) { toggleInjector(injector_id_4); }

#if INJ_CHANNELS >= 5
void openInjector5(void) { openInjector(injector_id_5); }
void closeInjector5(void) {  closeInjector(injector_id_5); }
void injector5Toggle(void) { toggleInjector(injector_id_5); }
#endif

#if INJ_CHANNELS >= 6
void openInjector6(void) { openInjector(injector_id_6); }
void closeInjector6(void) {  closeInjector(injector_id_6); }
void injector6Toggle(void) { toggleInjector(injector_id_6); }
#endif

#if INJ_CHANNELS >= 7
void openInjector7(void) { openInjector(injector_id_7); }
void closeInjector7(void) {  closeInjector(injector_id_7); }
void injector7Toggle(void) { toggleInjector(injector_id_7); }
#endif

#if INJ_CHANNELS >= 8
void openInjector8(void) { openInjector(injector_id_8); }
void closeInjector8(void) {  closeInjector(injector_id_8); }
void injector8Toggle(void) { toggleInjector(injector_id_8); }
#endif

// These are for Semi-Sequential and 5 Cylinder injection
//Standard 4 cylinder pairings
void openInjector1and3(void) { openInjector1(); openInjector3(); }
void closeInjector1and3(void) { closeInjector1(); closeInjector3(); }
void openInjector2and4(void) { openInjector2(); openInjector4(); }
void closeInjector2and4(void) { closeInjector2(); closeInjector4(); }
//Alternative output pairings
void openInjector1and4(void) { openInjector1(); openInjector4(); }
void closeInjector1and4(void) { closeInjector1(); closeInjector4(); }
void openInjector2and3(void) { openInjector2(); openInjector3(); }
void closeInjector2and3(void) { closeInjector2(); closeInjector3(); }

#if INJ_CHANNELS >= 5
void openInjector3and5(void) { openInjector3(); openInjector5(); }
void closeInjector3and5(void) { closeInjector3(); closeInjector5(); }
#endif

#if INJ_CHANNELS >= 6
void openInjector2and5(void) { openInjector2(); openInjector5(); }
void closeInjector2and5(void) { closeInjector2(); closeInjector5(); }
void openInjector3and6(void) { openInjector3(); openInjector6(); }
void closeInjector3and6(void) { closeInjector3(); closeInjector6(); }
#endif

#if INJ_CHANNELS >= 8
void openInjector1and5(void) { openInjector1(); openInjector5(); }
void closeInjector1and5(void) { closeInjector1(); closeInjector5(); }
void openInjector2and6(void) { openInjector2(); openInjector6(); }
void closeInjector2and6(void) { closeInjector2(); closeInjector6(); }
void openInjector3and7(void) { openInjector3(); openInjector7(); }
void closeInjector3and7(void) { closeInjector3(); closeInjector7(); }
void openInjector4and8(void) { openInjector4(); openInjector8(); }
void closeInjector4and8(void) { closeInjector4(); closeInjector8(); }
#endif

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
