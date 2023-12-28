#include "injector_schedule.h"
#include "injector_schedule_direct.h"
#include "injector_schedule_mc33810.h"
#include "globals.h"

/** @file
 * Injector (toggle/open/close) control (under various situations, eg with particular cylinder count, rotary engine type or wasted spark ign, etc.).
 * Also accounts for presence of MC33810 injector/ignition (dwell, etc.) control circuit.
 * Functions here are typically assigned (at initialisation) to callback function variables (e.g. inj1StartFunction or inj1EndFunction)
 * from where they are called (by scheduler.ino).
 */

static injector_control_st const * injector_control = injector_control_direct;

static void openInjector(injector_id_t injector)
{
  injector_control[injector].open();
}

static void closeInjector(injector_id_t injector)
{
  injector_control[injector].close();
}

static void toggleInjector(injector_id_t injector)
{
  injector_control[injector].toggle();
}

void openInjector1(void)
{
  openInjector(injector_id_1);
}

void closeInjector1(void)
{
  closeInjector(injector_id_1);
}

void openInjector2(void)
{
  openInjector(injector_id_2);
}

void closeInjector2(void)
{
  closeInjector(injector_id_2);
}

void openInjector3(void)
{
  openInjector(injector_id_3);
}

void closeInjector3(void)
{
  closeInjector(injector_id_3);
}

void openInjector4(void)
{
  openInjector(injector_id_4);
}

void closeInjector4(void)
{
  closeInjector(injector_id_4);
}

void openInjector5(void)
{
  openInjector(injector_id_5);
}

void closeInjector5(void)
{
  closeInjector(injector_id_5);
}

void openInjector6(void)
{
  openInjector(injector_id_6);
}

void closeInjector6(void)
{
  closeInjector(injector_id_6);
}

void openInjector7(void)
{
  openInjector(injector_id_7);
}

void closeInjector7(void)
{
  closeInjector(injector_id_7);
}

void openInjector8(void)
{
  openInjector(injector_id_8);
}

void closeInjector8(void)
{
  closeInjector(injector_id_8);
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

// These are for Semi-Sequential and 5 Cylinder injection
//Standard 4 cylinder pairings
void openInjector1and3(void)
{
  openInjector(injector_id_1);
  openInjector(injector_id_3);
}
void closeInjector1and3(void)
{
  closeInjector(injector_id_1);
  closeInjector(injector_id_3);
}
void openInjector2and4(void)
{
  openInjector(injector_id_2);
  openInjector(injector_id_4);
}
void closeInjector2and4(void)
{
  closeInjector(injector_id_2);
  closeInjector(injector_id_4);
}
//Alternative output pairings
void openInjector1and4(void)
{
  openInjector(injector_id_1);
  openInjector(injector_id_4);
}
void closeInjector1and4(void)
{
  closeInjector(injector_id_1);
  closeInjector(injector_id_4);
}
void openInjector2and3(void)
{
  openInjector(injector_id_2);
  openInjector(injector_id_3);
}
void closeInjector2and3(void)
{
  closeInjector(injector_id_2);
  closeInjector(injector_id_3);
}

void openInjector3and5(void)
{
  openInjector(injector_id_3);
  openInjector(injector_id_5);
}
void closeInjector3and5(void)
{
  closeInjector(injector_id_3);
  closeInjector(injector_id_5);
}

void openInjector2and5(void)
{
  openInjector(injector_id_2);
  openInjector(injector_id_5);
}
void closeInjector2and5(void)
{
  closeInjector(injector_id_2);
  closeInjector(injector_id_5);
}
void openInjector3and6(void)
{
  openInjector(injector_id_3);
  openInjector(injector_id_6);
}
void closeInjector3and6(void)
{
  closeInjector(injector_id_3);
  closeInjector(injector_id_6);
}

void openInjector1and5(void)
{
  openInjector(injector_id_1);
  openInjector(injector_id_5);
}
void closeInjector1and5(void)
{
  closeInjector(injector_id_1);
  closeInjector(injector_id_5);
}
void openInjector2and6(void)
{
  openInjector(injector_id_2);
  openInjector(injector_id_6);
}
void closeInjector2and6(void)
{
  closeInjector(injector_id_2);
  closeInjector(injector_id_6);
}
void openInjector3and7(void)
{
  openInjector(injector_id_3);
  openInjector(injector_id_7);
}
void closeInjector3and7(void)
{
  closeInjector(injector_id_3);
  closeInjector(injector_id_7);
}
void openInjector4and8(void)
{
  openInjector(injector_id_4);
  openInjector(injector_id_8);
}
void closeInjector4and8(void)
{
  closeInjector(injector_id_4);
  closeInjector(injector_id_8);
}

void injector_control_update(OUTPUT_CONTROL_TYPE const control_method)
{
  if (control_method == OUTPUT_CONTROL_MC33810)
  {
    injector_control = injector_control_mc33810;
  }
  else
  {
    injector_control = injector_control_direct;
  }
}

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE const control_method)
{
  injectorOutputControl = control_method;
  injector_control_update(control_method);
}
