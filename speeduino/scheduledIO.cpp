#include "scheduledIO.h"
#include "scheduler.h"
#include "globals.h"
#include "timers.h"
#include "acc_mc33810.h"
/** @file
 * Injector and Coil (toggle/open/close) control (under various situations, eg with particular cylinder count, rotary engine type or wasted spark ign, etc.).
 * Also accounts for presence of MC33810 injector/ignition (dwell, etc.) control circuit.
 * Functions here are typically assigned (at initialisation) to callback function variables (e.g. inj1StartFunction or inj1EndFunction)
 * from where they are called (by scheduler.ino).
 */

typedef void (*open_injector_fn)(void);
typedef void (*close_injector_fn)(void);
typedef void (*toggle_injector_fn)(void);
typedef enum injector_id_t {
  injector_id_1 = 0,
  injector_id_2,
  injector_id_3,
  injector_id_4,
  injector_id_5,
  injector_id_6,
  injector_id_7,
  injector_id_8,
  injector_id_COUNT,
} injector_id_t;

typedef struct injector_control_st
{
  open_injector_fn open;
  close_injector_fn close;
  toggle_injector_fn toggle;
} injector_control_st;

static void open_injector1_direct(void)
{
  openInjector1_DIRECT();
}

static void open_injector2_direct(void)
{
  openInjector2_DIRECT();
}

static void open_injector3_direct(void)
{
  openInjector3_DIRECT();
}

static void open_injector4_direct(void)
{
  openInjector4_DIRECT();
}

static void open_injector5_direct(void)
{
  openInjector5_DIRECT();
}

static void open_injector6_direct(void)
{
  openInjector6_DIRECT();
}

static void open_injector7_direct(void)
{
  openInjector7_DIRECT();
}

static void open_injector8_direct(void)
{
  openInjector8_DIRECT();
}

static void close_injector1_direct(void)
{
  closeInjector1_DIRECT();
}

static void close_injector2_direct(void)
{
  closeInjector2_DIRECT();
}

static void close_injector3_direct(void)
{
  closeInjector3_DIRECT();
}

static void close_injector4_direct(void)
{
  closeInjector4_DIRECT();
}

static void close_injector5_direct(void)
{
  closeInjector5_DIRECT();
}

static void close_injector6_direct(void)
{
  closeInjector6_DIRECT();
}

static void close_injector7_direct(void)
{
  closeInjector7_DIRECT();
}

static void close_injector8_direct(void)
{
  closeInjector8_DIRECT();
}

static void toggle_injector1_direct(void)
{
  injector1Toggle_DIRECT();
}

static void toggle_injector2_direct(void)
{
  injector2Toggle_DIRECT();
}

static void toggle_injector3_direct(void)
{
  injector3Toggle_DIRECT();
}

static void toggle_injector4_direct(void)
{
  injector4Toggle_DIRECT();
}

static void toggle_injector5_direct(void)
{
  injector5Toggle_DIRECT();
}

static void toggle_injector6_direct(void)
{
  injector6Toggle_DIRECT();
}

static void toggle_injector7_direct(void)
{
  injector7Toggle_DIRECT();
}

static void toggle_injector8_direct(void)
{
  injector8Toggle_DIRECT();
}

static void open_injector1_mc33810(void)
{
  openInjector1_MC33810();
}

static void open_injector2_mc33810(void)
{
  openInjector2_MC33810();
}

static void open_injector3_mc33810(void)
{
  openInjector3_MC33810();
}

static void open_injector4_mc33810(void)
{
  openInjector4_MC33810();
}

static void open_injector5_mc33810(void)
{
  openInjector5_MC33810();
}

static void open_injector6_mc33810(void)
{
  openInjector6_MC33810();
}

static void open_injector7_mc33810(void)
{
  openInjector7_MC33810();
}

static void open_injector8_mc33810(void)
{
  openInjector8_MC33810();
}

static void close_injector1_mc33810(void)
{
  closeInjector1_MC33810();
}

static void close_injector2_mc33810(void)
{
  closeInjector2_MC33810();
}

static void close_injector3_mc33810(void)
{
  closeInjector3_MC33810();
}

static void close_injector4_mc33810(void)
{
  closeInjector4_MC33810();
}

static void close_injector5_mc33810(void)
{
  closeInjector5_MC33810();
}

static void close_injector6_mc33810(void)
{
  closeInjector6_MC33810();
}

static void close_injector7_mc33810(void)
{
  closeInjector7_MC33810();
}

static void close_injector8_mc33810(void)
{
  closeInjector8_MC33810();
}

static void toggle_injector1_mc33810(void)
{
  injector1Toggle_MC33810();
}

static void toggle_injector2_mc33810(void)
{
  injector2Toggle_MC33810();
}

static void toggle_injector3_mc33810(void)
{
  injector3Toggle_MC33810();
}

static void toggle_injector4_mc33810(void)
{
  injector4Toggle_MC33810();
}

static void toggle_injector5_mc33810(void)
{
  injector5Toggle_MC33810();
}

static void toggle_injector6_mc33810(void)
{
  injector6Toggle_MC33810();
}

static void toggle_injector7_mc33810(void)
{
  injector7Toggle_MC33810();
}

static void toggle_injector8_mc33810(void)
{
  injector8Toggle_MC33810();
}

static injector_control_st const injector_control_direct[injector_id_COUNT] = {
  [injector_id_1] = {
    .open = open_injector1_direct,
    .close = close_injector1_direct,
    .toggle = toggle_injector1_direct,
  },
  [injector_id_2] = {
    .open = open_injector2_direct,
    .close = close_injector2_direct,
    .toggle = toggle_injector2_direct,
  },
  [injector_id_3] = {
    .open = open_injector3_direct,
    .close = close_injector3_direct,
    .toggle = toggle_injector3_direct,
  },
  [injector_id_4] = {
    .open = open_injector4_direct,
    .close = close_injector4_direct,
    .toggle = toggle_injector4_direct,
  },
  [injector_id_5] = {
    .open = open_injector5_direct,
    .close = close_injector5_direct,
    .toggle = toggle_injector5_direct,
  },
  [injector_id_6] = {
    .open = open_injector6_direct,
    .close = close_injector6_direct,
    .toggle = toggle_injector6_direct,
  },
  [injector_id_7] = {
    .open = open_injector7_direct,
    .close = close_injector7_direct,
    .toggle = toggle_injector7_direct,
  },
  [injector_id_8] = {
    .open = open_injector8_direct,
    .close = close_injector8_direct,
    .toggle = toggle_injector8_direct,
  }
};

static injector_control_st const injector_control_mc33810[injector_id_COUNT] = {
  [injector_id_1] = {
    .open = open_injector1_mc33810,
    .close = close_injector1_mc33810,
    .toggle = toggle_injector1_mc33810,
  },
  [injector_id_2] = {
    .open = open_injector2_mc33810,
    .close = close_injector2_mc33810,
    .toggle = toggle_injector2_mc33810,
  },
  [injector_id_3] = {
    .open = open_injector3_mc33810,
    .close = close_injector3_mc33810,
    .toggle = toggle_injector3_mc33810,
  },
  [injector_id_4] = {
    .open = open_injector4_mc33810,
    .close = close_injector4_mc33810,
    .toggle = toggle_injector4_mc33810,
  },
  [injector_id_5] = {
    .open = open_injector5_mc33810,
    .close = close_injector5_mc33810,
    .toggle = toggle_injector5_mc33810,
  },
  [injector_id_6] = {
    .open = open_injector6_mc33810,
    .close = close_injector6_mc33810,
    .toggle = toggle_injector6_mc33810,
  },
  [injector_id_7] = {
    .open = open_injector7_mc33810,
    .close = close_injector7_mc33810,
    .toggle = toggle_injector7_mc33810,
  },
  [injector_id_8] = {
    .open = open_injector8_mc33810,
    .close = close_injector8_mc33810,
    .toggle = toggle_injector8_mc33810,
  }
};

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

void injector_control_update(OUTPUT_CONTROL_TYPE const control_method)
{
  if( control_method == OUTPUT_CONTROL_MC33810 ) {
    injector_control = &injector_control_mc33810[0];
  }
  else {
    injector_control = &injector_control_direct[0];
  }
}

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE const control_method)
{
  injectorOutputControl = control_method;
  injector_control_update(control_method);
}

void coil1Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil1Toggle_DIRECT();
  }
  else {
    coil1Toggle_MC33810();
  }
}
void coil2Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil2Toggle_DIRECT();
  }
  else {
    coil2Toggle_MC33810();
  }
}
void coil3Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil3Toggle_DIRECT();
  }
  else {
    coil3Toggle_MC33810();
  }
}
void coil4Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil4Toggle_DIRECT();
  }
  else {
    coil4Toggle_MC33810();
  }
}
void coil5Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil5Toggle_DIRECT();
  }
  else {
    coil5Toggle_MC33810();
  }
}
void coil6Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil6Toggle_DIRECT();
  }
  else {
    coil6Toggle_MC33810();
  }
}
void coil7Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil7Toggle_DIRECT();
  }
  else {
    coil7Toggle_MC33810();
  }
}
void coil8Toggle(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil8Toggle_DIRECT();
  }
  else {
    coil8Toggle_MC33810();
  }
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

void beginCoil1Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil1Charging_DIRECT();
  }
  else {
    coil1Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil1Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil1StopCharging_DIRECT();
  }
  else {
    coil1StopCharging_MC33810();
  }
  tachoOutputOff();
}

void beginCoil2Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil2Charging_DIRECT();
  }
  else {
    coil2Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil2Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil2StopCharging_DIRECT();
  }
  else {
    coil2StopCharging_MC33810();
  }
  tachoOutputOff();
}

void beginCoil3Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil3Charging_DIRECT();
  }
  else {
    coil3Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil3Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil3StopCharging_DIRECT();
  }
  else {
    coil3StopCharging_MC33810();
  }
  tachoOutputOff();
}

void beginCoil4Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil4Charging_DIRECT();
  }
  else {
    coil4Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil4Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil4StopCharging_DIRECT();
  }
  else {
    coil4StopCharging_MC33810();
  }
  tachoOutputOff();
}

void beginCoil5Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil5Charging_DIRECT();
  }
  else {
    coil5Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil5Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil5StopCharging_DIRECT();
  }
  else {
    coil5StopCharging_MC33810();
  }
  tachoOutputOff();
}

void beginCoil6Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil6Charging_DIRECT();
  }
  else {
    coil6Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil6Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil6StopCharging_DIRECT();
  }
  else {
    coil6StopCharging_MC33810();
  }
  tachoOutputOff();
}

void beginCoil7Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil7Charging_DIRECT();
  }
  else {
    coil7Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil7Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil7StopCharging_DIRECT();
  }
  else {
    coil7StopCharging_MC33810();
  }
  tachoOutputOff();
}

void beginCoil8Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil8Charging_DIRECT();
  }
  else {
    coil8Charging_MC33810();
  }
  tachoOutputOn();
}
void endCoil8Charge(void)
{
  if( ignitionOutputControl != OUTPUT_CONTROL_MC33810 ) {
    coil8StopCharging_DIRECT();
  }
  else {
    coil8StopCharging_MC33810();
  }
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

void tachoOutputOn(void)
{
  if( configPage6.tachoMode ) {
    TACHO_PULSE_LOW();
  }
  else {
    tachoOutputFlag = READY;
  }
}
void tachoOutputOff(void)
{
  if( configPage6.tachoMode ) {
    TACHO_PULSE_HIGH();
  }
}

void nullCallback(void)
{
  return;
}
