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

open_injector_fn _openInjector1 = open_injector1_direct;
open_injector_fn _openInjector2 = open_injector2_direct;
open_injector_fn _openInjector3 = open_injector3_direct;
open_injector_fn _openInjector4 = open_injector4_direct;
open_injector_fn _openInjector5 = open_injector5_direct;
open_injector_fn _openInjector6 = open_injector6_direct;
open_injector_fn _openInjector7 = open_injector7_direct;
open_injector_fn _openInjector8 = open_injector8_direct;
close_injector_fn _closeInjector1 = close_injector1_direct;
close_injector_fn _closeInjector2 = close_injector2_direct;
close_injector_fn _closeInjector3 = close_injector3_direct;
close_injector_fn _closeInjector4 = close_injector4_direct;
close_injector_fn _closeInjector5 = close_injector5_direct;
close_injector_fn _closeInjector6 = close_injector6_direct;
close_injector_fn _closeInjector7 = close_injector7_direct;
close_injector_fn _closeInjector8 = close_injector8_direct;
toggle_injector_fn _injector1Toggle = toggle_injector1_direct;
toggle_injector_fn _injector2Toggle = toggle_injector2_direct;
toggle_injector_fn _injector3Toggle = toggle_injector3_direct;
toggle_injector_fn _injector4Toggle = toggle_injector4_direct;
toggle_injector_fn _injector5Toggle = toggle_injector5_direct;
toggle_injector_fn _injector6Toggle = toggle_injector6_direct;
toggle_injector_fn _injector7Toggle = toggle_injector7_direct;
toggle_injector_fn _injector8Toggle = toggle_injector8_direct;

void openInjector1(void)
{
    _openInjector1();
}

void closeInjector1(void)
{
    _closeInjector1();
}

void openInjector2(void)
{
    _openInjector2();
}

void closeInjector2(void)
{
    _closeInjector2();
}

void openInjector3(void)
{
    _openInjector3();
}

void closeInjector3(void)
{
    _closeInjector3();
}

void openInjector4(void)
{
    _openInjector4();
}

void closeInjector4(void)
{
    _closeInjector4();
}

void openInjector5(void)
{
    _openInjector5();
}

void closeInjector5(void)
{
    _closeInjector5();
}

void openInjector6(void)
{
    _openInjector6();
}

void closeInjector6(void)
{
    _closeInjector6();
}

void openInjector7(void)
{
    _openInjector7();
}

void closeInjector7(void)
{
    _closeInjector7();
}

void openInjector8(void)
{
    _openInjector8();
}

void closeInjector8(void)
{
    _closeInjector8();
}

void injector1Toggle(void)
{
    _injector1Toggle();
}

void injector2Toggle(void)
{
    _injector2Toggle();
}

void injector3Toggle(void)
{
    _injector3Toggle();
}

void injector4Toggle(void)
{
    _injector4Toggle();
}

void injector5Toggle(void)
{
    _injector5Toggle();
}

void injector6Toggle(void)
{
    _injector6Toggle();
}

void injector7Toggle(void)
{
    _injector7Toggle();
}

void injector8Toggle(void)
{
    _injector8Toggle();
}

void injector_control_update(OUTPUT_CONTROL_TYPE const control_method)
{
    if (control_method == OUTPUT_CONTROL_MC33810)
    {
        _openInjector1 = open_injector1_mc33810;
        _openInjector2 = open_injector2_mc33810;
        _openInjector3 = open_injector3_mc33810;
        _openInjector4 = open_injector4_mc33810;
        _openInjector5 = open_injector5_mc33810;
        _openInjector6 = open_injector6_mc33810;
        _openInjector7 = open_injector7_mc33810;
        _openInjector8 = open_injector8_mc33810;
        _closeInjector1 = close_injector1_mc33810;
        _closeInjector2 = close_injector2_mc33810;
        _closeInjector3 = close_injector3_mc33810;
        _closeInjector4 = close_injector4_mc33810;
        _closeInjector5 = close_injector5_mc33810;
        _closeInjector6 = close_injector6_mc33810;
        _closeInjector7 = close_injector7_mc33810;
        _closeInjector8 = close_injector8_mc33810;
        _injector1Toggle = toggle_injector1_mc33810;
        _injector2Toggle = toggle_injector2_mc33810;
        _injector3Toggle = toggle_injector3_mc33810;
        _injector4Toggle = toggle_injector4_mc33810;
        _injector5Toggle = toggle_injector5_mc33810;
        _injector6Toggle = toggle_injector6_mc33810;
        _injector7Toggle = toggle_injector7_mc33810;
        _injector8Toggle = toggle_injector8_mc33810;
    }
    else
    {
        _openInjector1 = open_injector1_direct;
        _openInjector2 = open_injector2_direct;
        _openInjector3 = open_injector3_direct;
        _openInjector4 = open_injector4_direct;
        _openInjector5 = open_injector5_direct;
        _openInjector6 = open_injector6_direct;
        _openInjector7 = open_injector7_direct;
        _openInjector8 = open_injector8_direct;
        _closeInjector1 = close_injector1_direct;
        _closeInjector2 = close_injector2_direct;
        _closeInjector3 = close_injector3_direct;
        _closeInjector4 = close_injector4_direct;
        _closeInjector5 = close_injector5_direct;
        _closeInjector6 = close_injector6_direct;
        _closeInjector7 = close_injector7_direct;
        _closeInjector8 = close_injector8_direct;
        _injector1Toggle = toggle_injector1_direct;
        _injector2Toggle = toggle_injector2_direct;
        _injector3Toggle = toggle_injector3_direct;
        _injector4Toggle = toggle_injector4_direct;
        _injector5Toggle = toggle_injector5_direct;
        _injector6Toggle = toggle_injector6_direct;
        _injector7Toggle = toggle_injector7_direct;
        _injector8Toggle = toggle_injector8_direct;
    }
}

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE const control_method)
{
    injectorOutputControl = control_method;
    injector_control_update(control_method);
}

void coil1Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil1Toggle_DIRECT(); } else { coil1Toggle_MC33810(); } }
void coil2Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil2Toggle_DIRECT(); } else { coil2Toggle_MC33810(); } }
void coil3Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil3Toggle_DIRECT(); } else { coil3Toggle_MC33810(); } }
void coil4Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil4Toggle_DIRECT(); } else { coil4Toggle_MC33810(); } }
void coil5Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil5Toggle_DIRECT(); } else { coil5Toggle_MC33810(); } }
void coil6Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil6Toggle_DIRECT(); } else { coil6Toggle_MC33810(); } }
void coil7Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil7Toggle_DIRECT(); } else { coil7Toggle_MC33810(); } }
void coil8Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil8Toggle_DIRECT(); } else { coil8Toggle_MC33810(); } }

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

void openInjector3and5(void) { openInjector3(); openInjector5(); }
void closeInjector3and5(void) { closeInjector3(); closeInjector5(); }

void openInjector2and5(void) { openInjector2(); openInjector5(); }
void closeInjector2and5(void) { closeInjector2(); closeInjector5(); }
void openInjector3and6(void) { openInjector3(); openInjector6(); }
void closeInjector3and6(void) { closeInjector3(); closeInjector6(); }

void openInjector1and5(void) { openInjector1(); openInjector5(); }
void closeInjector1and5(void) { closeInjector1(); closeInjector5(); }
void openInjector2and6(void) { openInjector2(); openInjector6(); }
void closeInjector2and6(void) { closeInjector2(); closeInjector6(); }
void openInjector3and7(void) { openInjector3(); openInjector7(); }
void closeInjector3and7(void) { closeInjector3(); closeInjector7(); }
void openInjector4and8(void) { openInjector4(); openInjector8(); }
void closeInjector4and8(void) { closeInjector4(); closeInjector8(); }

void beginCoil1Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil1Charging_DIRECT(); } else { coil1Charging_MC33810(); } tachoOutputOn(); }
void endCoil1Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil1StopCharging_DIRECT(); } else { coil1StopCharging_MC33810(); } tachoOutputOff(); }

void beginCoil2Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil2Charging_DIRECT(); } else { coil2Charging_MC33810(); } tachoOutputOn(); }
void endCoil2Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil2StopCharging_DIRECT(); } else { coil2StopCharging_MC33810(); } tachoOutputOff(); }

void beginCoil3Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil3Charging_DIRECT(); } else { coil3Charging_MC33810(); } tachoOutputOn(); }
void endCoil3Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil3StopCharging_DIRECT(); } else { coil3StopCharging_MC33810(); } tachoOutputOff(); }

void beginCoil4Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil4Charging_DIRECT(); } else { coil4Charging_MC33810(); } tachoOutputOn(); }
void endCoil4Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil4StopCharging_DIRECT(); } else { coil4StopCharging_MC33810(); } tachoOutputOff(); }

void beginCoil5Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil5Charging_DIRECT(); } else { coil5Charging_MC33810(); } tachoOutputOn(); }
void endCoil5Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil5StopCharging_DIRECT(); } else { coil5StopCharging_MC33810(); } tachoOutputOff(); }

void beginCoil6Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil6Charging_DIRECT(); } else { coil6Charging_MC33810(); } tachoOutputOn(); }
void endCoil6Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil6StopCharging_DIRECT(); } else { coil6StopCharging_MC33810(); } tachoOutputOff(); }

void beginCoil7Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil7Charging_DIRECT(); } else { coil7Charging_MC33810(); } tachoOutputOn(); }
void endCoil7Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil7StopCharging_DIRECT(); } else { coil7StopCharging_MC33810(); } tachoOutputOff(); }

void beginCoil8Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil8Charging_DIRECT(); } else { coil8Charging_MC33810(); } tachoOutputOn(); }
void endCoil8Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil8StopCharging_DIRECT(); } else { coil8StopCharging_MC33810(); } tachoOutputOff(); }

//The below 3 calls are all part of the rotary ignition mode
void beginTrailingCoilCharge(void) { beginCoil2Charge(); }
void endTrailingCoilCharge1(void) { endCoil2Charge(); beginCoil3Charge(); } //Sets ign3 (Trailing select) high
void endTrailingCoilCharge2(void) { endCoil2Charge(); endCoil3Charge(); } //sets ign3 (Trailing select) low

//As above but for ignition (Wasted COP mode)
void beginCoil1and3Charge(void) { beginCoil1Charge(); beginCoil3Charge(); }
void endCoil1and3Charge(void)   { endCoil1Charge();  endCoil3Charge(); }
void beginCoil2and4Charge(void) { beginCoil2Charge(); beginCoil4Charge(); }
void endCoil2and4Charge(void)   { endCoil2Charge();  endCoil4Charge(); }

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
void beginCoil3and7Charge(void) { beginCoil3Charge(); beginCoil7Charge();  }
void endCoil3and7Charge(void)   { endCoil3Charge(); endCoil7Charge(); }
void beginCoil4and8Charge(void) { beginCoil4Charge(); beginCoil8Charge(); }
void endCoil4and8Charge(void)   { endCoil4Charge();  endCoil8Charge(); }

void tachoOutputOn(void) { if(configPage6.tachoMode) { TACHO_PULSE_LOW(); } else { tachoOutputFlag = READY; } }
void tachoOutputOff(void) { if(configPage6.tachoMode) { TACHO_PULSE_HIGH(); } }

void nullCallback(void) { return; }
