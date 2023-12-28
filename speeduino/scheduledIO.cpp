#include "scheduledIO.h"
#include "scheduler.h"
#include "globals.h"
#include "timers.h"
#include "acc_mc33810.h"

/** @file
 * Coil (toggle/open/close) control (under various situations, eg with particular cylinder count, rotary engine type or wasted spark ign, etc.).
 * Also accounts for presence of MC33810 injector/ignition (dwell, etc.) control circuit.
 * Functions here are typically assigned (at initialisation) to callback function variables (e.g. inj1StartFunction or inj1EndFunction)
 * from where they are called (by scheduler.ino).
 */

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
