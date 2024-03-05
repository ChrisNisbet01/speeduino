#pragma once

#include "types.h"
#include "ignition_id.h"

#include <stdint.h>

typedef void (*init_ignition_fn)(void);
typedef void (*ignition_begin_charge_id_fn)(ignition_id_t coil);
typedef void (*ignition_end_charge_id_fn)(ignition_id_t coil);
typedef void (*ignition_toggle_id_fn)(ignition_id_t coil);

typedef struct ignition_st
{
  init_ignition_fn init;
  ignition_begin_charge_id_fn begin_charge;
  ignition_end_charge_id_fn end_charge;
  ignition_toggle_id_fn toggle;
} ignition_st;

void beginCoil1Charge(void);
void endCoil1Charge(void);

void beginCoil2Charge(void);
void endCoil2Charge(void);

void beginCoil3Charge(void);
void endCoil3Charge(void);

void beginCoil4Charge(void);
void endCoil4Charge(void);

#if IGN_CHANNELS >= 5
void beginCoil5Charge(void);
void endCoil5Charge(void);
void coil5Toggle(void);
#endif

#if IGN_CHANNELS >= 6
void beginCoil6Charge(void);
void endCoil6Charge(void);
void coil6Toggle(void);
#endif

#if IGN_CHANNELS >= 7
void beginCoil7Charge(void);
void endCoil7Charge(void);
void coil7Toggle(void);
#endif

#if IGN_CHANNELS >= 8
void beginCoil8Charge(void);
void endCoil8Charge(void);
void coil8Toggle(void);
#endif

//The following functions are used specifically for the trailing coil on rotary engines.
//They are separate as they also control the switching of the trailing select pin.
void beginTrailingCoilCharge(void);
void endTrailingCoilCharge1(void);
void endTrailingCoilCharge2(void);

void nullIgnCallback(void);

void coil1Toggle(void);
void coil2Toggle(void);
void coil3Toggle(void);
void coil4Toggle(void);

void ignitionControlMethodAssign(OUTPUT_CONTROL_TYPE control_method);

/* Must be called _after_ the control method has been assigned. */
void  ignition_pins_init(void);

