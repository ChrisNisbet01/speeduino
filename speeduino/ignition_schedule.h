#ifndef IGNITION_SCHEDULE_H__
#define IGNITION_SCHEDULE_H__

#include "types.h"

void beginCoil1Charge(void);
void endCoil1Charge(void);

void beginCoil2Charge(void);
void endCoil2Charge(void);

void beginCoil3Charge(void);
void endCoil3Charge(void);

void beginCoil4Charge(void);
void endCoil4Charge(void);

void beginCoil5Charge(void);
void endCoil5Charge(void);

void beginCoil6Charge(void);
void endCoil6Charge(void);

void beginCoil7Charge(void);
void endCoil7Charge(void);

void beginCoil8Charge(void);
void endCoil8Charge(void);

//The following functions are used specifically for the trailing coil on rotary engines. They are separate as they also control the switching of the trailing select pin
void beginTrailingCoilCharge(void);
void endTrailingCoilCharge1(void);
void endTrailingCoilCharge2(void);

//And the combined versions of the above for simplicity
void beginCoil1and3Charge(void);
void endCoil1and3Charge(void);
void beginCoil2and4Charge(void);
void endCoil2and4Charge(void);

//For 6-cyl cop
void beginCoil1and4Charge(void);
void endCoil1and4Charge(void);
void beginCoil2and5Charge(void);
void endCoil2and5Charge(void);
void beginCoil3and6Charge(void);
void endCoil3and6Charge(void);

//For 8-cyl cop
void beginCoil1and5Charge(void);
void endCoil1and5Charge(void);
void beginCoil2and6Charge(void);
void endCoil2and6Charge(void);
void beginCoil3and7Charge(void);
void endCoil3and7Charge(void);
void beginCoil4and8Charge(void);
void endCoil4and8Charge(void);

void coil1Toggle(void);
void coil2Toggle(void);
void coil3Toggle(void);
void coil4Toggle(void);
void coil5Toggle(void);
void coil6Toggle(void);
void coil7Toggle(void);
void coil8Toggle(void);

void ignitionControlMethodAssign(OUTPUT_CONTROL_TYPE control_method);

/* Must be called _after_ the control method has been assigned. */
void  ignition_pins_init(void);

#endif /* IGNITION_SCHEDULE_H__ */

