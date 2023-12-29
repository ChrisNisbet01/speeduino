#ifndef IGNITION_SCHEDULE_H__
#define IGNITION_SCHEDULE_H__

#include "types.h"
#include "ignition_id.h"

#include <cstdint>

void singleCoilBeginCharge(uint8_t coil_id, uint8_t unused);
void singleCoilBeginCharge(uint8_t coil_id);
void singleCoilEndCharge(uint8_t coil_id, uint8_t unused);
void singleCoilEndCharge(uint8_t coil_id);

void twoCoilsBeginCharge(uint8_t coil_id1, uint8_t coil_id2);
void twoCoilsEndCharge(uint8_t coil_id1, uint8_t coil_id2);

//The following functions are used specifically for the trailing coil on rotary engines.
//They are separate as they also control the switching of the trailing select pin.
void beginTrailingCoilCharge(uint8_t unused1, uint8_t unused2);
void endTrailingCoilCharge1(uint8_t unused1, uint8_t unused2);
void endTrailingCoilCharge2(uint8_t unused1, uint8_t unused2);

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

