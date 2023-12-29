#ifndef INJECTOR_SCHEDULE_H__
#define INJECTOR_SCHEDULE_H__

#include "types.h"
#include "injector_id.h"

#include <cstdint>

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE control_method);

/* Must be called _after_ the control method has been assigned. */
void injector_pins_init(void);

void openSingleInjector(uint8_t injector_id, uint8_t unused);
void openSingleInjector(uint8_t injector_id);

void closeSingleInjector(uint8_t injector_id, uint8_t unused);
void closeSingleInjector(uint8_t injector_id);

void openTwoInjectors(uint8_t injector1_id, uint8_t injector2_id);
void closeTwoInjectors(uint8_t injector1_id, uint8_t injector2_id);

#endif /* INJECTOR_SCHEDULE_H__ */

