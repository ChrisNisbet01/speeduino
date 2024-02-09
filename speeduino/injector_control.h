#pragma once

#include "types.h"
#include "injector_id.h"

#include <stdint.h>

void injectorControlMethodAssign(OUTPUT_CONTROL_TYPE control_method);

/* Must be called _after_ the control method has been assigned. */
void injector_pins_init(void);

void openSingleInjector(injector_id_t injector_id, injector_id_t unused);
void openSingleInjector(injector_id_t injector_id);

void closeSingleInjector(injector_id_t injector_id, injector_id_t unused);
void closeSingleInjector(injector_id_t injector_id);

void openTwoInjectors(injector_id_t injector1_id, injector_id_t injector2_id);
void closeTwoInjectors(injector_id_t injector1_id, injector_id_t injector2_id);

void nullCallback(injector_id_t coil_id1, injector_id_t coil_id2);

