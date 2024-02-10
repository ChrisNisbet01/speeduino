#pragma once

#include "fuel_schedule_class.h"
#include "injector_id.h"

void
configure_injector_schedule(FuelSchedule &fuelSchedule, injector_id_t injector_id);

void
configure_injector_schedule(FuelSchedule &fuelSchedule, injector_id_t injector_id1, injector_id_t injector_id2);

