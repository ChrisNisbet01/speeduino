#include "injector_schedule.h"

#include "injector_control.h"

void
configure_injector_schedule(FuelSchedule &fuelSchedule, injector_id_t injector_id)
{
  fuelSchedule.start.pCallback = openSingleInjector;
  fuelSchedule.start.injector_ids[0] = injector_id;
  fuelSchedule.end.pCallback = closeSingleInjector;
  fuelSchedule.end.injector_ids[0] = injector_id;
}

void
configure_injector_schedule(FuelSchedule &fuelSchedule, injector_id_t injector_id1, injector_id_t injector_id2)
{
  fuelSchedule.start.pCallback = openTwoInjectors;
  fuelSchedule.start.injector_ids[0] = injector_id1;
  fuelSchedule.start.injector_ids[1] = injector_id2;
  fuelSchedule.end.pCallback = closeTwoInjectors;
  fuelSchedule.end.injector_ids[0] = injector_id1;
  fuelSchedule.end.injector_ids[1] = injector_id2;
}


