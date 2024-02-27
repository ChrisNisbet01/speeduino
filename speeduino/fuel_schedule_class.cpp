#include "fuel_schedule_class.h"
#include "injector_control.h"

void FuelSchedule::reset(void)
{
  noInterrupts();

  Status = OFF;
  start.pCallback = nullInjCallback;
  end.pCallback = nullInjCallback;
  pTimerDisable();

  interrupts();
}

