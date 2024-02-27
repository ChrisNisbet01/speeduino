#include "ignition_schedule_class.h"
#include "ignition_control.h"

void IgnitionSchedule::reset(void)
{
  noInterrupts();

  Status = OFF;
  start.pCallback = nullIgnCallback;
  end.pCallback = nullIgnCallback;
  pTimerDisable();

  interrupts();
}
