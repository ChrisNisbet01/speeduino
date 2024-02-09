#include "fuel_schedule_class.h"
#include "injector_control.h"

void FuelSchedule::reset(void)
{
    Status = OFF;
    pTimerEnable();
    start.pCallback = nullCallback;
    end.pCallback = nullCallback;
}

