#include "ignition_schedule_class.h"
#include "ignition_control.h"

void IgnitionSchedule::reset(void)
{
    Status = OFF;
    start.pCallback = nullCallback;
    end.pCallback = nullCallback;
    pTimerDisable();
}
