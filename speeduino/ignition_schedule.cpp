#include "ignition_schedule.h"
#include "ignition_control.h"

void configure_rotary_fc_trailing_coil_schedules(
  IgnitionSchedule &ignitionSchedule1, IgnitionSchedule &ignitionSchedule2)
{

  ignitionSchedule1.start.pCallback = beginTrailingCoilCharge;
  ignitionSchedule1.end.pCallback = endTrailingCoilCharge1;

  ignitionSchedule2.start.pCallback = beginTrailingCoilCharge;
  ignitionSchedule2.end.pCallback = endTrailingCoilCharge2;
}

