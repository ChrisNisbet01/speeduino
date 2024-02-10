#include "ignition_schedule.h"
#include "ignition_control.h"

void
configure_ignition_coil_schedule(IgnitionSchedule &ignitionSchedule, ignition_id_t ignition_id1)
{
  ignitionSchedule.start.pCallback = singleCoilBeginCharge;
  ignitionSchedule.start.coil_ids[0] = ignition_id1;
  ignitionSchedule.end.pCallback = singleCoilEndCharge;
  ignitionSchedule.end.coil_ids[0] = ignition_id1;
}

void
configure_ignition_coil_schedule(
  IgnitionSchedule &ignitionSchedule, ignition_id_t ignition_id1, ignition_id_t ignition_id2)
{
  ignitionSchedule.start.pCallback = twoCoilsBeginCharge;
  ignitionSchedule.start.coil_ids[0] = ignition_id1;
  ignitionSchedule.start.coil_ids[1] = ignition_id2;
  ignitionSchedule.end.pCallback = twoCoilsEndCharge;
  ignitionSchedule.end.coil_ids[0] = ignition_id1;
  ignitionSchedule.end.coil_ids[1] = ignition_id2;
}

void configure_rotary_fc_trailing_coil_schedules(
  IgnitionSchedule &ignitionSchedule1, IgnitionSchedule &ignitionSchedule2)
{

  ignitionSchedule1.start.pCallback = beginTrailingCoilCharge;
  ignitionSchedule1.end.pCallback = endTrailingCoilCharge1;

  ignitionSchedule2.start.pCallback = beginTrailingCoilCharge;
  ignitionSchedule2.end.pCallback = endTrailingCoilCharge2;
}

