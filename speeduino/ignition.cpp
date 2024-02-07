#include "ignition.h"
#include "ignition_control.h"

void
configure_ignition_coil_schedule(IgnitionSchedule &ignitionSchedule, ignition_id_t ignition_id1)
{
  ignitionSchedule.start.pCallback = singleCoilBeginCharge;
  ignitionSchedule.start.args[0] = ignition_id1;
  ignitionSchedule.end.pCallback = singleCoilEndCharge;
  ignitionSchedule.end.args[0] = ignition_id1;
}

void
configure_ignition_coil_schedule(
  IgnitionSchedule &ignitionSchedule, ignition_id_t ignition_id1, ignition_id_t ignition_id2)
{
  ignitionSchedule.start.pCallback = twoCoilsBeginCharge;
  ignitionSchedule.start.args[0] = ignition_id1;
  ignitionSchedule.start.args[1] = ignition_id2;
  ignitionSchedule.end.pCallback = twoCoilsEndCharge;
  ignitionSchedule.end.args[0] = ignition_id1;
  ignitionSchedule.end.args[1] = ignition_id2;
}

