#pragma once

#include "ignition_id.h"
#include "ignition_schedule_class.h"

void
configure_ignition_coil_schedule(
  IgnitionSchedule &ignitionSchedule, ignition_id_t ignition_id1);

void
configure_ignition_coil_schedule(
  IgnitionSchedule &ignitionSchedule, ignition_id_t ignition_id1, ignition_id_t ignition_id2);

void configure_rotary_fc_trailing_coil_schedules(
  IgnitionSchedule &ignitionSchedule1, IgnitionSchedule &ignitionSchedule2);

