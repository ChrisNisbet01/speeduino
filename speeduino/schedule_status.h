#pragma once

/** Schedule statuses.
 * - OFF - Schedule turned off and there is no scheduled plan
 * - PENDING - There's a scheduled plan, but is has not started to run yet
 * - STAGED - (???, Not used)
 * - RUNNING - Schedule is currently running
 */
//The statuses that a schedule can have
enum ScheduleStatus
{
  OFF,
  PENDING,
  STAGED,
  RUNNING
};

