#include "default.h"
#include "missing_tooth.h"
#include "null_trigger.h"
#include "decoders.h"
#include "utilities.h"

static void triggerSetup_default(bool initialisationComplete)
{
  UNUSED(initialisationComplete);
  /* Do nothing. */
}

decoder_handler_st const trigger_default =
{
  .setup = triggerSetup_default,
  .primaryToothHandler = triggerPri_missingTooth,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_missingTooth,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = nullSetEndTeeth,
};

