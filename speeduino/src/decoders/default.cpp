#include "default.h"
#include "missing_tooth.h"
#include "null_trigger.h"
#include "decoders.h"
#include "utilities.h"
#include "auxiliary_pins.h"

static void triggerSetup_default(bool initialisationComplete)
{
  UNUSED(initialisationComplete);
  /* Do nothing. */
}

static void attach_interrupts(void)
{
  byte const primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_missingTooth, primaryTriggerEdge);
}

decoder_handler_st const trigger_default PROGMEM =
{
  .setup = triggerSetup_default,
  .primaryToothHandler = triggerPri_missingTooth,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_missingTooth,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = nullSetEndTeeth,
  .attach_interrupts = attach_interrupts,
};

