#include "null_trigger.h"
#include "../../utilities.h"

//initialisation function for triggerhandlers, does exactly nothing
void nullTriggerHandler(void)
{
}

//initialisation function for getRpm, returns safe value of 0
uint16_t nullGetRPM(void)
{
  return 0;
}

//initialisation function for getCrankAngle, returns safe value of 0
int nullGetCrankAngle(void)
{
  return 0;
}

void nullSetEndTeeth(void)
{
}

static void nullSetup(bool initialisationComplete)
{
  UNUSED(initialisationComplete);
  /* Do nothing. */
}

static void attach_interrupts(void)
{
}

decoder_handler_st const trigger_null PROGMEM =
{
  .setup = nullSetup,
  .primaryToothHandler = nullTriggerHandler,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = nullGetRPM,
  .get_crank_angle = nullGetCrankAngle,
  .set_end_teeth = nullSetEndTeeth,
  .attach_interrupts = attach_interrupts,
};

