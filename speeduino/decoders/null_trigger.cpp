#include "null_trigger.h"

//initialisation function for triggerhandlers, does exactly nothing
void nullTriggerHandler(void)
{
  return;
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


