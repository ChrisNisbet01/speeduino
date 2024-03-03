#include "non_360.h"
#include "dual_wheel.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"

/** @} */

/** Non-360 Dual wheel with 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
There can be no missing teeth on the primary wheel.
* @defgroup dec_non360 Non-360 Dual wheel
* @{
*/
void triggerSetup_non360(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth multiplied by the additional multiplier
  triggerToothAngle = (360U * configPage4.TrigAngMul) / configPage4.triggerTeeth;
  toothCurrentCount = 255; //Default value
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth);
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2
  //(for cam speed)
  triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle;
}

uint16_t getRPM_non360(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync && toothCurrentCount != 0)
  {
    if (currentStatus.RPM < currentStatus.crankRPM)
    {
      tempRPM = crankingGetRPM(configPage4.triggerTeeth, CRANK_SPEED);
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}

int getCrankAngle_non360(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  unsigned long const lastCrankAngleCalc = micros();

  interrupts();

  //Handle case where the secondary tooth was the last one seen
  if (tempToothCurrentCount == 0)
  {
    tempToothCurrentCount = configPage4.triggerTeeth;
  }

  //Number of teeth that have passed since tooth 1, multiplied by the angle each
  //tooth represents, plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = (tempToothCurrentCount - 1) * triggerToothAngle;
  //Have to divide by the multiplier to get back to actual crank angle.
  crankAngle = (crankAngle / configPage4.TrigAngMul) + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  unsigned long const elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += 360;
  }

  return crankAngle;
}

static void attach_interrupts(void)
{
  // Attach the crank trigger wheel interrupt (Hall sensor drags to ground
  // when triggering)
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_DualWheel, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_DualWheel, secondaryTriggerEdge);
}

decoder_handler_st const trigger_non_360 PROGMEM =
{
  .setup = triggerSetup_non360,
  .primaryToothHandler = triggerPri_DualWheel,
  .secondaryToothHandler = triggerSec_DualWheel,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_non360,
  .get_crank_angle = getCrankAngle_non360,
  .set_end_teeth = nullSetEndTeeth,
  .attach_interrupts = attach_interrupts,
};

