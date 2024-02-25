#include "honda_d17.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */
/** Honda D17 (1.7 liter 4 cyl SOHC).
*
* @defgroup dec_honda_d17 Honda D17
* @{
*/
void triggerSetup_HondaD17(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerToothAngle = 360 / 12; //The number of degrees that passes from tooth to tooth
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const mimimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / mimimum_rpm) * triggerToothAngle);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_HondaD17(void)
{
  lastGap = curGap;
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  toothCurrentCount++; //Increment the tooth counter

  //Flag this pulse as being a valid trigger (ie that it passed filters)
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  if (toothCurrentCount == 13 && currentStatus.hasSync)
  {
    toothCurrentCount = 0;
  }
  else if (toothCurrentCount == 1 && currentStatus.hasSync)
  {
    toothOneMinusOneTime = toothOneTime;
    toothOneTime = curTime;
    currentStatus.startRevolutions++; //Counter

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  }
  else
  {
    //13th tooth
    targetGap = (lastGap) >> 1; //The target gap is set at half the last tooth gap

    //If the gap between this tooth and the last one is less than half of the previous gap,
    //then we are very likely at the magical 13th tooth
    if (curGap < targetGap)
    {
      toothCurrentCount = 0;
      currentStatus.hasSync = true;
    }
    else
    {
      //The tooth times below don't get set on tooth 13(The magical 13th tooth
      //should not be considered for any calculations that use those times)
      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
    }
  }
}

//The 4+1 signal on the cam is yet to be supported. If this ever changes,
//update BIT_DECODER_HAS_SECONDARY in the setup() function
void triggerSec_HondaD17(void)
{
  return;
}

uint16_t getRPM_HondaD17(void)
{
  return stdGetRPM(CRANK_SPEED);
}

int getCrankAngle_HondaD17(void)
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
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Check if the last tooth seen was the reference tooth 13 (Number 0 here).
  //All others can be calculated, but tooth 3 has a unique angle
  int crankAngle;

  if (tempToothCurrentCount == 0)
  {
    //if temptoothCurrentCount is 0, the last tooth seen was the 13th one.
    //Based on this, ignore the 13th tooth and use the 12th one as the last reference.
    crankAngle = (11 * triggerToothAngle) + configPage4.triggerAngle;
  }
  else
  {
    //Number of teeth that have passed since tooth 1, multiplied by the angle
    //each tooth represents, plus the angle that tooth 1 is ATDC.
    //This gives accuracy only to the nearest tooth.
    crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
  }

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
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

void triggerSetEndTeeth_HondaD17(void)
{
}

decoder_handler_st const trigger_jeep_2000 =
{
  .setup = triggerSetup_HondaD17,
  .primaryToothHandler = triggerPri_HondaD17,
  .secondaryToothHandler = triggerSec_HondaD17,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_HondaD17,
  .get_crank_angle = getCrankAngle_HondaD17,
  .set_end_teeth = triggerSetEndTeeth_HondaD17,
};

