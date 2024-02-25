#include "harley.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */

/** Harley Davidson (V2) with 2 unevenly Spaced Teeth.
Within the decoder code, the sync tooth is referred to as tooth #1. Derived from GMX7 and adapted for Harley.
Only rising Edge is used for simplicity.The second input is ignored, as it does not help to resolve cam position.
* @defgroup dec_harley Harley Davidson
* @{
*/
void triggerSetup_Harley(bool const initialisationComplete)
{
  triggerToothAngle = 0; // The number of degrees that passes from tooth to tooth, ev. 0. It alternates uneven
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * 60U);

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on all the time.
    toothLastToothTime = micros();
  }
  triggerFilterTime = 1500;
}

void triggerPri_Harley(void)
{
  lastGap = curGap;
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  setFilter(curGap); // Filtering adjusted according to setting
  if (curGap > triggerFilterTime)
  {
    if (Trigger.read()) // Has to be the same as in main() trigger-attach, for readability we do it this way.
    {
      //Flag this pulse as being a valid trigger (ie that it passed filters)
      BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
      targetGap = lastGap; //Gap is the Time to next toothtrigger, so we know where we are
      toothCurrentCount++;
      if (curGap > targetGap)
      {
        toothCurrentCount = 1;
        triggerToothAngle = 0; // Has to be equal to Angle Routine
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.hasSync = true;
      }
      else
      {
        toothCurrentCount = 2;
        triggerToothAngle = 157;
      }
      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if (currentStatus.hasSync)
      {
        currentStatus.syncLossCounter++;
      }
      currentStatus.hasSync = false;
      toothCurrentCount = 0;
    } //Primary trigger high
  } //Trigger filter
}


uint16_t getRPM_Harley(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync)
  {
    if (currentStatus.RPM < (unsigned int)(configPage4.crankRPM * 100))
    {
      // No difference with this option?
      int tempToothAngle;
      unsigned long toothTime;
      if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
      {
        tempRPM = 0;
      }
      else
      {
        noInterrupts();

        tempToothAngle = triggerToothAngle;
        //The time in uS that one revolution would take at current speed
        //(The time tooth 1 was last seen, minus the time it was seen prior to that)
        SetRevolutionTime(toothOneTime - toothOneMinusOneTime);
        //Note that trigger tooth angle changes between 129 and 332 depending on
        //the last tooth that was seen
        toothTime = (toothLastToothTime - toothLastMinusOneToothTime);

        interrupts();

        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN / 10U)) / toothTime;
      }
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}

int getCrankAngle_Harley(void)
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

  //Check if the last tooth seen was the reference tooth (Number 3). All others can be calculated, but tooth 3 has a unique angle
  int crankAngle;

  if (tempToothCurrentCount == 1 || tempToothCurrentCount == 3)
  {
    //Number of teeth that have passed since tooth 1, multiplied by the angle
    //each tooth represents, plus the angle that tooth 1 is ATDC.
    //This gives accuracy only to the nearest tooth.
    crankAngle = 0 + configPage4.triggerAngle;
  }
  else
  {
    crankAngle = 157 + configPage4.triggerAngle;
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

decoder_handler_st const trigger_harley =
{
  .setup = triggerSetup_Harley,
  .primaryToothHandler = triggerPri_Harley,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Harley,
  .get_crank_angle = getCrankAngle_Harley,
  .set_end_teeth = nullSetEndTeeth,
};

