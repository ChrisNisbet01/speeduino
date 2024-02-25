#include "gm7x.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "utilities.h"

/** @} */

/** Decode GM 7X trigger wheel with six equally spaced teeth and a seventh tooth
*   for cylinder identification.
* Note: Within the decoder code pf GM7X, the sync tooth is referred to as tooth #3
* rather than tooth #7. This makes for simpler angle calculations
* (See: http://www.speeduino.com/forum/download/file.php?id=4743 ).
* @defgroup dec_gm7x GM7X
* @{
*/
void triggerSetup_GM7X(bool initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerToothAngle = 360 / 6; //The number of degrees that passes from tooth to tooth
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
}

void triggerPri_GM7X(void)
{
  lastGap = curGap;
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  toothCurrentCount++; //Increment the tooth counter
  //Flag this pulse as being a valid trigger (ie that it passed filters)
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  if (toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
  {
    if (toothCurrentCount > 7)
    {
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;

      BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
    }
    else
    {
      targetGap = lastGap >> 1; //The target gap is set at half the last tooth gap

      //If the gap between this tooth and the last one is less than half of the
      //previous gap, then we are very likely at the magical 3rd tooth
      if (curGap < targetGap)
      {
        toothCurrentCount = 3;
        currentStatus.hasSync = true;
        //The tooth angle is double at this point
        BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
        currentStatus.startRevolutions++; //Counter
      }
      else
      {
        BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      }
    }
  }

  //New ignition mode!
  if (configPage2.perToothIgn)
  {
    //Never do the check on the extra tooth. It's not needed anyway
    if (toothCurrentCount != 3)
    {
      //configPage4.triggerAngle must currently be below 48 and above -81
      int16_t crankAngle;

      if (toothCurrentCount < 3)
      {
        //Number of teeth that have passed since tooth 1, multiplied by the angle
        //each tooth represents, plus the angle that tooth 1 is ATDC.
        //This gives accuracy only to the nearest tooth.
        crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + 42 + configPage4.triggerAngle;
      }
      else
      {
        //Number of teeth that have passed since tooth 1, multiplied by the angle
        //each tooth represents, plus the angle that tooth 1 is ATDC.
        //This gives accuracy only to the nearest tooth.
        crankAngle = ((toothCurrentCount - 2) * triggerToothAngle) + 42 + configPage4.triggerAngle;
      }
      checkPerToothTiming(crankAngle, toothCurrentCount);
    }
  }

  toothLastMinusOneToothTime = toothLastToothTime;
  toothLastToothTime = curTime;


}
void triggerSec_GM7X(void) //Not required
{
  return;
}
uint16_t getRPM_GM7X(void)
{
  return stdGetRPM(CRANK_SPEED);
}
int getCrankAngle_GM7X(void)
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

  //Check if the last tooth seen was the reference tooth (Number 3).
  //All others can be calculated, but tooth 3 has a unique angle
  int crankAngle;
  if (tempToothCurrentCount < 3)
  {
    //Number of teeth that have passed since tooth 1, multiplied by the angle
    //each tooth represents, plus the angle that tooth 1 is ATDC.
    //This gives accuracy only to the nearest tooth.
    crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + 42 + configPage4.triggerAngle;
  }
  else if (tempToothCurrentCount == 3)
  {
    crankAngle = 112;
  }
  else
  {
    //Number of teeth that have passed since tooth 1, multiplied by the angle each
    //tooth represents, plus the angle that tooth 1 is ATDC.
    //This gives accuracy only to the nearest tooth.
    crankAngle = ((tempToothCurrentCount - 2) * triggerToothAngle) + 42 + configPage4.triggerAngle;
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

void triggerSetEndTeeth_GM7X(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);

  if (currentStatus.advance < 18)
  {
    ignition1.endTooth = 7;
    ignition2.endTooth = 2;
    ignition3.endTooth = 5;
  }
  else
  {
    ignition1.endTooth = 6;
    ignition2.endTooth = 1;
    ignition3.endTooth = 4;
  }
}

decoder_handler_st const trigger_GM7X =
{
  .setup = triggerSetup_GM7X,
  .primaryToothHandler = triggerPri_GM7X,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_GM7X,
  .get_crank_angle = getCrankAngle_GM7X,
  .set_end_teeth = triggerSetEndTeeth_GM7X,
};

