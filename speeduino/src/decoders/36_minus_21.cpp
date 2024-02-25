#include "36_minus_21.h"
#include "missing_tooth.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** 36-2-1 / Mistsubishi 4B11 - A crank based trigger with a nominal 36 teeth,
*   but with 1 single and 1 double missing tooth.
* @defgroup dec_36_2_1 36-2-1 For Mistsubishi 4B11
* @{
*/
void triggerSetup_ThirtySixMinus21(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  //The number of physical teeth on the wheel. Doing this here saves us a
  //calculation each time in the interrupt. Not Used
  triggerActualTeeth = 33;
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 36);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;
}

void triggerPri_ThirtySixMinus21(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    //Begin the missing tooth detection
    //If the time between the current tooth and the last is greater than 2x the
    //time between the last tooth and the tooth before that, we make the
    //assertion that we must be at the first tooth after a gap

    //Multiply by 3 (Checks for a gap 3x greater than the last one)
    targetGap2 = 3 * (toothLastToothTime - toothLastMinusOneToothTime);
    //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one)
    //(Uses bitshift to divide by 2 as in the missing tooth decoder)
    targetGap = targetGap2 >> 1;

    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      curGap = 0;
    }

    if (curGap > targetGap)
    {
      if (curGap < targetGap2)
      {
        //we are at the tooth after the single gap
        toothCurrentCount = 20; //it's either 19 or 20, need to clarify engine direction!
        currentStatus.hasSync = true;
      }
      else
      {
        //we are at the tooth after the double gap
        toothCurrentCount = 1;
        currentStatus.hasSync = true;
      }

      //The tooth angle is double at this point
      BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      //This is used to prevent a condition where serious intermittent signals
      //(e.g. someone furiously plugging the sensor wire in and out) can leave
      //the filter in an unrecoverable state
      triggerFilterTime = 0;
    }
  }
  else
  {
    if (toothCurrentCount > 36 || toothCurrentCount == 1)
    {
      //Means a complete rotation has occurred.
      toothCurrentCount = 1;
      revolutionOne = !revolutionOne; //Flip sequential revolution tracker
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //Filter can only be recalculated for the regular teeth, not the missing one.
    setFilter(curGap);

    BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  }

  toothLastMinusOneToothTime = toothLastToothTime;
  toothLastToothTime = curTime;

  //EXPERIMENTAL!
  if (configPage2.perToothIgn)
  {
    int16_t crankAngle =
      ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
    crankAngle = ignitionLimits(crankAngle);
    checkPerToothTiming(crankAngle, toothCurrentCount);
  }
}

void triggerSec_ThirtySixMinus21(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus21(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (toothCurrentCount != 20 && BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes
      //the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }

  return tempRPM;
}

void triggerSetEndTeeth_ThirtySixMinus21(void)
{
  ignitions.ignition(ignChannel1).endTooth = 10;
  ignitions.ignition(ignChannel2).endTooth = 28; // Arbitrarily picked  at 180°.
}

static void attach_interrupts(void)
{
  //36-2-1
  byte const primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  byte const secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_ThirtySixMinus21, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_missingTooth, secondaryTriggerEdge);
}

decoder_handler_st const trigger_36_minus_21 PROGMEM =
{
  .setup = triggerSetup_ThirtySixMinus21,
  .primaryToothHandler = triggerPri_ThirtySixMinus21,
  .secondaryToothHandler = triggerSec_missingTooth,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_ThirtySixMinus21,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = triggerSetEndTeeth_ThirtySixMinus21,
  .attach_interrupts = attach_interrupts,
};

