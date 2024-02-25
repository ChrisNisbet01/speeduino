#include "36_minus_222.h"
#include "missing_tooth.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** 36-2-2-2 crank based trigger wheel.
* A crank based trigger with a nominal 36 teeth, but 6 of these removed in 3 groups of 2.
* 2 of these groups are located concurrently.
* Note: This decoder supports both the H4 version (13-missing-16-missing-1-missing)
* and the H6 version of 36-2-2-2 (19-missing-10-missing-1-missing).
* The decoder checks which pattern is selected in order to determine the tooth number
* Note: www.thefactoryfiveforum.com/attachment.php?attachmentid=34279&d=1412431418
*
* @defgroup dec_36_2_2_2 36-2-2-2 Trigger wheel
* @{
*/
void triggerSetup_ThirtySixMinus222(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  //The number of physical teeth on the wheel.
  //Doing this here saves us a calculation each time in the interrupt
  triggerActualTeeth = 30;
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 36);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  checkSyncToothCount = configPage4.triggerTeeth >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;
}

void triggerPri_ThirtySixMinus222(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger.
  //(A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    //Begin the missing tooth detection
    //If the time between the current tooth and the last is greater than 2x the
    //time between the last tooth and the tooth before that, we make the
    //assertion that we must be at the first tooth after a gap
    //toothSystemCount is used to keep track of which missed tooth we're on.
    //It will be set to 1 if that last tooth seen was the middle one in the
    //-2-2 area. At all other times it will be 0
    if (toothSystemCount == 0)
    {
      //Multiply by 2 (Checks for a gap 2x greater than the last one)
      targetGap = (toothLastToothTime - toothLastMinusOneToothTime) * 2;
    }

    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      curGap = 0;
    }

    if (curGap > targetGap)
    {
      {
        if (toothSystemCount == 1)
        {
          //This occurs when we're at the first tooth after the 2 lots of 2x missing tooth.
          if (configPage2.nCylinders == 4) //H4
          {
            toothCurrentCount = 19;
          }
          else if (configPage2.nCylinders == 6) //H6 - NOT TESTED!
          {
            toothCurrentCount = 12;
          }

          toothSystemCount = 0;
          currentStatus.hasSync = true;
        }
        else
        {
          //We've seen a missing tooth set, but do not yet know whether it is
          //the single one or the double one.
          toothSystemCount = 1;
          toothCurrentCount++;
          //Accurately reflect the actual tooth count, including the skipped ones
          toothCurrentCount++;
        }
        //The tooth angle is double at this point
        BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
        //This is used to prevent a condition where serious intermittent signals
        //(e.g. someone furiously plugging the sensor wire in and out)
        //can leave the filter in an unrecoverable state
        triggerFilterTime = 0;
      }
    }
    else
    {
      if (toothCurrentCount > 36)
      {
        //Means a complete rotation has occurred.
        toothCurrentCount = 1;
        revolutionOne = !revolutionOne; //Flip sequential revolution tracker
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.startRevolutions++; //Counter

      }
      else if (toothSystemCount == 1)
      {
        //This occurs when a set of missing teeth had been seen, but the next
        //one was NOT missing.
        if (configPage2.nCylinders == 4)
        {
          //H4
          toothCurrentCount = 35;
          currentStatus.hasSync = true;
        }
        else if (configPage2.nCylinders == 6)
        {
          //H6 - THIS NEEDS TESTING
          toothCurrentCount = 34;
          currentStatus.hasSync = true;
        }
      }

      //Filter can only be recalculated for the regular teeth, not the missing one.
      setFilter(curGap);

      BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      toothSystemCount = 0;
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
}

void triggerSec_ThirtySixMinus222(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus222(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (configPage2.nCylinders == 4
        && toothCurrentCount != 19
        && toothCurrentCount != 16
        && toothCurrentCount != 34
        && BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else if (configPage2.nCylinders == 6
             && toothCurrentCount != 9
             && toothCurrentCount != 12
             && toothCurrentCount != 33
             && BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at and of the missing teeth as it
      //messes the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

void triggerSetEndTeeth_ThirtySixMinus222(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);

  if (configPage2.nCylinders == 4)
  {
    if (currentStatus.advance < 10)
    {
      ignition1.endTooth = 36;
    }
    else if (currentStatus.advance < 20)
    {
      ignition1.endTooth = 35;
    }
    else if (currentStatus.advance < 30)
    {
      ignition1.endTooth = 34;
    }
    else
    {
      ignition1.endTooth = 31;
    }

    if (currentStatus.advance < 30)
    {
      ignition2.endTooth = 16;
    }
    else
    {
      ignition2.endTooth = 13;
    }
  }
  else if (configPage2.nCylinders == 6)
  {
    //H6
    if (currentStatus.advance < 10)
    {
      ignition1.endTooth = 36;
    }
    else if (currentStatus.advance < 20)
    {
      ignition1.endTooth = 35;
    }
    else if (currentStatus.advance < 30)
    {
      ignition1.endTooth = 34;
    }
    else if (currentStatus.advance < 40)
    {
      ignition1.endTooth = 33;
    }
    else
    {
      ignition1.endTooth = 31;
    }

    if (currentStatus.advance < 20)
    {
      ignition2.endTooth = 9;
    }
    else
    {
      ignition2.endTooth = 6;
    }

    if (currentStatus.advance < 10)
    {
      ignition3.endTooth = 23;
    }
    else if (currentStatus.advance < 20)
    {
      ignition3.endTooth = 22;
    }
    else if (currentStatus.advance < 30)
    {
      ignition3.endTooth = 21;
    }
    else if (currentStatus.advance < 40)
    {
      ignition3.endTooth = 20;
    }
    else
    {
      ignition3.endTooth = 19;
    }
  }
}

static void attach_interrupts(void)
{
  //36-2-2-2
  byte const primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  byte const secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_ThirtySixMinus222, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_ThirtySixMinus222, secondaryTriggerEdge);
}

decoder_handler_st const trigger_36_minus_222 PROGMEM =
{
  .setup = triggerSetup_ThirtySixMinus222,
  .primaryToothHandler = triggerPri_ThirtySixMinus222,
  .secondaryToothHandler = triggerSec_ThirtySixMinus222,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_ThirtySixMinus222,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = triggerSetEndTeeth_ThirtySixMinus222,
  .attach_interrupts = attach_interrupts,
};

