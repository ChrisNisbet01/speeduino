#include "subaru_67.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"

/** @} */

/** Subaru 6/7 Trigger pattern decoder for 6 tooth (irregularly spaced) crank
*   and 7 tooth (also fairly irregular) cam wheels (eg late 90's Impreza 2.2).
This seems to be present in late 90's Subaru. In 2001 Subaru moved to 36-2-2-2*
*(See: http://www.vems.hu/wiki/index.php?page=InputTrigger%2FSubaruTrigger ).
* @defgroup dec_subaru_6_7 Subaru 6/7
* @{
*/
void triggerSetup_Subaru67(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 360UL);
  triggerSecFilterTime = 0;
  //Initially set to 0 prior to calculating the secondary window duration
  secondaryToothCount = 0;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  toothCurrentCount = 1;
  triggerToothAngle = 2;
  BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  toothSystemCount = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * 93U;

  toothAngles[0] = 710; //tooth #1
  toothAngles[1] = 83; //tooth #2
  toothAngles[2] = 115; //tooth #3
  toothAngles[3] = 170; //tooth #4
  toothAngles[4] = toothAngles[1] + 180;
  toothAngles[5] = toothAngles[2] + 180;
  toothAngles[6] = toothAngles[3] + 180;
  toothAngles[7] = toothAngles[1] + 360;
  toothAngles[8] = toothAngles[2] + 360;
  toothAngles[9] = toothAngles[3] + 360;
  toothAngles[10] = toothAngles[1] + 540;
  toothAngles[11] = toothAngles[2] + 540;
}


void triggerPri_Subaru67(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap < triggerFilterTime)
  {
    return;
  }

  toothCurrentCount++; //Increment the tooth counter
  //Used to count the number of primary pulses that have occurred since the last
  //secondary. Is part of the noise filtering system.
  toothSystemCount++;
  //Flag this pulse as being a valid trigger (ie that it passed filters)
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  toothLastMinusOneToothTime = toothLastToothTime;
  toothLastToothTime = curTime;

  if (toothCurrentCount > 13) //can't have more than 12 teeth so have lost sync
  {
    toothCurrentCount = 0;
    currentStatus.hasSync = false;
    currentStatus.syncLossCounter++;
  }

  //Sync is determined by counting the number of cam teeth that have passed between the crank teeth
  switch (secondaryToothCount)
  {
  case 0:
    //If no teeth have passed, we can't do anything
    break;

  case 1:
    //Can't do anything with a single pulse from the cam either (We need either 2 or 3 pulses)
    if (toothCurrentCount == 5 || toothCurrentCount == 11)
    {
      currentStatus.hasSync = true;
    }
    else
    {
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      // we don't know if its 5 or 11, but we'll be right 50% of the time and
      // speed up getting sync 50%
      toothCurrentCount = 5;
    }
    secondaryToothCount = 0;
    break;

  case 2:
    if (toothCurrentCount == 8)
    {
      currentStatus.hasSync = true;
    }
    else
    {
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      toothCurrentCount = 8;
    }
    secondaryToothCount = 0;
    break;

  case 3:
    if (toothCurrentCount == 2)
    {
      currentStatus.hasSync = true;
    }
    else
    {
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      toothCurrentCount = 2;
    }
    secondaryToothCount = 0;
    break;

  default:
    //Almost certainly due to noise or cranking stop/start
    currentStatus.hasSync = false;
    BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
    currentStatus.syncLossCounter++;
    secondaryToothCount = 0;
    break;
  }

  //Check sync again
  if (currentStatus.hasSync)
  {
    //Locked timing during cranking. This is fixed at 10* BTDC.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
    {
      if (toothCurrentCount == 1 || toothCurrentCount == 7)
      {
        endCoil1and3Charge();
      }
      else if (toothCurrentCount == 4 || toothCurrentCount == 10)
      {
        endCoil2and4Charge();
      }
    }

    if (toothCurrentCount > 12) // done 720 degrees so increment rotation
    {
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //Set the last angle between teeth for better calc accuracy
    if (toothCurrentCount == 1) //Special case for tooth 1
    {
      triggerToothAngle = 55;
    }
    else if (toothCurrentCount == 2) //Special case for tooth 2
    {
      triggerToothAngle = 93;
    }
    else
    {
      triggerToothAngle = toothAngles[toothCurrentCount - 1] - toothAngles[toothCurrentCount - 2];
    }
    BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);


    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = toothAngles[toothCurrentCount - 1] + configPage4.triggerAngle;

      if ((configPage4.sparkMode != IGN_MODE_SEQUENTIAL))
      {
        crankAngle = ignitionLimits(toothAngles[toothCurrentCount - 1]);

        //Handle non-sequential tooth counts
        if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && toothCurrentCount > 6)
        {
          checkPerToothTiming(crankAngle, (toothCurrentCount - 6));
        }
        else
        {
          checkPerToothTiming(crankAngle, toothCurrentCount);
        }
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

void triggerSec_Subaru67(void)
{
  if (toothSystemCount == 0 || toothSystemCount == 3)
  {
    curTime2 = micros();
    curGap2 = curTime2 - toothLastSecToothTime;

    if (curGap2 > triggerSecFilterTime)
    {
      toothLastSecToothTime = curTime2;
      secondaryToothCount++;
      toothSystemCount = 0;

      if (secondaryToothCount > 1)
      {
        //Set filter at 25% of the current speed
        //Note that this can only be set on the 2nd or 3rd cam tooth in each set.
        triggerSecFilterTime = curGap2 >> 2;
      }
      else //Filter disabled
      {
        triggerSecFilterTime = 0;
      }
    }
  }
  else
  {
    //Sanity check
    if (toothSystemCount > 3)
    {
      toothSystemCount = 0;
      secondaryToothCount = 1;
      // impossible to have more than 3 crank teeth between cam teeth
      // - must have noise but can't have sync
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
    }
    secondaryToothCount = 0;
  }

}

uint16_t getRPM_Subaru67(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.startRevolutions > 0)
  {
    //As the tooth count is over 720 degrees
    tempRPM = stdGetRPM(CAM_SPEED);
  }

  return tempRPM;
}

int getCrankAngle_Subaru67(void)
{
  int crankAngle = 0;
  if (currentStatus.hasSync)
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

    //Perform a lookup of the fixed toothAngles array to find what the angle of
    //the last tooth passed was.
    crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

    //Estimate the number of degrees travelled since the last tooth}
    unsigned long const elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
    crankAngle += timeToAngleIntervalTooth(elapsedTime);

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
  }

  return crankAngle;
}

void triggerSetEndTeeth_Subaru67(void)
{
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    ignition_context_st &ignition1 = ignition_contexts[ignChannel1];
    ignition_context_st &ignition2 = ignition_contexts[ignChannel2];
    ignition_context_st &ignition3 = ignition_contexts[ignChannel3];
    ignition_context_st &ignition4 = ignition_contexts[ignChannel4];

    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 12;
      ignition2.endTooth = 3;
      ignition3.endTooth = 6;
      ignition4.endTooth = 9;
    }
    else
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 4;
      ignition3.endTooth = 7;
      ignition4.endTooth = 10;
    }
  }
  else
  {
    ignition_context_st &ignition1 = ignition_contexts[ignChannel1];
    ignition_context_st &ignition2 = ignition_contexts[ignChannel2];

    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 6;
      ignition2.endTooth = 3;
    }
    else
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 4;
    }
  }
}

static void attach_interrupts(void)
{
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_Subaru67, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_Subaru67, secondaryTriggerEdge);
}

decoder_handler_st const trigger_subaru_67 PROGMEM =
{
  .setup = triggerSetup_Subaru67,
  .primaryToothHandler = triggerPri_Subaru67,
  .secondaryToothHandler = triggerSec_Subaru67,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Subaru67,
  .get_crank_angle = getCrankAngle_Subaru67,
  .set_end_teeth = triggerSetEndTeeth_Subaru67,
  .attach_interrupts = attach_interrupts,
};

