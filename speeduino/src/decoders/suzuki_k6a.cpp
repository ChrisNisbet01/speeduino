#include "suzuki_k6a.h"
#include "missing_tooth.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"
#include "../../globals.h"
#include "../../crank.h"

/** Suzuki K6A 3 cylinder engine

* (See: https://www.msextra.com/forums/viewtopic.php?t=74614)
* @defgroup Suzuki_K6A Suzuki K6A
* @{
*/
void triggerSetup_SuzukiK6A(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth (primary)
  //- set to a value, needs to be set per tooth
  triggerToothAngle = 90;
  toothCurrentCount = 99; //Fake tooth count represents no sync

  configPage4.TrigSpeed = CAM_SPEED;
  triggerActualTeeth = 7;
  toothCurrentCount = 1;
  curGap = curGap2 = curGap3 = 0;

  //Set a startup value here to avoid filter errors when starting.
  //This MUST have the initial check to prevent the fuel pump just staying on all the time
  if (!currentStatus.initialisationComplete)
  {
    toothLastToothTime = micros();
  }
  else
  {
    toothLastToothTime = 0;
  }
  toothLastMinusOneToothTime = 0;

  // based on data in msextra page linked to above we can deduce,
  // gap between rising and falling edge of a normal 70 degree tooth is 48 degrees,
  // this means the gap is 70 degrees - 48 degrees = 22 degrees.
  // assume this is constant for all similar sized gaps and teeth
  // sync tooth is 35 degrees - eyeball looks like the tooth is 50% tooth and 50% gap
  // so guess its 17 degrees and 18 degrees.

  // coded every tooth here in case you want to try "change" setting on the trigger setup
  // (this is defined in init.ino and what i've set it to, otherwise you need code
  // to select rising or falling in init.ino (steal it from another trigger)).
  // If you don't want change then drop the 'falling' edges listed below and half
  // the number of edges + reduce the triggerActualTeeth
  // nb as you can edit the trigger offset using rising or falling edge setup
  // below is irrelevant as you can adjust via the trigger ofset to cover the difference.

  // not using toothAngles[0] as i'm hoping it makes logic easier

/*
 |   170  | 70  |   170  | 70  | 35| 135  | 70  |
  -        -     -        -     -   -      -     -
 |1|------|2|---|3|------|4|---|5|-|6|----|7|---|1|...
 */
  toothAngles[1] = 0;   // 0 TDC cylinder 1,
  toothAngles[2] = 170; // 170 - end of cylinder 1, start of cylinder 3, trigger ignition for cylinder 3 on this tooth
  toothAngles[3] = 240; // 70 TDC cylinder 3
  toothAngles[4] = 410; // 170  - end of cylinder 3, start of cylinder2, trigger ignition for cylinder 2 on this tooth
  toothAngles[5] = 480; // 70 TDC cylinder 2
  toothAngles[6] = 515; // 35 Additional sync tooth
  toothAngles[7] = 650; // 135 end of cylinder 2, start of cylinder 1, trigger ignition for cylinder 1 on this tooth
  // 70 - gap to rotation to TDC1. array item 1 and 8 are the same,
  // code never gets here its for reference only
  toothAngles[8] = 720;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  MAX_STALL_TIME = 3333UL * triggerToothAngle;
  triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerSecFilterTime = 0; //Need to figure out something better for this
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  // never sure if we need to set this in this type of trigger
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  // we can never have half sync - its either full or none.
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
}

void triggerPri_SuzukiK6A(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  if (curGap >= triggerFilterTime || currentStatus.startRevolutions == 0)
  {
    toothCurrentCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    // now to figure out if its a normal tooth or the extra sync tooth
    // pattern is normally small tooth, big tooth, small tooth, big tooth.
    // The extra tooth breaks the pattern so it goes, big tooth (curGap3),
    // small tooth(curGap2), small tooth(curGap)
    // reuse curGap2 and curGap3 (from secondary and tertiary decoders) to store
    // previous tooth sizes as not needed in this decoder.

    if (curGap <= curGap2 && curGap2 <= curGap3)
    {
      // cur Gap is smaller than last gap & last gap is smaller than gap before that
      // - means we must be on sync tooth
      toothCurrentCount = 6; // set tooth counter to correct tooth
      currentStatus.hasSync = true;
    }

    curGap3 = curGap2; // update values for next time we're in the loop
    curGap2 = curGap;

    if (toothCurrentCount == triggerActualTeeth + 1 && currentStatus.hasSync)
    {
      // seen enough teeth to have a revolution of the crank
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      // increment for 2 revs as we do 720 degrees on the the crank
      currentStatus.startRevolutions = currentStatus.startRevolutions + 2;
    }
    else if (toothCurrentCount > triggerActualTeeth + 1)
    {
      // Lost sync
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      triggerFilterTime = 0;
      toothCurrentCount = 0;
    }

    // check gaps match with tooth to check we have sync
    // so if we *think* we've seen tooth 3, whose gap should be smaller than the
    // previous tooth & it isn't, then we've lost sync
    switch (toothCurrentCount)
    {
    case 1:
    case 3:
    case 5:
    case 6:
      // current tooth gap is bigger than previous tooth gap = syncloss
      // eg tooth 3 should be smaller than tooth 2 gap, if its not then we've
      // lost sync and the tooth 3 we've just seen isn't really tooth 3
      if (curGap > curGap2)
      {
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerFilterTime = 0;
        toothCurrentCount = 2;
      }
      break;

    case 2:
    case 4:
    case 7:
      // current tooth gap is smaller than the previous tooth gap = syncloss
      // eg tooth 2 should be bigger than tooth 1, if its not then we've got syncloss
      if (curGap < curGap2)
      {
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerFilterTime = 0;
        toothCurrentCount = 1;
      }
      break;
    }

    // Setup data to allow other areas of the system to work due to odd sized teeth
    // - this could be merged with sync checking above, left separate to keep
    // code clearer as its doing only one function at once
    // % of filter are not based on previous tooth size but expected next tooth size
    // triggerToothAngle is the size of the prevous tooth not the future tooth
    if (currentStatus.hasSync)
    {
      // Set tooth angle based on previous gap and triggerFilterTime based on
      // previous gap and next gap
      switch (toothCurrentCount)
      {
      case 2:
      case 4:
        // equivalent of tooth 1 except we've not done rotation code yet so its 8
        // 170 degree tooth, next tooth is 70
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 17 degrees
          triggerFilterTime = curGap >> 3;
          break;

        case 2: // 50 % 35 degrees
          triggerFilterTime = (curGap >> 3) + (curGap >> 4);
          break;

        case 3: // 75 % 52 degrees
          triggerFilterTime = (curGap >> 2) + (curGap >> 4);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 5:
        // 70 degrees, next tooth is 35
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 8 degrees
          triggerFilterTime = curGap >> 3;
          break;

        case 2: // 50 % 17 degrees
          triggerFilterTime = curGap >> 2;
          break;

        case 3: // 75 % 25 degrees
          triggerFilterTime = (curGap >> 2) + (curGap >> 3);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 6:
        // sync tooth (35 degree tooth), next tooth is 135
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 33 degrees
          triggerFilterTime = curGap;
          break;

        case 2: // 50 % 67 degrees
          triggerFilterTime = curGap * 2;
          break;

        case 3: // 75 % 100 degrees
          triggerFilterTime = curGap * 3;
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 7:
        // 135 degree tooth, next tooth is 70
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 17 degrees
          triggerFilterTime = curGap >> 3;
          break;

        case 2: // 50 % 35 degrees
          triggerFilterTime = curGap >> 2;
          break;

        case 3: // 75 % 52 degrees
          triggerFilterTime = (curGap >> 2) + (curGap >> 3);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 1:
      case 3:
        // 70 degree tooth, next tooth is 170
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 42 degrees
          triggerFilterTime = (curGap >> 1) + (curGap >> 3);
          break;

        case 2: // 50 % 85 degrees
          triggerFilterTime = curGap + (curGap >> 2);
          break;

        case 3: // 75 % 127 degrees
          triggerFilterTime = curGap + (curGap >> 1) + (curGap >> 2);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      }

      //NEW IGNITION MODE
      if (configPage2.perToothIgn)
      {
        int16_t crankAngle = toothAngles[toothCurrentCount] + configPage4.triggerAngle;

        crankAngle = ignitionLimits(crankAngle);
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    } // has sync
  } //Trigger filter
}

void triggerSec_SuzukiK6A(void)
{
  return;
}

uint16_t getRPM_SuzukiK6A(void)
{
  //Cranking code needs working out.

  uint16_t tempRPM;

  tempRPM = stdGetRPM(720);
  //Set the stall time to be twice the current RPM.
  //This is a safe figure as there should be no single revolution where this
  //changes more than this
  MAX_STALL_TIME = crank.revolutionTime << 1;
  if (MAX_STALL_TIME < 366667UL) //Check for 50rpm minimum
  {
    MAX_STALL_TIME = 366667UL;
  }

  return tempRPM;
}

int getCrankAngle_SuzukiK6A(void)
{
  int crankAngle = 0;

  //This is the current angle ATDC the engine is at.
  //This is the last known position based on what tooth was last 'seen'.
  //It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  uint32_t tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.
  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  uint32_t const lastCrankAngleCalc = micros();

  interrupts();

  //Perform a lookup of the fixed toothAngles array to find what the angle of
  //the last tooth passed was.
  crankAngle = toothAngles[tempToothCurrentCount] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  uint32_t const elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;

  switch (toothCurrentCount)
  {
  case 2:
  case 4:
    // equivalent of tooth 1 except we've not done rotation code yet so its 8
    // 170 degree tooth, next tooth is 70
    triggerToothAngle = 170;
    break;

  case 5:
    // 70 degrees, next tooth is 35
    triggerToothAngle = 70;
    break;

  case 6:
    // sync tooth, next tooth is 135
    triggerToothAngle = 35;
    break;

  case 7:
    // 135 degree tooth, next tooth is 70
    triggerToothAngle = 135;
    break;

  case 1:
  case 3:
    // 70 degree tooth, next tooth is 170
    triggerToothAngle = 70;
    break;
  }

  crankAngle += crank.timeToAngleDegPerMicroSec(elapsedTime);
  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle < 0)
  {
    crankAngle += 720;
  }

  return crankAngle;
}

// Assumes no advance greater than 48 degrees.
// Triggers on the tooth before the ignition event
void triggerSetEndTeeth_SuzukiK6A(void)
{
  byte nCount, bExit;

  //Temp variables are used here to avoid potential issues if a trigger interrupt
  //occurs part way through this function
  int16_t tempIgnitionEndTooth;

  tempIgnitionEndTooth = ignition_contexts[ignChannel1].endAngle - configPage4.triggerAngle;
  tempIgnitionEndTooth = ignitionLimits(tempIgnitionEndTooth);

  for (nCount = 1, bExit = false; nCount < 8 && !bExit; nCount++)
  {
    if (tempIgnitionEndTooth <= toothAngles[nCount])
    {
      // The tooth we want is the tooth prior to this one.
      tempIgnitionEndTooth = nCount - 1;
      if (tempIgnitionEndTooth <= 0)
      {
        tempIgnitionEndTooth = 7;
      }
      bExit = true;
    }
  }
  if (nCount == 8)
  {
    // didn't find a match, use tooth 7 as it must be greater than 7 but less than 1.
    tempIgnitionEndTooth = 7;
  }
  ignition_contexts[ignChannel1].endTooth = tempIgnitionEndTooth;

  tempIgnitionEndTooth = ignition_contexts[ignChannel2].endAngle - configPage4.triggerAngle;
  tempIgnitionEndTooth = ignitionLimits(tempIgnitionEndTooth);

  for (nCount = 1, bExit = false; nCount < 8 && !bExit; nCount++)
  {
    if (tempIgnitionEndTooth <= toothAngles[nCount])
    {
      // The tooth we want is the tooth prior to this one.
      tempIgnitionEndTooth = nCount - 1;
      if (tempIgnitionEndTooth <= 0)
      {
        tempIgnitionEndTooth = 7;
      }
      bExit = true; // force exit from loop
    }
  }
  if (nCount == 8)
  {
    // didn't find a match, use tooth 7 as it must be greater than 7 but less than 1.
    tempIgnitionEndTooth = 7;
  }

  ignition_contexts[ignChannel2].endTooth = tempIgnitionEndTooth;

  tempIgnitionEndTooth = ignition_contexts[ignChannel3].endAngle - configPage4.triggerAngle;
  tempIgnitionEndTooth = ignitionLimits(tempIgnitionEndTooth);

  for (nCount = 1, bExit = false; nCount < 8 && !bExit; nCount++)
  {
    if (tempIgnitionEndTooth <= toothAngles[nCount])
    {
      // The tooth we want is the tooth prior to this one.
      tempIgnitionEndTooth = nCount - 1;
      if (tempIgnitionEndTooth <= 0)
      {
        tempIgnitionEndTooth = 7;
      }
      bExit = true; // force exit from loop
    }
  }

  if (nCount == 8)
  {
    // didn't find a match, use tooth 7 as it must be greater than 7 but less than 1.
    tempIgnitionEndTooth = 7;
  }
  ignition_contexts[ignChannel1].endTooth = tempIgnitionEndTooth;
}

static void attach_interrupts(void)
{
  // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_SuzukiK6A, primaryTriggerEdge);
}

decoder_handler_st const trigger_suzuki_k6a PROGMEM =
{
  .setup = triggerSetup_SuzukiK6A,
  .primaryToothHandler = triggerPri_SuzukiK6A,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_SuzukiK6A,
  .get_crank_angle = getCrankAngle_SuzukiK6A,
  .set_end_teeth = triggerSetEndTeeth_SuzukiK6A,
  .attach_interrupts = attach_interrupts,
};

