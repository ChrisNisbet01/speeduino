#include "chrysler_ngc.h"
#include "missing_tooth.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"
#include "globals.h"

/** Chrysler NGC - a dedicated decoder for vehicles with 4, 6 and 8 cylinder NGC pattern.
4-cyl: 36+2-2 crank wheel and 7 tooth cam
6-cyl: 36-2+2 crank wheel and 12 tooth cam in 6 groups
8-cyl: 36-2+2 crank wheel and 15 tooth cam in 8 groups
The crank decoder uses the polarity of the missing teeth to determine position
The 4-cyl cam decoder uses the polarity of the missing teeth to determine position
The 6 and 8-cyl cam decoder uses the amount of teeth in the two previous groups of teeth to determine position
* @defgroup dec Chrysler NGC - 4, 6 and 8-cylinder
* @{
*/

void triggerSetup_NGC(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);

  //Primary trigger
  configPage4.triggerTeeth = 36; //The number of teeth on the wheel incl missing teeth.
  triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U) / (360U / triggerToothAngle);
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  toothLastMinusOneToothTime = 0;
  toothLastToothRisingTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;

  //Secondary trigger
  if (configPage2.nCylinders == 4)
  {
    //Two nearest edges are 36 degrees apart. Multiply by 2 for half cam speed.
    triggerSecFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U) / (360U / 36U) * 2U;
  }
  else
  {
    //Two nearest edges are 21 degrees apart. Multiply by 2 for half cam speed.
    triggerSecFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U) / (360U / 21U) * 2U;
  }
  secondaryToothCount = 0;
  toothSystemCount = 0;
  toothLastSecToothRisingTime = 0;
  toothLastSecToothTime = 0;
  toothLastMinusOneSecToothTime = 0;

  //toothAngles is reused to store the cam pattern, only used for 6 and 8 cylinder pattern
  if (configPage2.nCylinders == 6)
  {
    toothAngles[0] = 1; // Pos 0 is required to be the same as group 6 for easier math
    toothAngles[1] = 3; // Group 1 ...
    toothAngles[2] = 1;
    toothAngles[3] = 2;
    toothAngles[4] = 3;
    toothAngles[5] = 2;
    toothAngles[6] = 1;
    toothAngles[7] = 3; // Pos 7 is required to be the same as group 1 for easier math
  }
  else if (configPage2.nCylinders == 8)
  {
    toothAngles[0] = 3; // Pos 0 is required to be the same as group 8 for easier math
    toothAngles[1] = 1; // Group 1 ...
    toothAngles[2] = 1;
    toothAngles[3] = 2;
    toothAngles[4] = 3;
    toothAngles[5] = 2;
    toothAngles[6] = 2;
    toothAngles[7] = 1;
    toothAngles[8] = 3;
    toothAngles[9] = 1; // Pos 9 is required to be the same as group 1 for easier math
  }
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

void triggerPri_NGC(void)
{
  curTime = micros();
  // We need to know the polarity of the missing tooth to determine position
  if (Trigger.read())
  {
    toothLastToothRisingTime = curTime;
    return;
  }

  curGap = curTime - toothLastToothTime;
  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger.
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
    bool isMissingTooth = false;

    //Make sure we haven't enough tooth information to calculate missing tooth length
    if (toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
    {
      //Only check for missing tooth if we expect this one to be it or if we haven't found one yet
      if (toothCurrentCount == 17
          || toothCurrentCount == 35
          || !(!currentStatus.hasSync && BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)))
      {
        //If the time between the current tooth and the last is greater than 2x
        //the time between the last tooth and the tooth before that, we make the
        //assertion that we must be at the first tooth after the gap
        if (curGap > (toothLastToothTime - toothLastMinusOneToothTime) * 2)
        {
          isMissingTooth = true; //Missing tooth detected
          //This is used to prevent a condition where serious intermittent signals
          //(e.g. someone furiously plugging the sensor wire in and out) can
          //leave the filter in an unrecoverable state
          triggerFilterTime = 0;
          //The tooth angle is double at this point
          BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);

          // Figure out the polarity of the missing tooth by comparing how far ago the last tooth rose
          if (toothLastToothRisingTime - toothLastToothTime < curTime - toothLastToothRisingTime)
          {
            //Just passed the HIGH missing tooth
            toothCurrentCount = 1;

            toothOneMinusOneTime = toothOneTime;
            toothOneTime = curTime;

            if (currentStatus.hasSync)
            {
              currentStatus.startRevolutions++;
            }
            else
            {
              currentStatus.startRevolutions = 0;
            }
          }
          else
          {
            //Just passed the first tooth after the LOW missing tooth
            toothCurrentCount = 19;
          }

          //If Sequential fuel or ignition is in use, further checks are needed before determining sync
          if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL || configPage2.injLayout == INJ_SEQUENTIAL)
          {
            // Verify the tooth counters are valid and use this to determine current revolution
            if ((configPage2.nCylinders == 4 && ((toothCurrentCount == 1 && (secondaryToothCount == 1 || secondaryToothCount == 2)) || (toothCurrentCount == 19 && secondaryToothCount == 4)))
                || (configPage2.nCylinders == 6 && ((toothCurrentCount == 1 && (toothSystemCount == 1    || toothSystemCount == 2)) || (toothCurrentCount == 19 && (toothSystemCount == 2 || toothSystemCount == 3))))
                || (configPage2.nCylinders == 8 && ((toothCurrentCount == 1 && (toothSystemCount == 1    || toothSystemCount == 2)) || (toothCurrentCount == 19 && (toothSystemCount == 3 || toothSystemCount == 4)))))
            {
              revolutionOne = false;
              currentStatus.hasSync = true;
              BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
            }
            else if ((configPage2.nCylinders == 4 && ((toothCurrentCount == 1 && secondaryToothCount == 5) || (toothCurrentCount == 19 && secondaryToothCount == 7)))
                     || (configPage2.nCylinders == 6 && ((toothCurrentCount == 1 && (toothSystemCount == 4 || toothSystemCount == 5)) || (toothCurrentCount == 19 && (toothSystemCount == 5 || toothSystemCount == 6))))
                     || (configPage2.nCylinders == 8 && ((toothCurrentCount == 1 && (toothSystemCount == 5 || toothSystemCount == 6)) || (toothCurrentCount == 19 && (toothSystemCount == 7 || toothSystemCount == 8)))))
            {
              revolutionOne = true;
              currentStatus.hasSync = true;
              //the engine is fully synced so clear the Half Sync bit
              BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
            }
            else
            {
              // If tooth counters are not valid, set half sync bit
              if (currentStatus.hasSync)
              {
                currentStatus.syncLossCounter++;
              }
              currentStatus.hasSync = false;
              //If there is primary trigger but no secondary we only have half sync.
              BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC);
            }
          }
          else //If nothing is using sequential, we have sync and also clear half sync bit
          {
            currentStatus.hasSync = true;
            BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          }
        }
        else
        {
          // If we have found a missing tooth and don't get the next one at the
          // correct tooth we end up here -> Resync
          if (currentStatus.hasSync)
          {
            currentStatus.syncLossCounter++;
          }
          currentStatus.hasSync = false;
          BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
        }
      }

      if (!isMissingTooth)
      {
        //Regular (non-missing) tooth
        setFilter(curGap);
        BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      }
    }

    if (isMissingTooth)
    {
      // If we have a missing tooth, copy the gap from the previous tooth as
      // that is the correct normal tooth length
      toothLastMinusOneToothTime = curTime - (toothLastToothTime - toothLastMinusOneToothTime);
    }
    else
    {
      toothLastMinusOneToothTime = toothLastToothTime;
    }
    toothLastToothTime = curTime;

    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
          && revolutionOne
          && configPage4.TrigSpeed == CRANK_SPEED)
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, configPage4.triggerTeeth + toothCurrentCount);
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

void triggerSec_NGC4(void)
{
  //Only check the cam wheel for sequential operation
  if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && configPage2.injLayout != INJ_SEQUENTIAL)
  {
    return;
  }

  curTime2 = micros();

  // We need to know the polarity of the missing tooth to determine position
  if (Trigger2.read())
  {
    toothLastSecToothRisingTime = curTime2;
    return;
  }

  curGap2 = curTime2 - toothLastSecToothTime;

  if (curGap2 > triggerSecFilterTime)
  {
    //Make sure we have enough tooth information to calculate tooth lengths
    if (toothLastSecToothTime > 0 && toothLastMinusOneSecToothTime > 0)
    {
      if (secondaryToothCount > 0)
      {
        secondaryToothCount++;
      }

      // Check if we have a bigger gap, that is a long tooth
      uint32_t const deltaToothTime = toothLastSecToothTime - toothLastMinusOneSecToothTime;

      if (curGap2 >= deltaToothTime + (deltaToothTime >> 1))
      {
        // Check long tooth polarity
        if (toothLastSecToothRisingTime - toothLastSecToothTime < curTime2 - toothLastSecToothRisingTime)
        {
          //Just passed the HIGH missing tooth
          if (secondaryToothCount == 0 || secondaryToothCount == 8) // synced
          {
            secondaryToothCount = 1;
          }
          else if (secondaryToothCount > 0)
          {
            //Any other number of teeth seen means we missed something or
            //something extra was seen so attempt resync.
            secondaryToothCount = 0;
          }
        }
        else
        {
          //Just passed the first tooth after the LOW missing tooth
          if (secondaryToothCount == 0 || secondaryToothCount == 5)
          {
            secondaryToothCount = 5;
          }
          else if (secondaryToothCount > 0)
          {
            secondaryToothCount = 0;
          }
        }

        //This is used to prevent a condition where serious intermitent signals
        //(e.g. someone furiously plugging the sensor wire in and out) can leave
        //the filter in an unrecoverable state
        triggerSecFilterTime = 0;
      }
      else if (secondaryToothCount > 0)
      {
        //Set filter at 25% of the current speed. Filter can only be recalc'd
        //for the regular teeth, not the missing one.
        triggerSecFilterTime = curGap2 >> 2;
      }
    }

    toothLastMinusOneSecToothTime = toothLastSecToothTime;
    toothLastSecToothTime = curTime2;
  }
}

#define secondaryToothLastCount checkSyncToothCount

void triggerSec_NGC68(void)
{
  //Only check the cam wheel for sequential operation
  if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && configPage2.injLayout != INJ_SEQUENTIAL)
  {
    return;
  }

  curTime2 = micros();

  curGap2 = curTime2 - toothLastSecToothTime;

  if (curGap2 > triggerSecFilterTime)
  {
    //Make sure we have enough tooth information to calculate tooth lengths
    if (toothLastSecToothTime > 0 && toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
    {
      /*
       * Cam wheel can have a single tooth in a group which can screw up the "targetgap" calculations
       * Instead use primary wheel tooth gap as comparison as those values are
       * always correct.
       * 2.1 primary teeth are the same duration as one secondary tooth.
       */
      // Check if we have a bigger gap, that is missing teeth
      if (curGap2 >= (3 * (toothLastToothTime - toothLastMinusOneToothTime)))
      {
        //toothSystemCount > 0 means we have cam sync and identifies which group we have synced with
        //toothAngles is reused to store the cam pattern
        // Only check for cam sync if we have actually detected two groups and can get cam sync
        if (secondaryToothCount > 0 && secondaryToothLastCount > 0)
        {
          // Do a quick check if we already have cam sync
          if (toothSystemCount > 0
              && secondaryToothCount == (unsigned int)toothAngles[toothSystemCount + 1])
          {
            toothSystemCount++;
            if (toothSystemCount > configPage2.nCylinders)
            {
              toothSystemCount = 1;
            }
          }
          else
          {
            // Check for a pair of matching groups which tells us which group we are at,
            // this should only happen when we don't have cam sync
            toothSystemCount = 0; // We either haven't got cam sync yet or we lost cam sync
            for (byte group = 1; group <= configPage2.nCylinders; group++)
            {
              // Find a matching pattern/position
              if (secondaryToothCount == (unsigned int)toothAngles[group]
                  && secondaryToothLastCount == (byte)toothAngles[group - 1])
              {
                toothSystemCount = group;
                break;
              }
            }
          }
        }

        secondaryToothLastCount = secondaryToothCount;
        //This is the first tooth in this group
        secondaryToothCount = 1;

        //This is used to prevent a condition where serious intermitent signals
        //(e.g. someone furiously plugging the sensor wire in and out) can leave
        //the filter in an unrecoverable state
        triggerSecFilterTime = 0;
      }
      else if (secondaryToothCount > 0)
      {
        //Normal tooth
        secondaryToothCount++;
        triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed
      }
    }

    toothLastSecToothTime = curTime2;
  }
}

uint16_t getRPM_NGC(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at any of the missing teeth as it messes the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }

  return tempRPM;
}

static inline uint16_t calcSetEndTeeth_NGC_SkipMissing(uint16_t toothNum)
{
  if (toothNum == 17U || toothNum == 18U)
  {
    // These are missing teeth, so set the next one before instead
    return 16U;
  }
  if (toothNum == 35U || toothNum == 36U)
  {
    // These are missing teeth, so set the next one before instead
    return 34U;
  }
  if (toothNum == 53U || toothNum == 54U)
  {
    // These are missing teeth, so set the next one before instead
    return 52U;
  }
  if (toothNum > 70U)
  {
    // These are missing teeth, so set the next one before instead
    return 70U;
  }

  return toothNum;

}

static uint16_t __attribute__((noinline))
calcSetEndTeeth_NGC(int ignitionAngle, uint8_t toothAdder)
{
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;

#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divTriggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerToothAngle;
#endif

  return calcSetEndTeeth_NGC_SkipMissing(clampToToothCount(tempEndTooth - 1, toothAdder));
}

static void calcSetEndTeeth_NGC_ignition(ignition_context_st &ignition)
{
  byte toothAdder = 0;
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
      && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = configPage4.triggerTeeth;
  }

  ignition.endTooth = calcSetEndTeeth_NGC(ignition.endAngle, toothAdder);
}

void triggerSetEndTeeth_NGC(void)
{
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel1));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel2));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel3));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel4));
#if IGN_CHANNELS >= 6
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel5));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel6));
#endif

#if IGN_CHANNELS >= 8
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel7));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel8));
#endif
}

decoder_handler_st const trigger_ngc_4 =
{
  .setup = triggerSetup_NGC,
  .primaryToothHandler = triggerPri_NGC,
  .secondaryToothHandler = triggerSec_NGC4,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_NGC,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = triggerSetEndTeeth_NGC,
};

decoder_handler_st const trigger_ngc_68 =
{
  .setup = triggerSetup_NGC,
  .primaryToothHandler = triggerPri_NGC,
  .secondaryToothHandler = triggerSec_NGC68,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_NGC,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = triggerSetEndTeeth_NGC,
};

