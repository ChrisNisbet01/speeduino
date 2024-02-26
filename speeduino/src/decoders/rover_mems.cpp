#include "rover_mems.h"
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

/*****************************************************************
 * Rover MEMS decoder
 * Covers multiple trigger wheels used interchangeably over the range of MEMS units
 * Specifically covers teeth patterns on the primary trigger (crank)
 * 3 gap 14 gap 2 gap 13 gap
 * 11 gap 5 gap 12 gap 4 gap
 * 2 gap 14 gap 3 gap 13 gap
 * 17 gap 17 gap
 *
 * Support no cam, single tooth Cam (or half moon cam), and multi tooth (5-3-2 teeth)
 *
 * @defgroup dec_rover_mems Rover MEMS all versions including T Series, O Series, Mini and K Series
 * @{
 */
// used for flywheel gap pattern matching
volatile uint32_t roverMEMSTeethSeen = 0;

void triggerSetup_RoverMEMS(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  for (unsigned i = 0; i < 10; i++)
  {
    // Repurpose ToothAngles to store data needed for this implementation.
    toothAngles[i] = 0;
  }

  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be disgarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 36U);
  // only 1 tooth on the wheel not 36
  triggerSecFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U);

  configPage4.triggerTeeth = 36;
  //The number of degrees that passes from tooth to tooth 360 / 36 theortical teeth
  triggerToothAngle = 360 / configPage4.triggerTeeth;
  //The number of physical teeth on the wheel. Need to fix now so we can identify
  //the wheel on the first rotation and not risk a type 1 wheel not being spotted
  triggerActualTeeth = 36;
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0; // current tooth
  secondaryToothCount = 0;
  secondaryLastToothCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  revolutionOne = 0;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

static void triggerRoverMEMSCommon(void)
{
  // pattern 1 isn't unique & if we don't have a cam we need special code to
  // identify if we're tooth 18 or 36 - this allows batch injection but not spark to run
  // as we have to be greater than 18 teeth when using the cam this code also works for that.
  if (toothCurrentCount > 18)
  {
    toothCurrentCount = 1;
    toothOneMinusOneTime = toothOneTime;
    toothOneTime = curTime;
    revolutionOne = !revolutionOne; //Flip sequential revolution tracker
  }

  //if Sequential fuel or ignition is in use, further checks are needed before determining sync
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL || configPage2.injLayout == INJ_SEQUENTIAL)
  {
    //If either fuel or ignition is sequential, only declare sync if the cam tooth
    //has been seen OR if the missing wheel is on the cam
    if (secondaryToothCount > 0 || configPage4.TrigSpeed == CAM_SPEED)
    {
      currentStatus.hasSync = true;
      //the engine is fully synced so clear the Half Sync bit
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
      //Reset the secondary tooth counter to prevent it overflowing
      if (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE)
      {
        secondaryToothCount = 0;
      }
    }
    else if (currentStatus.hasSync != true)
    {
      BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC);
    } //If there is primary trigger but no secondary we only have half sync.
  }
  else
  {
    //If nothing is using sequential, we have sync and also clear half sync bit
    currentStatus.hasSync = true;
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
  }

  if (currentStatus.hasSync)
  {
    currentStatus.startRevolutions++;
  }
  else
  {
    currentStatus.startRevolutions = 0;
  }
}

void triggerPri_RoverMEMS(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    // have we seen more than 1 tooth so we start processing
    if (toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
    {
      uint32_t const deltaLastToothTime = toothLastToothTime - toothLastMinusOneToothTime;

      //Begin the missing tooth detection
      //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one)
      targetGap = deltaLastToothTime + (deltaLastToothTime >> 1);
      currentStatus.hasSync = true;
      if (curGap > targetGap) // we've found a gap
      {
        // add the space for the gap and the tooth we've just seen so mulitply by 4
        roverMEMSTeethSeen = roverMEMSTeethSeen << 2;
        roverMEMSTeethSeen++; // add the tooth seen to the variable
        // Increment the tooth counter on the wheel
        // (used to spot a revolution and trigger igition timing)
        toothCurrentCount++;

        // the missing tooth gap messing up timing as it appears in different
        // parts of the cycle. Don't update setFilter as it would be wrong with the gap
        toothCurrentCount++;
      }
      else
      {
        //Regular (non-missing) tooth so update things
        // make a space, multiply by 2
        roverMEMSTeethSeen = roverMEMSTeethSeen << 1;
        roverMEMSTeethSeen++; // add the tooth seen
        //Increment the tooth counter on the wheel (used to spot a revolution)
        toothCurrentCount++;
        setFilter(curGap);
      }

      // reduce checks to minimise cpu load when looking for key point to
      // identify where we are on the wheel
      if (toothCurrentCount >= triggerActualTeeth)
      {
                                  //12345678901234567890123456789012
        if (roverMEMSTeethSeen == 0b11111101111111011111111110111111)
        {
          // Binary pattern for trigger pattern 9-7-10-6- (#5)
          if (toothAngles[ID_TOOTH_PATTERN] != 5)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 1;
            toothAngles[SKIP_TOOTH2] = 11;
            toothAngles[SKIP_TOOTH3] = 19;
            toothAngles[SKIP_TOOTH4] = 30;
            toothAngles[ID_TOOTH_PATTERN] = 5;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //123456789012345678901234567890123456
        else if (roverMEMSTeethSeen == 0b11011101111111111111101101111111)
        {
          // Binary pattern for trigger pattern 3-14-2-13- (#4)
          if (toothAngles[ID_TOOTH_PATTERN] != 4)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 8;
            toothAngles[SKIP_TOOTH2] = 11;
            toothAngles[SKIP_TOOTH3] = 25;
            toothAngles[SKIP_TOOTH4] = 27;
            toothAngles[ID_TOOTH_PATTERN] = 4;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //123456789012345678901234567890123456
        else if (roverMEMSTeethSeen == 0b11011011111111111111011101111111)
        {
          // Binary pattern for trigger pattern 2-14-3-13- (#3)
          if (toothAngles[ID_TOOTH_PATTERN] != 3)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 8;
            toothAngles[SKIP_TOOTH2] = 10;
            toothAngles[SKIP_TOOTH3] = 24;
            toothAngles[SKIP_TOOTH4] = 27;
            toothAngles[ID_TOOTH_PATTERN] = 3;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //12345678901234567890123456789012
        else if (roverMEMSTeethSeen == 0b11111101111101111111111110111101)
        {
          // Binary pattern for trigger pattern 11-5-12-4- (#2)
          if (toothAngles[ID_TOOTH_PATTERN] != 2)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 1;
            toothAngles[SKIP_TOOTH2] = 12;
            toothAngles[SKIP_TOOTH3] = 17;
            toothAngles[SKIP_TOOTH4] = 29;
            toothAngles[ID_TOOTH_PATTERN] = 2;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //12345678901234567890123456789012
        else if (roverMEMSTeethSeen == 0b11111111111101111111111111111101)
        {
          // Binary pattern for trigger pattern 17-17- (#1)
          if (toothAngles[ID_TOOTH_PATTERN] != 1)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 1;
            toothAngles[SKIP_TOOTH2] = 18;
            toothAngles[ID_TOOTH_PATTERN] = 1;
            // this should be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 2;
            triggerActualTeeth = 36; // should be 34 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }
        else if (toothCurrentCount > triggerActualTeeth + 1)
        {
          // no patterns match after a rotation when we only need 32 teeth to match,
          // we've lost sync
          currentStatus.hasSync = false;
          if (secondaryToothCount > 0)
            BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          else
            BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          currentStatus.syncLossCounter++;
        }
      }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

      crankAngle = ignitionLimits(crankAngle);
      if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && revolutionOne)
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount));
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

int getCrankAngle_RoverMEMS(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  bool tempRevolutionOne;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempRevolutionOne = revolutionOne;
  tempToothLastToothTime = toothLastToothTime;

  interrupts();

  //Number of teeth that have passed since tooth 1,
  //multiplied by the angle each tooth represents,
  //plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

  //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
  if (tempRevolutionOne && configPage4.TrigSpeed == CRANK_SPEED)
  {
    crankAngle += 360;
  }

  lastCrankAngleCalc = micros();
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  else if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += CRANK_ANGLE_MAX;
  }

  return crankAngle;
}

void triggerSec_RoverMEMS(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if ((toothLastSecToothTime == 0))
  {
    targetGap2 = curGap * 2;
    curGap2 = 0;
    toothLastSecToothTime = curTime2;
  }

  if (curGap2 >= triggerSecFilterTime)
  {
    secondaryToothCount++;
    toothLastSecToothTime = curTime2;

    //Record the VVT Angle
    if (configPage6.vvtEnabled > 0
        && (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE ||
            (configPage4.trigPatternSec == SEC_TRIGGER_5_3_2 && secondaryToothCount == 6)))
    {
      int16_t curAngle;

      curAngle = decoder.handler.get_crank_angle();
      while (curAngle > 360)
      {
        curAngle -= 360;
      }
      curAngle -= configPage4.triggerAngle; //Value at TDC
      if (configPage6.vvtMode == VVT_MODE_CLOSED_LOOP)
      {
        curAngle -= configPage10.vvtCLMinAng;
      }

      currentStatus.vvt1Angle = curAngle;
    }

    if (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE)
    {
      //Standard single tooth cam trigger
      revolutionOne = true;
      //Next secondary filter is half the current gap
      triggerSecFilterTime = curGap2 >> 1;
    }
    else if (configPage4.trigPatternSec == SEC_TRIGGER_5_3_2) // multi tooth cam
    {
      if (curGap2 < targetGap2) // ie normal tooth sized gap, not a single or double gap
      {
        //Next secondary filter is half the current gap
        triggerSecFilterTime = curGap2 >> 1;
        //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one)
        targetGap2 = curGap2 + (curGap2 >> 1);
      }
      else
      {
        // gap either single or double - nb remember we've got the tooth after the gap,
        // so on the 5 tooth pattern we'll see here tooth 6
        if (secondaryToothCount == 6)
        {
          // if we've got the tooth after the gap from reading 5 teeth we're on
          // cycle 360-720 & tooth 18-36
          revolutionOne = false;
          if (toothCurrentCount < 19)
          {
            toothCurrentCount += 18;
          }
        }
        else if (secondaryToothCount == 4)
        {
          // we've got the tooth after the gap from reading 3 teeth we're on
          // cycle 0-360 & tooth 1-18
          revolutionOne = true;
          if (toothCurrentCount > 17)
          {
            toothCurrentCount -= 18;
          }
        }
        else if (secondaryToothCount == 3)
        {
          // if we've got the tooth after the gap from reading 2 teeth we're on
          // cycle 0-360 & tooth 18-36
          revolutionOne = true;
          if (toothCurrentCount < 19)
          {
            toothCurrentCount += 18;
          }
        }
        // as we've had a gap we need to reset to this being the first tooth after the gap
        secondaryToothCount = 1;
      }
    }
  } //Trigger filter
}

uint16_t getRPM_RoverMEMS(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH1]
        && toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH2]
        && toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH3]
        && toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH4])
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM as the missing tooth messes the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

void triggerSetEndTeeth_RoverMEMS(void)
{
  int16_t tempIgnitionEndTooth[5];
  int16_t toothAdder = 0;

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = 36;
  }
  int16_t const toothAdderLimit = 36 + toothAdder;
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
  ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

  tempIgnitionEndTooth[1] = ((ignition1.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[1] > toothAdderLimit)
  {
    tempIgnitionEndTooth[1] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[1] <= 0)
  {
    tempIgnitionEndTooth[1] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[1] > toothAdderLimit)
  {
    tempIgnitionEndTooth[1] = toothAdderLimit;
  }

  tempIgnitionEndTooth[2] = ((ignition2.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[2] > toothAdderLimit)
  {
    tempIgnitionEndTooth[2] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[2] <= 0)
  {
    tempIgnitionEndTooth[2] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[2] > toothAdderLimit)
  {
    tempIgnitionEndTooth[2] = toothAdderLimit;
  }

  tempIgnitionEndTooth[3] = ((ignition3.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[3] > toothAdderLimit)
  {
    tempIgnitionEndTooth[3] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[3] <= 0)
  {
    tempIgnitionEndTooth[3] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[3] > toothAdderLimit)
  {
    tempIgnitionEndTooth[3] = toothAdderLimit;
  }

  tempIgnitionEndTooth[4] = ((ignition4.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[4] > toothAdderLimit)
  {
    tempIgnitionEndTooth[4] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[4] <= 0)
  {
    tempIgnitionEndTooth[4] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[4] > toothAdderLimit)
  {
    tempIgnitionEndTooth[4] = toothAdderLimit;
  }

  // take into account the missing teeth on the Rover flywheels
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    // check the calculated trigger tooth exists, if it doesn't use the previous tooth
    // nb the toothAngles[x] holds the tooth after the gap, hence the '-1' to see if it matches a gap

    for (unsigned tempCount = 1; tempCount < 5; tempCount++)
    {
      if (tempIgnitionEndTooth[tempCount] == toothAngles[1]
          || tempIgnitionEndTooth[tempCount] == toothAngles[2]
          || tempIgnitionEndTooth[tempCount] == toothAngles[3]
          || tempIgnitionEndTooth[tempCount] == toothAngles[4]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[1]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[2]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[3]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[4])
      {
        tempIgnitionEndTooth[tempCount]--;
      }
    }
  }
  else
  {
    for (unsigned tempCount = 1; tempCount < 5; tempCount++)
    {
      if (tempIgnitionEndTooth[tempCount] == toothAngles[1]
          || tempIgnitionEndTooth[tempCount] == toothAngles[2])
      {
        tempIgnitionEndTooth[tempCount]--;
      }
    }
  }

  ignition1.endTooth = tempIgnitionEndTooth[1];
  ignition2.endTooth = tempIgnitionEndTooth[2];
  ignition3.endTooth = tempIgnitionEndTooth[3];
  ignition4.endTooth = tempIgnitionEndTooth[4];
}

static void attach_interrupts(void)
{
  //Rover MEMs - covers multiple flywheel trigger combinations.
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_RoverMEMS, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_RoverMEMS, secondaryTriggerEdge);
}

decoder_handler_st const trigger_rover_mems PROGMEM =
{
  .setup = triggerSetup_RoverMEMS,
  .primaryToothHandler = triggerPri_RoverMEMS,
  .secondaryToothHandler = triggerSec_RoverMEMS,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_RoverMEMS,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = triggerSetEndTeeth_RoverMEMS,
  .attach_interrupts = attach_interrupts,
};

