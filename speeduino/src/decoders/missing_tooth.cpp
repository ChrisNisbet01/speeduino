#include "missing_tooth.h"
#include "decoders.h"
#include "triggers.h"
#include "../../crankMaths.h"
#include "../../bit_macros.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"

static inline void triggerRecordVVT1Angle(void)
{
  //Record the VVT Angle
  if (configPage6.vvtEnabled > 0 && revolutionOne)
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
      curAngle -= configPage10.vvtCL0DutyAng;
    }

    currentStatus.vvt1Angle =
      ANGLE_FILTER(curAngle << 1, configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);
  }
}

/** @} */

/** A (single) multi-tooth wheel with one of more 'missing' teeth.
* The first tooth after the missing one is considered number 1 and is the basis for the trigger angle.
* Note: This decoder does not currently support dual wheel (i.e. missing tooth + single tooth on cam).
* @defgroup dec_miss Missing tooth wheel
* @{
*/
void triggerSetup_missingTooth(bool initialisationComplete)
{
  UNUSED(initialisationComplete);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  //The number of degrees that passes from tooth to tooth
  triggerToothAngle = 360 / configPage4.triggerTeeth;

  if (configPage4.TrigSpeed == CAM_SPEED)
  {
    //Account for cam speed missing tooth
    triggerToothAngle = 720 / configPage4.triggerTeeth;
    BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  }

  //The number of physical teeth on the wheel.
  //Doing this here saves us a calculation each time in the interrupt
  triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth;
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth);
  if (configPage4.trigPatternSec == SEC_TRIGGER_4_1)
  {
    triggerSecFilterTime = MICROS_PER_MIN / MAX_RPM / 4U / 2U;
  }
  else
  {
    triggerSecFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U);
  }
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  checkSyncToothCount = configPage4.triggerTeeth >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  secondaryToothCount = 0;
  thirdToothCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;
  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm)
                    * triggerToothAngle
                    * (configPage4.triggerMissingTeeth + 1U);

  if (configPage4.TrigSpeed == CRANK_SPEED
      && (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
          || configPage2.injLayout == INJ_SEQUENTIAL))
  {
    BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  }
  else
  {
    BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  }
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

void triggerPri_missingTooth(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger.
  //(A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap < triggerFilterTime)
  {
    return;
  }
  toothCurrentCount++; //Increment the tooth counter
  //Flag this pulse as being a valid trigger (ie that it passed filters)
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  if (toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
  {
    bool isMissingTooth = false;

    /*
    Performance Optimisation:
    Only need to try and detect the missing tooth if:
    1. We don't have sync yet
    2. We have sync and are in the final 1/4 of the wheel (Missing tooth will/should never occur in the first 3/4)
    3. RPM is under 2000. This is to ensure that we don't interfere with strange
       timing when cranking or idling. Optimisation not really required at these speeds anyway
    */
    if (!currentStatus.hasSync
        || currentStatus.RPM < 2000
        || toothCurrentCount >= (3 * triggerActualTeeth >> 2))
    {
      //Begin the missing tooth detection
      //If the time between the current tooth and the last is greater than 1.5x
      //the time between the last tooth and the tooth before that, we make the
      //assertion that we must be at the first tooth after the gap
      uint32_t const deltaT = toothLastToothTime - toothLastMinusOneToothTime;
      if (configPage4.triggerMissingTeeth == 1)
      {
        //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one)
        targetGap = deltaT + (deltaT >> 1);
      }
      else
      {
        //Multiply by 2 (Checks for a gap 2x greater than the last one)
        targetGap = deltaT * configPage4.triggerMissingTeeth;
      }

      if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
      {
        curGap = 0;
      }

      if (curGap > targetGap || toothCurrentCount > triggerActualTeeth)
      {
        //Missing tooth detected
        isMissingTooth = true;
        if (toothCurrentCount < triggerActualTeeth && currentStatus.hasSync)
        {
          //This occurs when we're at tooth #1, but haven't seen all the other teeth.
          //This indicates a signal issue so we flag lost sync so this will
          //attempt to resync on the next revolution.
          currentStatus.hasSync = false;
          //No sync at all, so also clear HalfSync bit.
          BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          currentStatus.syncLossCounter++;
        }
        else
        {
          //This is to handle a special case on startup where sync can be obtained
          //and the system immediately thinks the revs have jumped:
          if (currentStatus.hasSync || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC))
          {
            //Add an extra revolution count if we're running at cam speed
            currentStatus.startRevolutions += 1 + (configPage4.TrigSpeed == CAM_SPEED);
          }
          else
          {
            currentStatus.startRevolutions = 0;
          }

          toothCurrentCount = 1;
          // at tooth one check if the cam sensor is high or low in poll level mode
          if (configPage4.trigPatternSec == SEC_TRIGGER_POLL)
          {
            revolutionOne = configPage4.PollLevelPolarity == Trigger2.read();
          }
          else //Flip sequential revolution tracker if poll level is not used
          {
            revolutionOne = !revolutionOne;
          }
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;

          //if Sequential fuel or ignition is in use, further checks are needed
          //before determining sync
          if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
              || configPage2.injLayout == INJ_SEQUENTIAL)
          {
            //If either fuel or ignition is sequential, only declare sync if the
            //cam tooth has been seen OR if the missing wheel is on the cam
            if (secondaryToothCount > 0
                || configPage4.TrigSpeed == CAM_SPEED
                || configPage4.trigPatternSec == SEC_TRIGGER_POLL
                || configPage2.strokes == TWO_STROKE)
            {
              currentStatus.hasSync = true;
              //the engine is fully synced so clear the Half Sync bit
              BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
            }
            else if (!currentStatus.hasSync)
            {
              //If there is primary trigger but no secondary we only have half sync.
              BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC);
            }
          }
          else //If nothing is using sequential, we have sync and also clear half sync bit
          {
            currentStatus.hasSync = true;
            BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          }

          //Reset the secondary tooth counter to prevent it overflowing,
          //done outside of sequential as v6 & v8 engines could be batch firing
          //with VVT that needs the cam resetting
          if (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE
              || configPage4.trigPatternSec == SEC_TRIGGER_TOYOTA_3)
          {
            secondaryToothCount = 0;
          }

          //This is used to prevent a condition where serious intermittent signals
          //(e.g. someone furiously plugging the sensor wire in and out)
          //can leave the filter in an unrecoverable state
          triggerFilterTime = 0;
          toothLastMinusOneToothTime = toothLastToothTime;
          toothLastToothTime = curTime;
          //The tooth angle is double at this point
          BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
        }
      }
    }

    if (!isMissingTooth)
    {
      //Regular (non-missing) tooth
      setFilter(curGap);
      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
      BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
    }
  }
  else
  {
    //We fall here on initial startup when enough teeth have not yet been seen
    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  }


  //NEW IGNITION MODE
  if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
  {
    int16_t crankAngle =
      ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
    uint16_t currentTooth;

    if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
        && revolutionOne
        && configPage4.TrigSpeed == CRANK_SPEED
        && configPage2.strokes == FOUR_STROKE)
    {
      crankAngle += 360;
      currentTooth = configPage4.triggerTeeth + toothCurrentCount;
    }
    else
    {
      currentTooth = toothCurrentCount;
    }
    crankAngle = ignitionLimits(crankAngle);
    checkPerToothTiming(crankAngle, currentTooth);
  }
}

void triggerSec_missingTooth(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if (toothLastSecToothTime == 0)
  {
    curGap2 = 0;
    toothLastSecToothTime = curTime2;
  }

  if (curGap2 >= triggerSecFilterTime)
  {
    uint32_t const deltaT = toothLastSecToothTime - toothLastMinusOneSecToothTime;

    switch (configPage4.trigPatternSec)
    {
    case SEC_TRIGGER_4_1:
      //If the time between the current tooth and the last is greater than 1.5x
      //the time between the last tooth and the tooth before that, we make the
      //assertion that we must be at the first tooth after the gap.
      targetGap2 = deltaT + (deltaT >> 1);
      toothLastMinusOneSecToothTime = toothLastSecToothTime;
      if (curGap2 >= targetGap2 || secondaryToothCount > 3)
      {
        secondaryToothCount = 1;
        revolutionOne = 1; //Sequential revolution reset
        //This is used to prevent a condition where serious intermitent signals
        //(e.g.someone furiously plugging the sensor wire in and out) can leave
        //the filter in an unrecoverable state
        triggerSecFilterTime = 0;
        triggerRecordVVT1Angle();
      }
      else
      {
        //Set filter at 25% of the current speed. Filter can only be recalc'd
        //for the regular teeth, not the missing one.
        triggerSecFilterTime = curGap2 >> 2;
        secondaryToothCount++;
      }
      break;

    case SEC_TRIGGER_SINGLE:
      //Standard single tooth cam trigger
      revolutionOne = 1; //Sequential revolution reset
      //Next secondary filter is half the current gap
      triggerSecFilterTime = curGap2 >> 1;
      secondaryToothCount++;
      triggerRecordVVT1Angle();
      break;

    case SEC_TRIGGER_TOYOTA_3:
      // designed for Toyota VVTI (2JZ) engine - 3 triggers on the cam.
      // the 2 teeth for this are within 1 rotation (1 tooth first 360, 2 teeth second 360)
      secondaryToothCount++;
      if (secondaryToothCount == 2)
      {
        revolutionOne = 1; // sequential revolution reset
        triggerRecordVVT1Angle();
      }
      //Next secondary filter is 25% the current gap, done here so we don't get
      //a great big gap for the 1st tooth
      triggerSecFilterTime = curGap2 >> 2;
      break;
    }
    toothLastSecToothTime = curTime2;
  } //Trigger filter
}

void triggerThird_missingTooth(void)
{
  //Record the VVT2 Angle (the only purpose of the third trigger)
  //NB: no filtering of this signal with current implementation unlike Cam (VVT1)

  int16_t curAngle;
  curTime3 = micros();
  curGap3 = curTime3 - toothLastThirdToothTime;

  //Safety check for initial startup
  if (toothLastThirdToothTime == 0)
  {
    curGap3 = 0;
    toothLastThirdToothTime = curTime3;
  }

  if (curGap3 >= triggerThirdFilterTime)
  {
    thirdToothCount++;
    //Next third filter is 25% the current gap
    triggerThirdFilterTime = curGap3 >> 2;

    curAngle = decoder.handler.get_crank_angle();
    while (curAngle > 360)
    {
      curAngle -= 360;
    }
    curAngle -= configPage4.triggerAngle; //Value at TDC
    if (configPage6.vvtMode == VVT_MODE_CLOSED_LOOP)
    {
      curAngle -= configPage4.vvt2CL0DutyAng;
    }
    currentStatus.vvt2Angle =
      ANGLE_FILTER(curAngle << 1, configPage4.ANGLEFILTER_VVT, currentStatus.vvt2Angle);

    toothLastThirdToothTime = curTime3;
  }
}

static uint16_t __attribute__((noinline))
calcEndTeeth_missingTooth(int endAngle, uint8_t toothAdder)
{
  //Temp variable used here to avoid potential issues if a trigger interrupt
  //occurs part way through this function
  int16_t tempEndTooth;
#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(endAngle - configPage4.triggerAngle, &divTriggerToothAngle);
#else
  tempEndTooth = (endAngle - (int16_t)configPage4.triggerAngle) / (int16_t)triggerToothAngle;
#endif

  //For higher tooth count triggers, add a 1 tooth margin to allow for calculation time.
  if (configPage4.triggerTeeth > 12U)
  {
    tempEndTooth--;
  }

  // Clamp to tooth count
  return clampToActualTeeth(clampToToothCount(tempEndTooth, toothAdder), toothAdder);
}

void triggerSetEndTeeth_missingTooth(void)
{
  uint8_t toothAdder = 0;

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
      && configPage4.TrigSpeed == CRANK_SPEED
      && configPage2.strokes == FOUR_STROKE)
  {
    toothAdder = configPage4.triggerTeeth;
  }

  ignitions.ignition(ignChannel1).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel1).endAngle, toothAdder);
  ignitions.ignition(ignChannel2).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel2).endAngle, toothAdder);
  ignitions.ignition(ignChannel3).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel3).endAngle, toothAdder);
  ignitions.ignition(ignChannel4).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel4).endAngle, toothAdder);
#if IGN_CHANNELS >= 5
  ignitions.ignition(ignChannel5).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel5).endAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 6
  ignitions.ignition(ignChannel6).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel6).endAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 7
  ignitions.ignition(ignChannel7).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel7).endAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 8
  ignitions.ignition(ignChannel8).endTooth =
    calcEndTeeth_missingTooth(ignitions.ignition(ignChannel8).endAngle, toothAdder);
#endif
}

uint16_t getRPM_missingTooth(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (toothCurrentCount != 1)
    {
       //Account for cam speed
      tempRPM = crankingGetRPM(configPage4.triggerTeeth, configPage4.TrigSpeed == CAM_SPEED);
    }
    else //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
    {
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(configPage4.TrigSpeed == CAM_SPEED); //Account for cam speed
  }
  return tempRPM;
}

int getCrankAngle_missingTooth(void)
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

  //Number of teeth that have passed since tooth 1, multiplied by the angle
  //each tooth represents, plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle)
    + configPage4.triggerAngle;

  //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
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

static void attach_interrupts(void)
{
  // Attach the crank trigger wheel interrupt
  // (Hall sensor drags to ground when triggering)
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_missingTooth, primaryTriggerEdge);

  if (BIT_CHECK(decoderState, BIT_DECODER_HAS_SECONDARY))
  {
    secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;
    attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_missingTooth, secondaryTriggerEdge);
  }

  if (configPage10.vvt2Enabled > 0)
  {
    // we only need this for vvt2, so not really needed if it's not used
    tertiaryTriggerEdge = (configPage10.TrigEdgeThrd == 0) ? RISING : FALLING;
    attachInterrupt(digitalPinToInterrupt(Trigger3.pin), triggerThird_missingTooth, tertiaryTriggerEdge);
  }
}

decoder_handler_st const trigger_missing_tooth PROGMEM =
{
  .setup = triggerSetup_missingTooth,
  .primaryToothHandler = triggerPri_missingTooth,
  .secondaryToothHandler = triggerSec_missingTooth,
  .tertiaryToothHandler = triggerThird_missingTooth,
  .get_rpm = getRPM_missingTooth,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = triggerSetEndTeeth_missingTooth,
  .attach_interrupts = attach_interrupts,
};

