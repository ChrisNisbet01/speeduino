#include "miata_9905.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"

/** @} */

/** Miata '99 to '05 with 4x 70 degree duration teeth running at cam speed.
Teeth believed to be at the same angles as the 4g63 decoder.
Tooth #1 is defined as the next crank tooth after the crank signal is HIGH when*
*the cam signal is falling.
Tooth number one is at 355* ATDC.
* (See: www.forum.diyefi.org/viewtopic.php?f=56&t=1077)
* @defgroup miata_99_05 Miata '99 to '05
* @{
*/
void triggerSetup_Miata9905(bool const initialisationComplete)
{
  triggerToothAngle = 90; //The number of degrees that passes from tooth to tooth (primary)
  toothCurrentCount = 99; //Fake tooth count represents no sync
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  triggerActualTeeth = 8;

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on all the time
    secondaryToothCount = 0;
    toothLastToothTime = micros();
  }
  else
  {
    toothLastToothTime = 0;
  }
  toothLastMinusOneToothTime = 0;

  //Note that these angles are for every rising and falling edge
  toothAngles[0] = 710; //
  toothAngles[1] = 100; //First crank pulse after the SINGLE cam pulse
  toothAngles[2] = 170; //
  toothAngles[3] = 280; //
  toothAngles[4] = 350; //
  toothAngles[5] = 460; //First crank pulse AFTER the DOUBLE cam pulse
  toothAngles[6] = 530; //
  toothAngles[7] = 640; //

  unsigned const minimum_rpm = 50;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
  //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerFilterTime = 1500;
  triggerSecFilterTime = 0; //Need to figure out something better for this
  BIT_SET(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_Miata9905(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  if (curGap >= triggerFilterTime || currentStatus.startRevolutions == 0)
  {
    toothCurrentCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    if (toothCurrentCount == triggerActualTeeth + 1)
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if (!currentStatus.hasSync || configPage4.useResync)
      {
        if (secondaryToothCount == 2)
        {
          toothCurrentCount = 6;
          currentStatus.hasSync = true;
        }
      }
    }

    if (currentStatus.hasSync)
    {
      //Whilst this is an uneven tooth pattern, if the specific angle between
      //the last 2 teeth is specified, 1st deriv prediction can be used
      if (configPage4.triggerFilter == 1 || currentStatus.RPM < 1400)
      {
        //Lite filter
        //Trigger filter is set to whatever time it took to do 70 degrees
        //(Next trigger is 110 degrees away)
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7)
        {
          triggerToothAngle = 70;
          triggerFilterTime = curGap;
        }
        else
        {
          //Trigger filter is set to (110*3)/8=41.25=41 degrees
          //(Next trigger is 70 degrees away).
          triggerToothAngle = 110;
          triggerFilterTime = (curGap * 3) >> 3;
        }
      }
      else if (configPage4.triggerFilter == 2)
      {
        //Medium filter level
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7) //87.5 degrees with a target of 110
        {
          triggerToothAngle = 70;
          triggerFilterTime = (curGap * 5) >> 2;
        }
        else //55 degrees with a target of 70
        {
          triggerToothAngle = 110;
          triggerFilterTime = (curGap >> 1);
        }
      }
      else if (configPage4.triggerFilter == 3)
      {
        //Aggressive filter level
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7) //96.26 degrees with a target of 110
        {
          triggerToothAngle = 70;
          triggerFilterTime = (curGap * 11) >> 3;
        }
        else //61.87 degrees with a target of 70
        {
          triggerToothAngle = 110;
          triggerFilterTime = (curGap * 9) >> 5;
        }
      }
      else if (configPage4.triggerFilter == 0)
      {
        //trigger filter is turned off.
        triggerFilterTime = 0;
        triggerSecFilterTime = 0;
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7) //96.26 degrees with a target of 110
        {
          triggerToothAngle = 70;
        }
        else
        {
          triggerToothAngle = 110;
        }
      }

      //EXPERIMENTAL!
      //New ignition mode is ONLY available on 9905 when the trigger angle is
      //set to the stock value of 0.
      if (configPage2.perToothIgn && configPage4.triggerAngle == 0 && currentStatus.advance > 0)
      {
        int16_t crankAngle = ignitionLimits(toothAngles[toothCurrentCount - 1]);

        //Handle non-sequential tooth counts
        if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && toothCurrentCount > configPage2.nCylinders)
        {
          checkPerToothTiming(crankAngle, toothCurrentCount - configPage2.nCylinders);
        }
        else
        {
          checkPerToothTiming(crankAngle, toothCurrentCount);
        }
      }
    } //Has sync

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //The +30 here is a safety margin. When switching from fixed timing to normal,
    //there can be a situation where a pulse started when fixed and ending when
    //in normal mode causes problems. This prevents that.
    unsigned const rpm_safety_margin = 30;

    if (currentStatus.RPM < currentStatus.crankRPM + rpm_safety_margin && configPage4.ignCranklock)
    {
      if (toothCurrentCount == 1 || toothCurrentCount == 5)
      {
        endCoil1and3Charge();
      }
      else if (toothCurrentCount == 3 || toothCurrentCount == 7)
      {
        endCoil2and4Charge();
      }
    }
    secondaryToothCount = 0;
  } //Trigger filter

}

void triggerSec_Miata9905(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) || !currentStatus.hasSync)
  {
    //If this is removed, can have trouble getting sync again after the
    //engine is turned off (but ECU not reset).
    triggerFilterTime = 1500;
  }

  if (curGap2 >= triggerSecFilterTime)
  {
    toothLastSecToothTime = curTime2;
    lastGap = curGap2;
    secondaryToothCount++;

    //TODO Add some secondary filtering here

    //Record the VVT tooth time
    if (toothCurrentCount == 1 && curTime2 > toothLastToothTime)
    {
      lastVVTtime = curTime2 - toothLastToothTime;
    }
  }
}

uint16_t getRPM_Miata9905(void)
{
  //During cranking, RPM is calculated 4 times per revolution, once for each
  //tooth on the crank signal.
  //Because these signals aren't even (Alternating 110 and 70 degrees), this
  //needs a special function
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM && currentStatus.hasSync)
  {
    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      tempRPM = 0;
    }
    else
    {
      int tempToothAngle;
      uint32_t toothTime;

      noInterrupts();

      tempToothAngle = triggerToothAngle;
      //Note that trigger tooth angle changes between 70 and 110 depending on
      //the last tooth that was seen
      toothTime = toothLastToothTime - toothLastMinusOneToothTime;

      interrupts();

      toothTime = toothTime * 36;
      tempRPM = ((uint32_t)tempToothAngle * (MICROS_PER_MIN / 10U)) / toothTime;
      SetRevolutionTime((10UL * toothTime) / tempToothAngle);
      MAX_STALL_TIME = 366667UL; // 50RPM
    }
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
    //Set the stall time to be twice the current RPM. This is a safe figure as
    //there should be no single revolution where this changes more than this
    MAX_STALL_TIME = revolutionTime << 1;
    if (MAX_STALL_TIME < 366667UL) //Check for 50rpm minimum
    {
      MAX_STALL_TIME = 366667UL;
    }
  }

  return tempRPM;
}

int getCrankAngle_Miata9905(void)
{
  int crankAngle = 0;

  {
    //This is the current angle ATDC the engine is at. This is the last known
    //position based on what tooth was last 'seen'. It is only accurate to the
    //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
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
    crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

    //Estimate the number of degrees travelled since the last tooth}
    uint32_t const elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
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
  }

  return crankAngle;
}

int getCamAngle_Miata9905(void)
{
  int16_t curAngle;//lastVVTtime is the time between tooth #1 (10* BTDC) and the single cam tooth.
  //All cam angles in in BTDC, so the actual advance angle is
  //370 - timeToAngleDegPerMicroSec(lastVVTtime) - <the angle of the cam at 0 advance>
  curAngle = 370 - timeToAngleDegPerMicroSec(lastVVTtime, degreesPerMicro) - configPage10.vvtCL0DutyAng;
  currentStatus.vvt1Angle =
    ANGLE_FILTER((curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);

  return currentStatus.vvt1Angle;
}

void triggerSetEndTeeth_Miata9905(void)
{
  ignition_context_st &ignition1 = ignition_contexts[ignChannel1];
  ignition_context_st &ignition2 = ignition_contexts[ignChannel2];
  ignition_context_st &ignition3 = ignition_contexts[ignChannel3];
  ignition_context_st &ignition4 = ignition_contexts[ignChannel4];

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 8;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4;
      ignition4.endTooth = 6;
    }
    else if (currentStatus.advance > 0)
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 3;
      ignition3.endTooth = 5;
      ignition4.endTooth = 7;
    }
  }
  else
  {
    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 4;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4; //Not used
      ignition4.endTooth = 2; //Not used
    }
    else if (currentStatus.advance > 0)
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 3;
      ignition3.endTooth = 1; //Not used
      ignition4.endTooth = 3; //Not used
    }
  }
}

static void attach_interrupts(void)
{
  //These may both need to change, not sure
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_Miata9905, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_Miata9905, secondaryTriggerEdge);
}

decoder_handler_st const trigger_miata_9905 PROGMEM =
{
  .setup = triggerSetup_Miata9905,
  .primaryToothHandler = triggerPri_Miata9905,
  .secondaryToothHandler = triggerSec_Miata9905,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Miata9905,
  .get_crank_angle = getCrankAngle_Miata9905,
  .set_end_teeth = triggerSetEndTeeth_Miata9905,
  .attach_interrupts = attach_interrupts,
};

