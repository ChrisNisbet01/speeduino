#include "daihatsu_plus1.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"
#include "../../crank.h"

/** @} */

/** Daihatsu +1 trigger for 3 and 4 cylinder engines.
* Tooth equal to the number of cylinders are evenly spaced on the cam. No position sensing (Distributor is retained),
* so crank angle is a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_daihatsu Daihatsu (3  and 4 cyl.)
* @{
*/
void triggerSetup_Daihatsu(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerActualTeeth = configPage2.nCylinders + 1;
  triggerToothAngle = 720 / triggerActualTeeth; //The number of degrees that passes from tooth to tooth
  triggerFilterTime = MICROS_PER_MIN / MAX_RPM / configPage2.nCylinders; // Minimum time required between teeth
  triggerFilterTime = triggerFilterTime / 2; //Safety margin
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);

  //Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm
  //rather than 50rpm due to the potentially very high stall time on a 4
  //cylinder if we wait that long.
  unsigned const minimum_rpm = 90;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle) * 4U;

  if (configPage2.nCylinders == 3)
  {
    toothAngles[0] = 0; //tooth #1
    toothAngles[1] = 30; //tooth #2 (Extra tooth)
    toothAngles[2] = 240; //tooth #3
    toothAngles[3] = 480; //tooth #4
  }
  else
  {
    //Should be 4 cylinders here
    toothAngles[0] = 0; //tooth #1
    toothAngles[1] = 30; //tooth #2 (Extra tooth)
    toothAngles[2] = 180; //tooth #3
    toothAngles[3] = 360; //tooth #4
    toothAngles[4] = 540; //tooth #5
  }
}

void triggerPri_Daihatsu(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  {
    toothSystemCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    if (currentStatus.hasSync)
    {
      //Check if we're back to the beginning of a revolution
      if (toothCurrentCount == triggerActualTeeth)
      {
        toothCurrentCount = 1; //Reset the counter
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.hasSync = true;
        currentStatus.startRevolutions++; //Counter

        //Need to set a special filter time for the next tooth
        triggerFilterTime = 20; //Fix this later
      }
      else
      {
        toothCurrentCount++; //Increment the tooth counter
        setFilter(curGap); //Recalc the new filter value
      }

      if (configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
      {
        //This locks the cranking timing to 0 degrees BTDC (All the triggers allow for)
        if (toothCurrentCount == 1)
        {
          endCoil1Charge();
        }
        else if (toothCurrentCount == 2)
        {
          endCoil2Charge();
        }
        else if (toothCurrentCount == 3)
        {
          endCoil3Charge();
        }
        else if (toothCurrentCount == 4)
        {
          endCoil4Charge();
        }
      }
    }
    else //NO SYNC
    {
      if (toothSystemCount >= 3) //Need to have seen at least 3 teeth to determine SYNC
      {
        uint32_t targetTime;
        //We need to try and find the extra tooth (#2) which is located 30 degrees
        //after tooth #1
        //Aim for tooth times less than about 60 degrees
        if (configPage2.nCylinders == 3)
        {
          //Teeth are 240 degrees apart for 3 cylinder. 240/4 = 60
          targetTime = (toothLastToothTime -  toothLastMinusOneToothTime) / 4;
        }
        else
        {
          //Teeth are 180 degrees apart for 4 cylinder. (180*3)/8 = 67
          targetTime = ((toothLastToothTime -  toothLastMinusOneToothTime) * 3) / 8;
        }
        if (curGap < targetTime)
        {
          //Means we're on the extra tooth here
          toothCurrentCount = 2; //Reset the counter
          currentStatus.hasSync = true;
          triggerFilterTime = targetTime; //Lazy, but it works
        }
      }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  }
}

void triggerSec_Daihatsu(void) //Not required (Should never be called in the first place)
{
  return;
}

uint16_t getRPM_Daihatsu(void)
{
  uint16_t tempRPM = 0;

  //Disable special cranking processing for now
#ifdef INCLUDE_DAIHATSU_SPECIAL_CRANK_PROCESSING
  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    //Can't use standard cranking RPM function due to extra tooth
    if (currentStatus.hasSync)
    {
      if (toothCurrentCount == 2 || toothCurrentCount == 3)
      {
        tempRPM = currentStatus.RPM;
      }
      else
      {
        noInterrupts();

        uint32_t const tooth_time_delta =
          toothLastToothTime - toothLastMinusOneToothTime;

        crank.SetRevolutionTime(tooth_time_delta * (triggerActualTeeth - 1));

        interrupts();

        tempRPM = RpmFromRevolutionTimeUs(crank.revolutionTime);
      } //is tooth #2
    }
    else //No sync
    {
      tempRPM = 0;
    }
  }
  else
#endif
  {
    //Tracking over 2 crank revolutions
    tempRPM = stdGetRPM(CAM_SPEED);
  }

  return tempRPM;
}

int getCrankAngle_Daihatsu(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  uint32_t tempToothLastToothTime;
  int tempToothCurrentCount;
  int crankAngle;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  uint32_t const lastCrankAngleCalc = micros();

  interrupts();

  //Crank angle of the last tooth seen
  crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  uint32_t const elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += crank.timeToAngleDegPerMicroSec(elapsedTime);

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
    crankAngle += CRANK_ANGLE_MAX;
  }

  return crankAngle;
}

static void attach_interrupts(void)
{
  //No secondary input required for this pattern
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_Daihatsu, primaryTriggerEdge);
}

decoder_handler_st const trigger_daihatsu_plus1 PROGMEM =
{
  .setup = triggerSetup_Daihatsu,
  .primaryToothHandler = triggerPri_Daihatsu,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Daihatsu,
  .get_crank_angle = getCrankAngle_Daihatsu,
  .set_end_teeth = nullSetEndTeeth,
  .attach_interrupts = attach_interrupts,
};

