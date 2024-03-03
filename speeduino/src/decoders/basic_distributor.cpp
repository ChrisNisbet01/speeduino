#include "basic_distributor.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../utilities.h"
#include "../../auxiliary_pins.h"

/** @} */

/** Basic Distributor where tooth count is equal to the number of cylinders and
*   teeth are evenly spaced on the cam.
* No position sensing (Distributor is retained) so crank angle is
* a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_dist Basic Distributor
* @{
*/
void triggerSetup_BasicDistributor(bool initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerActualTeeth = configPage2.nCylinders;
  if (triggerActualTeeth == 0)
  {
    triggerActualTeeth = 1;
  }
  //The number of degrees that passes from tooth to tooth
  triggerToothAngle = 720U / triggerActualTeeth;
  // Minimum time required between teeth
  triggerFilterTime = MICROS_PER_MIN / MAX_RPM / configPage2.nCylinders;
  triggerFilterTime = triggerFilterTime / 2; //Safety margin
  triggerFilterTime = 0;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  toothCurrentCount = 0; //Default value
  BIT_SET(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  //Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm
  //rather than 50rpm due to the potentially very high stall time on a 4 cylinder
  //if we wait that long.
  if (configPage2.nCylinders <= 4U)
  {
    unsigned const minimum_rpm = 90;

    MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
  }
  else //Minimum 50rpm. (3200uS is the time per degree at 50rpm).
  {
    unsigned const minimum_rpm = 50;

    MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
  }

}

void triggerPri_BasicDistributor(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  if (curGap >= triggerFilterTime)
  {
    if (currentStatus.hasSync) //Recalc the new filter value
    {
      setFilter(curGap);
    }
    else
    {
      //If we don't yet have sync, ensure that the filter won't prevent future
      //valid pulses from being ignored.
      triggerFilterTime = 0;
    }

    //Check if we're back to the beginning of a revolution
    if (toothCurrentCount == triggerActualTeeth || !currentStatus.hasSync)
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.hasSync = true;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if (toothCurrentCount < triggerActualTeeth)
      {
        toothCurrentCount++;
      }
      else
      {
        //This means toothCurrentCount is greater than triggerActualTeeth, which is bad.
        //If we have sync here then there's a problem. Throw a sync loss
        if (currentStatus.hasSync)
        {
          currentStatus.syncLossCounter++;
          currentStatus.hasSync = false;
        }
      }
    }

    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    if (configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      singleCoilEndCharge(ignition_id_1);
      singleCoilEndCharge(ignition_id_2);
      singleCoilEndCharge(ignition_id_3);
      singleCoilEndCharge(ignition_id_4);
    }

    if (configPage2.perToothIgn)
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
      crankAngle = ignitionLimits((crankAngle));
      if (toothCurrentCount > triggerActualTeeth / 2)
      {
        checkPerToothTiming(crankAngle, toothCurrentCount - triggerActualTeeth / 2);
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  } //Trigger filter
}
void triggerSec_BasicDistributor(void) //Not required
{
  return;
}

uint16_t getRPM_BasicDistributor(void)
{
  uint16_t tempRPM;
  if (currentStatus.RPM < currentStatus.crankRPM || currentStatus.RPM < 1500)
  {
    tempRPM = crankingGetRPM(triggerActualTeeth, CAM_SPEED);
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
  }

  //Set the stall time to be twice the current RPM. This is a safe figure as
  //there should be no single revolution where this changes more than this
  MAX_STALL_TIME = revolutionTime << 1;

  if (MAX_STALL_TIME < 366667UL) //Check for 50rpm minimum
  {
    MAX_STALL_TIME = 366667UL;
  }

  return tempRPM;

}

int getCrankAngle_BasicDistributor(void)
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

  //Number of teeth that have passed since tooth 1, multiplied by the angle each
  //tooth represents, plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

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
    crankAngle += CRANK_ANGLE_MAX;
  }

  return crankAngle;
}

void triggerSetEndTeeth_BasicDistributor(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

  int const tempEndAngle =
    ignitionLimits(ignition1.endAngle - configPage4.triggerAngle);

  switch (configPage2.nCylinders)
  {
  case 4:
    if (tempEndAngle > 180 || tempEndAngle <= 0)
    {
      ignition1.endTooth = 2;
      ignition2.endTooth = 1;
    }
    else
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 2;
    }
    break;

  case 3: //Shared with 6 cylinder
  case 6:
  {
    ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);

    if (tempEndAngle > 120 && tempEndAngle <= 240)
    {
      ignition1.endTooth = 2;
      ignition2.endTooth = 3;
      ignition3.endTooth = 1;
    }
    else if (tempEndAngle > 240 || tempEndAngle <= 0)
    {
      ignition1.endTooth = 3;
      ignition2.endTooth = 1;
      ignition3.endTooth = 2;
    }
    else
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 2;
      ignition3.endTooth = 3;
    }
  }
    break;

  case 8:
  {
    ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
    ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

    if (tempEndAngle > 90 && tempEndAngle <= 180)
    {
      ignition1.endTooth = 2;
      ignition2.endTooth = 3;
      ignition3.endTooth = 4;
      ignition4.endTooth = 1;
    }
    else if (tempEndAngle > 180 && tempEndAngle <= 270)
    {
      ignition1.endTooth = 3;
      ignition2.endTooth = 4;
      ignition3.endTooth = 1;
      ignition4.endTooth = 2;
    }
    else if (tempEndAngle > 270 || tempEndAngle <= 0)
    {
      ignition1.endTooth = 4;
      ignition2.endTooth = 1;
      ignition3.endTooth = 2;
      ignition4.endTooth = 3;
    }
    else
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 2;
      ignition3.endTooth = 3;
      ignition4.endTooth = 4;
    }
  }
    break;
  }
}

static void attach_interrupts(void)
{
  // Basic distributor
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_BasicDistributor, primaryTriggerEdge);
}

decoder_handler_st const trigger_basic_distributor PROGMEM =
{
  .setup = triggerSetup_BasicDistributor,
  .primaryToothHandler = triggerPri_BasicDistributor,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_BasicDistributor,
  .get_crank_angle = getCrankAngle_BasicDistributor,
  .set_end_teeth = triggerSetEndTeeth_BasicDistributor,
  .attach_interrupts = attach_interrupts,
};

