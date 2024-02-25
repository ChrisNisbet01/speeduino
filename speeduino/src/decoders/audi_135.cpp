#include "audi_135.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */

/** Audi with 135 teeth on the crank and 1 tooth on the cam.
* This is very similar to the dual wheel decoder, however due to the 135 teeth
* not dividing evenly into 360,
* only every 3rd crank tooth is used in calculating the crank angle.
* This effectively makes it a 45 tooth dual wheel setup.
* @defgroup dec_audi135 Audi 135
* @{
*/
void triggerSetup_Audi135(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
   //135/3 = 45, 360/45 = 8 degrees every 3 teeth
  triggerToothAngle = 360 / (135 / 3);
  toothCurrentCount = 255; //Default value
  toothSystemCount = 0;
  //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = (unsigned long)(MICROS_PER_SEC / (MAX_RPM / 60U * 135UL));
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_Audi135(void)
{
  curTime = micros();
  curGap = curTime - toothSystemLastToothTime;
  if (curGap > triggerFilterTime || currentStatus.startRevolutions == 0)
  {
    toothSystemCount++;

    if (!currentStatus.hasSync)
    {
      toothLastToothTime = curTime;
    }
    else
    {
      if (toothSystemCount >= 3)
      {
        //We only proceed for every third tooth

        //Flag this pulse as being a valid trigger (ie that it passed filters)
        BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
        toothSystemLastToothTime = curTime;
        toothSystemCount = 0;
        toothCurrentCount++; //Increment the tooth counter

        if (toothCurrentCount == 1 || toothCurrentCount > 45)
        {
          toothCurrentCount = 1;
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;
          revolutionOne = !revolutionOne;
          currentStatus.startRevolutions++; //Counter
        }

        setFilter(curGap); //Recalc the new filter value

        toothLastMinusOneToothTime = toothLastToothTime;
        toothLastToothTime = curTime;
      } //3rd tooth check
    } // Sync check
  } // Trigger filter
}

void triggerSec_Audi135(void)
{
  if (!currentStatus.hasSync)
  {
    toothCurrentCount = 0;
    currentStatus.hasSync = true;
    //Need to set this to 3 so that the next primary tooth is counted
    toothSystemCount = 3;
  }
  else if (configPage4.useResync == 1)
  {
    toothCurrentCount = 0;
    toothSystemCount = 3;
  }
  else if (currentStatus.startRevolutions < 100 && toothCurrentCount != 45)
  {
    toothCurrentCount = 0;
  }

  revolutionOne = 1; //Sequential revolution reset
}

uint16_t getRPM_Audi135(void)
{
  return stdGetRPM(CRANK_SPEED);
}

int getCrankAngle_Audi135(void)
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
  tempToothLastToothTime = toothLastToothTime;
  tempRevolutionOne = revolutionOne;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Handle case where the secondary tooth was the last one seen
  if (tempToothCurrentCount == 0)
  {
    tempToothCurrentCount = 45;
  }

  //Number of teeth that have passed since tooth 1, multiplied by the angle each
  //tooth represents, plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle =
    ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
  if (tempRevolutionOne)
  {
    crankAngle += 360;
  }

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

void triggerSetEndTeeth_Audi135(void)
{
}

decoder_handler_st const trigger_audi_135 =
{
  .setup = triggerSetup_Audi135,
  .primaryToothHandler = triggerPri_Audi135,
  .secondaryToothHandler = triggerSec_Audi135,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Audi135,
  .get_crank_angle = getCrankAngle_Audi135,
  .set_end_teeth = triggerSetEndTeeth_Audi135,
};

