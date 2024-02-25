#include "jeep_2000.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */

/** Jeep 2000 - 24 crank teeth over 720 degrees, in groups of 4 ('91 to 2000 6 cylinder Jeep engines).
* Crank wheel is high for 360 crank degrees. Quite similar to the 24X setup.
* As we only need timing within 360 degrees, only 12 tooth angles are defined.
* Tooth number 1 represents the first tooth seen after the cam signal goes high.
* www.speeduino.com/forum/download/file.php?id=205
* @defgroup dec_jeep Jeep 2000 (6 cyl)
* @{
*/
void triggerSetup_Jeep2000(bool const initialisationComplete)
{
  triggerToothAngle = 0; //The number of degrees that passes from tooth to tooth (primary)
  toothAngles[0] = 174;
  toothAngles[1] = 194;
  toothAngles[2] = 214;
  toothAngles[3] = 234;
  toothAngles[4] = 294;
  toothAngles[5] = 314;
  toothAngles[6] = 334;
  toothAngles[7] = 354;
  toothAngles[8] = 414;
  toothAngles[9] = 434;
  toothAngles[10] = 454;
  toothAngles[11] = 474;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm).
  //Largest gap between teeth is 60 degrees.
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * 60U);

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on all the time
    toothCurrentCount = 13;
    toothLastToothTime = micros();
  }

  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_Jeep2000(void)
{
  //Indicates sync has not been achieved
  //(Still waiting for 1 revolution of the crank to take place)
  if (toothCurrentCount == 13)
  {
    currentStatus.hasSync = false;
  }
  else
  {
    curTime = micros();
    curGap = curTime - toothLastToothTime;
    if (curGap >= triggerFilterTime)
    {
      if (toothCurrentCount == 0)
      {
        toothCurrentCount = 1; //Reset the counter
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.hasSync = true;
        currentStatus.startRevolutions++; //Counter
        //There are groups of 4 pulses (Each 20 degrees apart), with each group
        //being 60 degrees apart. Hence #1 is always 60
        triggerToothAngle = 60;
      }
      else
      {
        toothCurrentCount++; //Increment the tooth counter
        //Calculate the last tooth gap in degrees
        triggerToothAngle =
          toothAngles[toothCurrentCount - 1] - toothAngles[toothCurrentCount - 2];
      }

      setFilter(curGap); //Recalc the new filter value

      //Flag this pulse as being a valid trigger (ie that it passed filters)
      BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
    } //Trigger filter
  } //Sync check
}
void triggerSec_Jeep2000(void)
{
  //All we need to do is reset the tooth count back to zero, indicating that
  //we're at the beginning of a new revolution
  toothCurrentCount = 0;
  return;
}

uint16_t getRPM_Jeep2000(void)
{
  return stdGetRPM(CRANK_SPEED);
}

int getCrankAngle_Jeep2000(void)
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
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  int crankAngle;
  //This is the special case to handle when the 'last tooth' seen was the cam tooth.
  //Since the tooth timings were taken on the previous crank tooth, the previous
  //crank tooth angle is used here, not cam angle.
  if (toothCurrentCount == 0)
  {
    crankAngle = 114 + configPage4.triggerAngle;
  }
  else
  {
    //Perform a lookup of the fixed toothAngles array to find what the angle of
    //the last tooth passed was.
    crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;
  }

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
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

  return crankAngle;
}

decoder_handler_st const trigger_jeep_2000 =
{
  .setup = triggerSetup_Jeep2000,
  .primaryToothHandler = triggerPri_Jeep2000,
  .secondaryToothHandler = triggerSec_Jeep2000,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Jeep2000,
  .get_crank_angle = getCrankAngle_Jeep2000,
  .set_end_teeth = nullSetEndTeeth,
};

