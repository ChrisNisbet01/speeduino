#include "24x.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */

/** GM 24X Decoder (eg early LS1 1996-2005).
Note: Useful references:
*
- www.vems.hu/wiki/index.php?page=MembersPage%2FJorgenKarlsson%2FTwentyFourX

Provided that the cam signal is used, this decoder simply counts the teeth and then looks their angles up against a lookup table. The cam signal is used to determine tooth #1
* @defgroup dec_gm GM 24X
* @{
*/
void triggerSetup_24X(bool const initialisationComplete)
{
  triggerToothAngle = 15; //The number of degrees that passes from tooth to tooth (primary)
  toothAngles[0] = 12;
  toothAngles[1] = 18;
  toothAngles[2] = 33;
  toothAngles[3] = 48;
  toothAngles[4] = 63;
  toothAngles[5] = 78;
  toothAngles[6] = 102;
  toothAngles[7] = 108;
  toothAngles[8] = 123;
  toothAngles[9] = 138;
  toothAngles[10] = 162;
  toothAngles[11] = 177;
  toothAngles[12] = 183;
  toothAngles[13] = 198;
  toothAngles[14] = 222;
  toothAngles[15] = 237;
  toothAngles[16] = 252;
  toothAngles[17] = 258;
  toothAngles[18] = 282;
  toothAngles[19] = 288;
  toothAngles[20] = 312;
  toothAngles[21] = 327;
  toothAngles[22] = 342;
  toothAngles[23] = 357;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the init check to prevent the fuel pump just staying on all the time.
    toothCurrentCount = 25;
    toothLastToothTime = micros();
  }

  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_24X(void)
{
  //Indicates sync has not been achieved
  //(Still waiting for 1 revolution of the crank to take place)
  if (toothCurrentCount == 25)
  {
    currentStatus.hasSync = false;
  }
  else
  {
    curTime = micros();
    curGap = curTime - toothLastToothTime;

    if (toothCurrentCount == 0)
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      revolutionOne = !revolutionOne; //Sequential revolution flip
      currentStatus.hasSync = true;
      currentStatus.startRevolutions++; //Counter
      triggerToothAngle = 15; //Always 15 degrees for tooth #15
    }
    else
    {
      toothCurrentCount++; //Increment the tooth counter
      //Calculate the last tooth gap in degrees
      triggerToothAngle =
        toothAngles[toothCurrentCount - 1] - toothAngles[toothCurrentCount - 2];
    }

    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothLastToothTime = curTime;
  }
}

void triggerSec_24X(void)
{
  //All we need to do is reset the tooth count back to zero, indicating that
  //we're at the beginning of a new revolution
  toothCurrentCount = 0;
  revolutionOne = 1; //Sequential revolution reset
}

uint16_t getRPM_24X(void)
{
  return stdGetRPM(CRANK_SPEED);
}

int getCrankAngle_24X(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount, tempRevolutionOne;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  tempRevolutionOne = revolutionOne;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  int crankAngle;

  //This is the special case to handle when the 'last tooth' seen was the cam tooth.
  //0 is the angle at which the crank tooth goes high (Within 360 degrees).
  if (tempToothCurrentCount == 0)
  {
    crankAngle = configPage4.triggerAngle;
  }
  else
  {
    //Perform a lookup of the fixed toothAngles array to find what the angle of
    //the last tooth passed was.
    crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;
  }

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
  if (tempRevolutionOne == 1)
  {
    crankAngle += 360;
  }

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

void triggerSetEndTeeth_24X(void)
{
}

decoder_handler_st const trigger_24X =
{
  .setup = triggerSetup_24X,
  .primaryToothHandler = triggerPri_24X,
  .secondaryToothHandler = triggerSec_24X,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_24X,
  .get_crank_angle = getCrankAngle_24X,
  .set_end_teeth = triggerSetEndTeeth_24X,
};

