#include "420a.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** DSM 420a, For the DSM Eclipse with 16 teeth total on the crank.
* Tracks the falling side of the signal.
* Sync is determined by watching for a falling edge on the secondary signal and
* checking if the primary signal is high then.
* https://github.com/noisymime/speeduino/issues/133
* @defgroup dec_dsm_420a DSM 420a, For the DSM Eclipse
* @{
*/
void triggerSetup_420a(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be discarded as noise
  triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 360UL));
  triggerSecFilterTime = 0;
  //Initially set to 0 prior to calculating the secondary window duration
  secondaryToothCount = 0;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  toothCurrentCount = 1;
  triggerToothAngle = 20; //Is only correct for the 4 short pulses before each TDC
  BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  toothSystemCount = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * 93U);

  toothAngles[0] = 711; //tooth #1, just before #1 TDC
  toothAngles[1] = 111;
  toothAngles[2] = 131;
  toothAngles[3] = 151;
  toothAngles[4] = 171; //Just before #3 TDC
  toothAngles[5] = toothAngles[1] + 180;
  toothAngles[6] = toothAngles[2] + 180;
  toothAngles[7] = toothAngles[3] + 180;
  toothAngles[8] = toothAngles[4] + 180; //Just before #4 TDC
  toothAngles[9] = toothAngles[1] + 360;
  toothAngles[10] = toothAngles[2] + 360;
  toothAngles[11] = toothAngles[3] + 360;
  toothAngles[12] = toothAngles[4] + 360; //Just before #2 TDC
  toothAngles[13] = toothAngles[1] + 540;
  toothAngles[14] = toothAngles[2] + 540;
  toothAngles[15] = toothAngles[3] + 540;
}

void triggerPri_420a(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      curGap = 0;
    }

    if (toothCurrentCount > 16 && currentStatus.hasSync)
    {
      //Means a complete rotation has occurred.
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    triggerFilterTime = 0;

    BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //EXPERIMENTAL!
    if (configPage2.perToothIgn)
    {
      int16_t crankAngle = (toothAngles[toothCurrentCount - 1]) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      checkPerToothTiming(crankAngle, toothCurrentCount);
    }
  }
}

void triggerSec_420a(void)
{
  //Secondary trigger is only on falling edge

  if (Trigger.read())
  {
    //Secondary signal is falling and primary signal is HIGH
    if (!currentStatus.hasSync)
    {
      //If we don't have sync, then assume the signal is good
      toothCurrentCount = 13;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if (toothCurrentCount != 13)
      {
        currentStatus.syncLossCounter++;
        toothCurrentCount = 13;
      }
    }
  }
  else
  {
    //Secondary signal is falling and primary signal is LOW
    if (!currentStatus.hasSync)
    {
      //If we don't have sync, then assume the signal is good
      toothCurrentCount = 5;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if (toothCurrentCount != 5)
      {
        currentStatus.syncLossCounter++;
        toothCurrentCount = 5;
      }
    }
  }
}

uint16_t getRPM_420a(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    //Possibly look at doing special handling for cranking in the future,
    //but for now just use the standard method
    tempRPM = stdGetRPM(CAM_SPEED);
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
  }

  return tempRPM;
}

int getCrankAngle_420a(void)
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

  //Perform a lookup of the fixed toothAngles array to find what the angle of
  //the last tooth passed was.
  crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

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

void triggerSetEndTeeth_420a(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
  ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

  if (currentStatus.advance < 9)
  {
    ignition1.endTooth = 1;
    ignition2.endTooth = 5;
    ignition3.endTooth = 9;
    ignition4.endTooth = 13;
  }
  else
  {
    ignition1.endTooth = 16;
    ignition2.endTooth = 4;
    ignition3.endTooth = 8;
    ignition4.endTooth = 12;
  }
}

static void attach_interrupts(void)
{
  //DSM 420a
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = FALLING; //Always falling edge

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_420a, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_420a, secondaryTriggerEdge);
}

decoder_handler_st const trigger_420a PROGMEM =
{
  .setup = triggerSetup_420a,
  .primaryToothHandler = triggerPri_420a,
  .secondaryToothHandler = triggerSec_420a,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_420a,
  .get_crank_angle = getCrankAngle_420a,
  .set_end_teeth = triggerSetEndTeeth_420a,
  .attach_interrupts = attach_interrupts,
};

