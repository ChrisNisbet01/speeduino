#include "mazda_au.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */

/** Mazda AU version.
Tooth #2 is defined as the next crank tooth after the single cam tooth.
Tooth number one is at 348* ATDC.
* @defgroup mazda_au Mazda AU
* @{
*/
void triggerSetup_MazdaAU(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth (primary).
  //This is the maximum gap
  triggerToothAngle = 108;
  toothCurrentCount = 99; //Fake tooth count represents no sync
  secondaryToothCount = 0; //Needed for the cam tooth tracking
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);

  toothAngles[0] = 348; //tooth #1
  toothAngles[1] = 96; //tooth #2
  toothAngles[2] = 168; //tooth #3
  toothAngles[3] = 276; //tooth #4

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
  //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerFilterTime = 1500;
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2
  //(for cam speed)
  triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  BIT_SET(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_MazdaAU(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap >= triggerFilterTime)
  {
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothCurrentCount++;
    //Trigger is on CHANGE, hence 4 pulses = 1 crank rev
    if (toothCurrentCount == 1 || toothCurrentCount == 5)
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.hasSync = true;
      currentStatus.startRevolutions++; //Counter
    }

    if (currentStatus.hasSync)
    {
      // Locked cranking timing is available, fixed at 12* BTDC
      if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
      {
        if (toothCurrentCount == 1)
        {
          singleCoilEndCharge(ignition_id_1);
        }
        else if (toothCurrentCount == 3)
        {
          singleCoilEndCharge(ignition_id_2);
        }
      }

      //Whilst this is an uneven tooth pattern, if the specific angle between
      //the last 2 teeth is specified, 1st deriv prediction can be used
      //Trigger filter is set to whatever time it took to do 72 degrees
      //(Next trigger is 108 degrees away)
      if (toothCurrentCount == 1 || toothCurrentCount == 3)
      {
        triggerToothAngle = 72;
        triggerFilterTime = curGap;
      }
      else
      {
        //Trigger filter is set to (108*3)/8=40 degrees
        //(Next trigger is 70 degrees away).
        triggerToothAngle = 108;
        triggerFilterTime = (curGap * 3) >> 3;
      }

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
    } //Has sync
  } //Filter time
}

void triggerSec_MazdaAU(void)
{
  curTime2 = micros();
  lastGap = curGap2;
  curGap2 = curTime2 - toothLastSecToothTime;
  toothLastSecToothTime = curTime2;

  if (!currentStatus.hasSync)
  {
    //we find sync by looking for the 2 teeth that are close together.
    //The next crank tooth after that is the one we're looking for.
    //For the sake of this decoder, the lone cam tooth will be designated #1
    if (secondaryToothCount == 2)
    {
      toothCurrentCount = 1;
      currentStatus.hasSync = true;
    }
    else
    {
      triggerFilterTime = 1500; //In case the engine has been running and then lost sync.
      targetGap = (lastGap) >> 1; //The target gap is set at half the last tooth gap
      //If the gap between this tooth and the last one is less than half of the
      //previous gap, then we are very likely at the extra (3rd) tooth on the cam).
      //This tooth is located at 421 crank degrees (aka 61 degrees) and therefore
      //the last crank tooth seen was number 1 (At 350 degrees)
      if (curGap2 < targetGap)
      {
        secondaryToothCount = 2;
      }
    }
    secondaryToothCount++;
  }
}


uint16_t getRPM_MazdaAU(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync)
  {
    //During cranking, RPM is calculated 4 times per revolution, once for each tooth on the crank signal.
    //Because these signals aren't even (Alternating 108 and 72 degrees), this needs a special function
    if (currentStatus.RPM < currentStatus.crankRPM)
    {
      int tempToothAngle;

      noInterrupts();

      tempToothAngle = triggerToothAngle;
      //Note that trigger tooth angle changes between 72 and 108 depending on
      //the last tooth that was seen
      SetRevolutionTime(36 * (toothLastToothTime - toothLastMinusOneToothTime));

      interrupts();

      tempRPM = (tempToothAngle * MICROS_PER_MIN) / revolutionTime;
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}

int getCrankAngle_MazdaAU(void)
{
  int crankAngle = 0;
  if (currentStatus.hasSync)
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
  }

  return crankAngle;
}

static void attach_interrupts(void)
{
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_MazdaAU, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_MazdaAU, secondaryTriggerEdge);
}

decoder_handler_st const trigger_mazda_au PROGMEM =
{
  .setup = triggerSetup_MazdaAU,
  .primaryToothHandler = triggerPri_MazdaAU,
  .secondaryToothHandler = triggerSec_MazdaAU,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_MazdaAU,
  .get_crank_angle = getCrankAngle_MazdaAU,
  .set_end_teeth = nullSetEndTeeth,
  .attach_interrupts = attach_interrupts,
};

