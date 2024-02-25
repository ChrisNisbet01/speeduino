#include "vmax.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"
#include "globals.h"

static bool triggerEdge;

/** Yamaha Vmax 1990+ with 6 uneven teeth, triggering on the wide lobe.
Within the decoder code, the sync tooth is referred to as tooth #1.*
*Derived from Harley and made to work on the Yamaha Vmax.
Trigger is based on 'CHANGE' so we get a signal on the up and downward edges of*
*the lobe. This is required to identify the wide lobe.
* @defgroup dec_vmax Yamaha Vmax
* @{
*/
void triggerSetup_Vmax(bool const initialisationComplete)
{
  triggerToothAngle = 0; // The number of degrees that passes from tooth to tooth, ev. 0. It alternates uneven
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * 60U);

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on all the time.
    toothLastToothTime = micros();
  }
  triggerFilterTime = 1500;
  // We must start with a valid trigger or we cannot start measuring the lobe width.
  // We only have a false trigger on the lobe up event when it doesn't pass the filter.
  // Then, the lobe width will also not be measured.
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
  toothAngles[1] = 0;      //tooth #1, these are the absolute tooth positions
  toothAngles[2] = 40;     //tooth #2
  toothAngles[3] = 110;    //tooth #3
  toothAngles[4] = 180;    //tooth #4
  toothAngles[5] = 220;    //tooth #5
  toothAngles[6] = 290;    //tooth #6
}

//curGap = microseconds between primary triggers
//curGap2 = microseconds between secondary triggers
//toothCurrentCount = the current number for the end of a lobe
//secondaryToothCount = the current number of the beginning of a lobe
//We measure the width of a lobe so on the end of a lobe, but want to trigger on
//the beginning. Variable toothCurrentCount tracks the downward events, and
//secondaryToothCount updates on the upward events. Ideally, it should be the
//other way round but the engine stall routine resets secondaryToothCount,
//so it would not sync again after an engine stall.

void triggerPri_Vmax(void)
{
  curTime = micros();
  // Forwarded from the config page to setup the primary trigger edge (rising or falling).
  // Inverting VR-conditioners require FALLING, non-inverting VR-conditioners
  // require RISING in the Trigger edge setup.
  if (Trigger.read() == triggerEdge)
  {
    curGap2 = curTime;
    curGap = curTime - toothLastToothTime;
    if (curGap >= triggerFilterTime)
    {
      //Flag this pulse as being a valid trigger (ie that it passed filters)
      BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
      if (toothCurrentCount > 0) // We have sync based on the tooth width.
      {
        if (toothCurrentCount == 1)
        {
          secondaryToothCount = 1;
          // Has to be equal to Angle Routine, and describe the delta between two teeth.
          triggerToothAngle = 70;
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;
          currentStatus.hasSync = true;
          //Angle to this tooth is 70, next is in 40, compensating.
          setFilter((curGap * 4) / 7);
          currentStatus.startRevolutions++; //Counter
        }
        else if (toothCurrentCount == 2)
        {
          secondaryToothCount = 2;
          triggerToothAngle = 40;
          //Angle to this tooth is 40, next is in 70, compensating.
          setFilter((curGap * 7) / 4);
        }
        else if (toothCurrentCount == 3)
        {
          secondaryToothCount = 3;
          triggerToothAngle = 70;
          //Angle to this tooth is 70, next is in 70. No need to compensate.
          setFilter(curGap);
        }
        else if (toothCurrentCount == 4)
        {
          secondaryToothCount = 4;
          triggerToothAngle = 70;
          //Angle to this tooth is 70, next is in 40, compensating.
          setFilter((curGap * 4) / 7);
        }
        else if (toothCurrentCount == 5)
        {
          secondaryToothCount = 5;
          triggerToothAngle = 40;
          //Angle to this tooth is 40, next is in 70, compensating.
          setFilter((curGap * 7) / 4);
        }
        else if (toothCurrentCount == 6)
        {
          secondaryToothCount = 6;
          triggerToothAngle = 70;
          //Angle to this tooth is 70, next is in 70. No need to compensate.
          setFilter(curGap);
        }
        toothLastMinusOneToothTime = toothLastToothTime;
        toothLastToothTime = curTime;
        if (triggerFilterTime > 50000) //The first pulse seen
        {
          triggerFilterTime = 0;
        }
      }
      else
      {
        triggerFilterTime = 0;
        return; //Zero, no sync yet.
      }
    }
    else
    {
      BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being an invalid trigger
    }
  }
  else if (BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER))
  {
    // Inverted due to vr conditioner. So this is the falling lobe. We only
    // process if there was a valid trigger.
    unsigned long curGapLocal = curTime - curGap2;
    // Small lobe is 5 degrees, big lobe is 45 degrees. So this should be the wide lobe.
    if (curGapLocal > lastGap * 2)
    {
      //Wide should be seen with toothCurrentCount = 0, when there is no sync yet,
      //or toothCurrentCount = 6 when we have done a full revolution.
      if (toothCurrentCount == 0 || toothCurrentCount == 6)
      {
        currentStatus.hasSync = true;
      }
      else
      {
        //Wide lobe seen where it shouldn't, adding a sync error.
        currentStatus.syncLossCounter++;
      }
      toothCurrentCount = 1;
    }
    else if (toothCurrentCount == 6)
    {
      //The 6th lobe should be wide, adding a sync error.
      toothCurrentCount = 1;
      currentStatus.syncLossCounter++;
    }
    else
    {
      // Small lobe, just add 1 to the toothCurrentCount.
      toothCurrentCount++;
    }
    lastGap = curGapLocal;
  }
  else
  {
    //Reset this every time to ensure we only filter when needed.
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
  }
}

// Needs to be enabled in main()
void triggerSec_Vmax(void)
{
  return; // No need for now. The only thing it could help to sync more quickly
          //or confirm position.
}

uint16_t getRPM_Vmax(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync)
  {
    if (currentStatus.RPM < (configPage4.crankRPM * 100))
    {
      int tempToothAngle;
      unsigned long toothTime;

      if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
      {
        tempRPM = 0;
      }
      else
      {
        noInterrupts();

        tempToothAngle = triggerToothAngle;
        //The time in uS that one revolution would take at current speed
        //(The time tooth 1 was last seen, minus the time it was seen prior to that)
        SetRevolutionTime(toothOneTime - toothOneMinusOneTime);
        toothTime = (toothLastToothTime - toothLastMinusOneToothTime);

        interrupts();

        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN / 10U)) / toothTime;
      }
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }

  return tempRPM;
}


int getCrankAngle_Vmax(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempsecondaryToothCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempsecondaryToothCount = secondaryToothCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Check if the last tooth seen was the reference tooth (Number 3). All others
  //can be calculated, but tooth 3 has a unique angle
  int crankAngle;
  crankAngle = toothAngles[tempsecondaryToothCount] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
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

static void attach_interrupts(void)
{
  // set as boolean so we can directly use it in decoder.
  triggerEdge = (configPage4.TrigEdge == 0);

  //Hardcoded change, the primaryTriggerEdge will be used in the decoder to
  //select if it`s an inverted or non-inverted signal.
  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_Vmax, CHANGE);
}

decoder_handler_st const trigger_vmax PROGMEM =
{
  .setup = triggerSetup_Vmax,
  .primaryToothHandler = triggerPri_Vmax,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Vmax,
  .get_crank_angle = getCrankAngle_Vmax,
  .set_end_teeth = nullSetEndTeeth,
  .attach_interrupts = attach_interrupts,
};

