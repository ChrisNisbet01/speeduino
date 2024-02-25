#include "nissan_360.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */

/** Nissan 360 tooth on cam (Optical trigger disc inside distributor housing).
See http://wiki.r31skylineclub.com/index.php/Crank_Angle_Sensor .
* @defgroup dec_nissan360 Nissan 360 tooth on cam
* @{
*/
void triggerSetup_Nissan360(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 360UL);
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2
  //(for cam speed)
  triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  toothCurrentCount = 1;
  triggerToothAngle = 2;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle;
}

void triggerPri_Nissan360(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  toothCurrentCount++; //Increment the tooth counter
  //Flag this pulse as being a valid trigger (ie that it passed filters)
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  toothLastMinusOneToothTime = toothLastToothTime;
  toothLastToothTime = curTime;

  if (currentStatus.hasSync)
  {
    if (toothCurrentCount == 361) //2 complete crank revolutions
    {
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //EXPERIMENTAL!
    if (configPage2.perToothIgn)
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * 2) + configPage4.triggerAngle;
      if (crankAngle > CRANK_ANGLE_MAX_IGN)
      {
        crankAngle -= CRANK_ANGLE_MAX_IGN;
        checkPerToothTiming(crankAngle, toothCurrentCount / 2);
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

void triggerSec_Nissan360(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  toothLastSecToothTime = curTime2;

  //Calculate number of primary teeth that this window has been active for
  byte trigEdge;

  if (configPage4.TrigEdgeSec == 0)
  {
    trigEdge = LOW;
  }
  else
  {
    trigEdge = HIGH;
  }

  //This occurs on the first rotation upon powerup OR the start of a secondary window
  if (secondaryToothCount == 0 || Trigger2.read() == trigEdge)
  {
    secondaryToothCount = toothCurrentCount;
  }
  else
  {
    //If we reach here, we are at the end of a secondary window
    //How many primary teeth have passed during the duration of this secondary window
    byte secondaryDuration = toothCurrentCount - secondaryToothCount;

    if (!currentStatus.hasSync)
    {
      if (configPage2.nCylinders == 4)
      {
        //Supported pattern is where all the inner windows as a different size (Most SR engines)
        //These equate to 4,8,12,16 teeth spacings
        if (secondaryDuration >= 15 && secondaryDuration <= 17)
        {
          //Duration of window = 16 primary teeth
          //End of first window (The longest) occurs 16 teeth after TDC
          toothCurrentCount = 16;
          currentStatus.hasSync = true;
        }
        else if (secondaryDuration >= 11 && secondaryDuration <= 13)
        {
          //Duration of window = 12 primary teeth
          toothCurrentCount = 102; //End of second window is after 90+12 primary teeth
          currentStatus.hasSync = true;
        }
        else if (secondaryDuration >= 7 && secondaryDuration <= 9)
        {
          //Duration of window = 8 primary teeth
          toothCurrentCount = 188; //End of third window is after 90+90+8 primary teeth
          currentStatus.hasSync = true;
        }
        else if (secondaryDuration >= 3 && secondaryDuration <= 5)
        {
          //Duration of window = 4 primary teeth
          toothCurrentCount = 274; //End of fourth window is after 90+90+90+4 primary teeth
          currentStatus.hasSync = true;
        }
        else //This should really never happen
        {
          currentStatus.hasSync = false;
          currentStatus.syncLossCounter++;
        }
      }
      else if (configPage2.nCylinders == 6)
      {
        //Pattern on the 6 cylinders is 4-8-12-16-20-24
        if (secondaryDuration >= 3 && secondaryDuration <= 5)
        {
          //Duration of window = 4 primary teeth
          toothCurrentCount = 124; //End of smallest window is after 60+60+4 primary teeth
          currentStatus.hasSync = true;
        }
      }
      else if (configPage2.nCylinders == 8)
      {
        //V8 Optispark
        //Pattern on the 8 cylinders is the same as the 6 cylinder 4-8-12-16-20-24
        if (secondaryDuration >= 6 && secondaryDuration <= 8)
        {
          //Duration of window = 16 primary teeth
          toothCurrentCount = 56; //End of the shortest of the individual windows. Occurs at 102 crank degrees.
          currentStatus.hasSync = true;
        }
      }
      else //This should really never happen (Only 4, 6 and 8 cylinder engines for this pattern)
      {
        currentStatus.hasSync = false;
      }
    }
    else
    {
      if (configPage4.useResync)
      {
        //Already have sync, but do a verify every 720 degrees.
        if (configPage2.nCylinders == 4)
        {
          if (secondaryDuration >= 15 && secondaryDuration <= 17)
          {
            //Duration of window = 16 primary teeth
            toothCurrentCount = 16; //End of first window (The longest) occurs 16 teeth after TDC
          }
        }
        else if (configPage2.nCylinders == 6)
        {
          if (secondaryDuration == 4)
          {
            /* Do nothing. */
          }
        } //Cylinder count
      } //use resync
    } //Has sync
  } //First getting sync or not
}

uint16_t getRPM_Nissan360(void)
{
  //Can't use stdGetRPM as there is no separate cranking RPM calc
  //(stdGetRPM returns 0 if cranking)
  uint16_t tempRPM;

  if (currentStatus.hasSync && toothLastToothTime != 0 && toothLastMinusOneToothTime != 0)
  {
    if (currentStatus.startRevolutions < 2)
    {
      noInterrupts();

      //Each tooth covers 2 crank degrees, so multiply by 180 to get a full revolution time.
      SetRevolutionTime((toothLastToothTime - toothLastMinusOneToothTime) * 180);

      interrupts();
    }
    else
    {
      noInterrupts();

      //The time in uS that one revolution would take at current speed
      //(The time tooth 1 was last seen, minus the time it was seen prior to that)
      SetRevolutionTime((toothOneTime - toothOneMinusOneTime) >> 1);

      interrupts();
    }
    //Calc RPM based on last full revolution time (Faster as /)
    tempRPM = RpmFromRevolutionTimeUs(revolutionTime);
    //Set the stall time to be twice the current RPM. This is a safe figure as
    //there should be no single revolution where this changes more than this
    MAX_STALL_TIME = revolutionTime << 1;
  }
  else
  {
    tempRPM = 0;
  }

  return tempRPM;
}

int getCrankAngle_Nissan360(void)
{
  //As each tooth represents 2 crank degrees, we only need to determine whether
  //we're more or less than halfway between teeth to know whether to add another 1 degrees
  int crankAngle = 0;
  int tempToothLastToothTime;
  int tempToothLastMinusOneToothTime;
  int tempToothCurrentCount;

  noInterrupts();

  tempToothLastToothTime = toothLastToothTime;
  tempToothLastMinusOneToothTime = toothLastMinusOneToothTime;
  tempToothCurrentCount = toothCurrentCount;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  crankAngle = ((tempToothCurrentCount - 1) * 2) + configPage4.triggerAngle;
  unsigned long halfTooth = (tempToothLastToothTime - tempToothLastMinusOneToothTime) / 2;
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
  if (elapsedTime > halfTooth)
  {
    //Means we're over halfway to the next tooth, so add on 1 degree
    crankAngle += 1;
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

static void triggerSetEndTeeth_Nissan360_ignition(ignition_context_st &ignition)
{
  //This uses 4 prior teeth, just to ensure there is sufficient time to set the schedule etc
  byte const offset_teeth = 4;

  if (ignition.endAngle - offset_teeth > configPage4.triggerAngle)
  {
    ignition.endTooth = ((ignition.endAngle - configPage4.triggerAngle) / 2) - offset_teeth;
  }
  else
  {
    ignition.endTooth = ((ignition.endAngle + 720 - configPage4.triggerAngle) / 2) - offset_teeth;
  }
}

void triggerSetEndTeeth_Nissan360(void)
{
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel1));
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel2));
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel3));
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel4));
}

decoder_handler_st const trigger_nissan_360 =
{
  .setup = triggerSetup_Nissan360,
  .primaryToothHandler = triggerPri_Nissan360,
  .secondaryToothHandler = triggerSec_Nissan360,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_Nissan360,
  .get_crank_angle = getCrankAngle_Nissan360,
  .set_end_teeth = triggerSetEndTeeth_Nissan360,
};

