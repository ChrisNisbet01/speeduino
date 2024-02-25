#include "4g63.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** @} */

/** Mitsubishi 4G63 / NA/NB Miata + MX-5 / 4/2.
Note: raw.githubusercontent.com/noisymime/speeduino/master/reference/wiki/decoders/4g63_trace.png
Tooth #1 is defined as the next crank tooth after the crank signal is HIGH when the cam signal is falling.
Tooth number one is at 355* ATDC.
* @defgroup dec_mitsu_miata Mistsubishi 4G63 and Miata + MX-5
* @{
*/
void triggerSetup_4G63(bool const initialisationComplete)
{
  triggerToothAngle = 180; //The number of degrees that passes from tooth to tooth (primary)
  toothCurrentCount = 99; //Fake tooth count represents no sync
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  MAX_STALL_TIME = 366667UL; //Minimum 50rpm based on the 110 degree tooth spacing

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on
    //all the time.
    toothLastToothTime = micros();
  }

  //Note that these angles are for every rising and falling edge
  if (configPage2.nCylinders == 6)
  {
    //New values below
    toothAngles[0] = 715; //Rising edge of tooth #1
    toothAngles[1] = 45;  //Falling edge of tooth #1
    toothAngles[2] = 115; //Rising edge of tooth #2
    toothAngles[3] = 165; //Falling edge of tooth #2
    toothAngles[4] = 235; //Rising edge of tooth #3
    toothAngles[5] = 285; //Falling edge of tooth #3

    toothAngles[6] = 355; //Rising edge of tooth #4
    toothAngles[7] = 405; //Falling edge of tooth #4
    toothAngles[8] = 475; //Rising edge of tooth #5
    toothAngles[9] = 525; //Falling edge of tooth $5
    toothAngles[10] = 595; //Rising edge of tooth #6
    toothAngles[11] = 645; //Falling edge of tooth #6

    triggerActualTeeth = 12; //Both sides of all teeth over 720 degrees
  }
  else
  {
    // 70 / 110 for 4 cylinder
    toothAngles[0] = 715; //Falling edge of tooth #1
    toothAngles[1] = 105; //Rising edge of tooth #2
    toothAngles[2] = 175; //Falling edge of tooth #2
    toothAngles[3] = 285; //Rising edge of tooth #1

    toothAngles[4] = 355; //Falling edge of tooth #1
    toothAngles[5] = 465; //Rising edge of tooth #2
    toothAngles[6] = 535; //Falling edge of tooth #2
    toothAngles[7] = 645; //Rising edge of tooth #1

    triggerActualTeeth = 8;
  }

  //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerFilterTime = 1500;
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2
  //(for cam speed)
  triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  triggerSecFilterTime_duration = 4000;
  secondaryLastToothTime = 0;
}

void triggerPri_4G63(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap >= triggerFilterTime || currentStatus.startRevolutions == 0)
  {
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
    //This only applies during non-sync conditions. If there is sync then
    //triggerFilterTime gets changed again below with a better value.
    triggerFilterTime = curGap >> 2;

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    toothCurrentCount++;

    //Trigger is on CHANGE, hence 4 pulses = 1 crank rev (or 6 pulses for 6 cylinders)
    if (toothCurrentCount == 1 || toothCurrentCount > triggerActualTeeth)
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    if (currentStatus.hasSync)
    {
      if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)
          && configPage4.ignCranklock
          && (currentStatus.startRevolutions >= configPage4.StgCycles))
      {
        if (configPage2.nCylinders == 4)
        {
          //This operates in forced wasted spark mode during cranking to align
          //with crank teeth
          if (toothCurrentCount == 1 || toothCurrentCount == 5)
          {
            twoCoilsEndCharge(ignition_id_1, ignition_id_3);
          }
          else if (toothCurrentCount == 3 || toothCurrentCount == 7)
          {
            twoCoilsEndCharge(ignition_id_2, ignition_id_4);
          }
        }
        else if (configPage2.nCylinders == 6)
        {
          if (toothCurrentCount == 1 || toothCurrentCount == 7)
          {
            singleCoilEndCharge(ignition_id_1);
          }
          else if (toothCurrentCount == 3 || toothCurrentCount == 9)
          {
            singleCoilEndCharge(ignition_id_2);
          }
          else if (toothCurrentCount == 5 || toothCurrentCount == 11)
          {
            singleCoilEndCharge(ignition_id_3);
          }
        }
      }

      //Whilst this is an uneven tooth pattern, if the specific angle between
      //the last 2 teeth is specified,
      //1st deriv prediction can be used
      if (configPage4.triggerFilter == 1 || currentStatus.RPM < 1400)
      {
        //Lite filter
        bool const current_tooth_is_odd = (toothCurrentCount & 1) != 0;

        if (current_tooth_is_odd && toothCurrentCount < 12)
        {
          if (configPage2.nCylinders == 4)
          { //Trigger filter is set to whatever time it took to do 70 degrees
            //(Next trigger is 110 degrees away)
            triggerToothAngle = 70;
            triggerFilterTime = curGap;
          }
          else if (configPage2.nCylinders == 6)
          {
            //Trigger filter is set to (70/4)=17.5=17 degrees
            //(Next trigger is 50 degrees away).
            triggerToothAngle = 70;
            triggerFilterTime = (curGap >> 2);
          }
        }
        else if (configPage2.nCylinders == 4)
        {
          //Trigger filter is set to (110*3)/8=41.25=41 degrees
          //(Next trigger is 70 degrees away).
          triggerToothAngle = 110;
          triggerFilterTime = (curGap * 3) >> 3;
        }
        else if (configPage2.nCylinders == 6)
        {
          //Trigger filter is set to 25 degrees
          //(Next trigger is 70 degrees away).
          triggerToothAngle = 50;
          triggerFilterTime = curGap >> 1;
        }
        else
        {
          /* Do nothing. */
        }
      }
      else if (configPage4.triggerFilter == 2)
      {
        bool const current_tooth_is_odd = (toothCurrentCount & 1) != 0;

        //Medium filter level
        if (current_tooth_is_odd && toothCurrentCount < 12)
        {
          triggerToothAngle = 70;
          if (configPage2.nCylinders == 4)
          {
            triggerFilterTime = (curGap * 5) >> 2; //87.5 degrees with a target of 110
          }
          else
          {
            triggerFilterTime = curGap >> 1; //35 degrees with a target of 50
          }
        }
        else if (configPage2.nCylinders == 4)
        {
          triggerToothAngle = 110;
          triggerFilterTime = (curGap >> 1); //55 degrees with a target of 70
        }
        else
        {
          triggerToothAngle = 50;
          //Trigger filter is set to (50*3)/4=37.5=37 degrees
          //(Next trigger is 70 degrees away).
          triggerFilterTime = (curGap * 3) >> 2;
        }
      }
      else if (configPage4.triggerFilter == 3)
      {
        bool const current_tooth_is_odd = (toothCurrentCount & 1) != 0;

        //Aggressive filter level
        if (current_tooth_is_odd && toothCurrentCount < 12)
        {
          triggerToothAngle = 70;
          if (configPage2.nCylinders == 4)
          {
            triggerFilterTime = (curGap * 11) >> 3; //96.26 degrees with a target of 110
          }
          else
          {
            triggerFilterTime = curGap >> 1; //35 degrees with a target of 50
          }
        }
        else if (configPage2.nCylinders == 4)
        {
          triggerToothAngle = 110;
          triggerFilterTime = (curGap * 9) >> 5; //61.87 degrees with a target of 70
        }
        else
        {
          triggerToothAngle = 50;
          triggerFilterTime = curGap; //50 degrees with a target of 70
        }
      }
      else
      {
        //trigger filter is turned off.
        triggerFilterTime = 0;
        bool const current_tooth_is_odd = (toothCurrentCount & 1) != 0;

        if (current_tooth_is_odd && toothCurrentCount < 12)
        {
          triggerToothAngle = 70;
        }
        else
        {
          triggerToothAngle = configPage2.nCylinders == 4 ? 110 : 50;
        }
      }

      //EXPERIMENTAL!
      //New ignition mode is ONLY available on 4g63 when the trigger angle is
      //set to the stock value of 0.
      if (configPage2.perToothIgn && configPage4.triggerAngle == 0)
      {
        if (configPage2.nCylinders == 4 && currentStatus.advance > 0)
        {
          int16_t crankAngle = ignitionLimits(toothAngles[toothCurrentCount - 1]);

          //Handle non-sequential tooth counts
          if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL
              && toothCurrentCount > configPage2.nCylinders)
          {
            checkPerToothTiming(crankAngle, (toothCurrentCount - configPage2.nCylinders));
          }
          else
          {
            checkPerToothTiming(crankAngle, toothCurrentCount);
          }
        }
      }
    } //Has sync
    else
    {
      triggerSecFilterTime = 0;
      //New secondary method of determining sync
      if (Trigger.read())
      {
        revolutionOne = Trigger2.read();
      }
      else
      {
        if (!Trigger2.read() && revolutionOne)
        {
          //Crank is low, cam is low and the crank pulse STARTED when the cam was high.
          if (configPage2.nCylinders == 4) //Means we're at 5* BTDC on a 4G63 4 cylinder
          {
            toothCurrentCount = 1;
          }
        }
        //If sequential is ever enabled, the below toothCurrentCount will need to change:
        else if (Trigger2.read() && revolutionOne)
        {
          //Crank is low, cam is high and the crank pulse STARTED when the cam was high.
          if (configPage2.nCylinders == 4) //Means we're at 5* BTDC on a 4G63 4 cylinder
          {
            toothCurrentCount = 5;
          }
          else if (configPage2.nCylinders == 6) //Means we're at 45* ATDC on 6G72 6 cylinder
          {
            toothCurrentCount = 2; currentStatus.hasSync = true;
          }
          else
          {
            /* Do nothing. */
          }
        }
      }
    }
  } //Filter time

}
void triggerSec_4G63(void)
{
  bool trigger_is_high = Trigger.read();
  curTime2 = micros();

  curGap2 = curTime2 - toothLastSecToothTime;
  if (curGap2 >= triggerSecFilterTime)
  {
    toothLastSecToothTime = curTime2;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    triggerSecFilterTime = curGap2 >> 1; //Basic 50% filter for the secondary reading
    //More aggressive options:
    //62.5%:
    //triggerSecFilterTime = (curGap2 * 9) >> 5;
    //75%:
    //triggerSecFilterTime = (curGap2 * 6) >> 3;

    if (!currentStatus.hasSync)
    {
      //If this is removed, can have trouble getting sync again after the engine
      //is turned off (but ECU not reset).
      triggerFilterTime = 1500;
      //Divide the secondary filter time by 2 again, making it 25%.
      //Only needed when cranking
      triggerSecFilterTime = triggerSecFilterTime >> 1;
      if (trigger_is_high)
      {
        if (configPage2.nCylinders == 4)
        {
          if (toothCurrentCount == 8) //Is 8 for sequential, was 4
          {
            currentStatus.hasSync = true;
          }
        }
        else if (configPage2.nCylinders == 6)
        {
          if (toothCurrentCount == 7)
          {
            currentStatus.hasSync = true;
          }
        }
        else
        {
          /* Do nothing. */
        }
      }
      else
      {
        if (configPage2.nCylinders == 4)
        {
          if (toothCurrentCount == 5) //Is 5 for sequential, was 1
          {
            currentStatus.hasSync = true;
          }
        }
        //Cannot gain sync for 6 cylinder here.
      }
    }

    if (currentStatus.RPM < currentStatus.crankRPM || configPage4.useResync == 1)
    {
      if (currentStatus.hasSync && configPage2.nCylinders == 4)
      {
        triggerSecFilterTime_duration = (micros() - secondaryLastToothTime1) >> 1;

        if (trigger_is_high)
        {
          //Whilst we're cranking and have sync, we need to watch for noise pulses.
          if (toothCurrentCount != 8)
          {
            // This should never be true, except when there's noise
            currentStatus.hasSync = false;
            currentStatus.syncLossCounter++;
          }
        }
      } //Has sync and 4 cylinder
    } // Use resync or cranking
  } //Trigger filter
}

uint16_t getRPM_4G63(void)
{
  uint16_t tempRPM = 0;
  //During cranking, RPM is calculated 4 times per revolution, once for each
  //rising/falling of the crank signal.
  //Because these signals aren't even (Alternating 110 and 70 degrees),
  //this needs a special function
  if (currentStatus.hasSync)
  {
    if (currentStatus.RPM < currentStatus.crankRPM)
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
        //Note that trigger tooth angle changes between 70 and 110 depending on
        //the last tooth that was seen (or 70/50 for 6 cylinders)
        toothTime = (toothLastToothTime - toothLastMinusOneToothTime);

        interrupts();

        toothTime *= 36;
        tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN / 10U)) / toothTime;
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
  }

  return tempRPM;
}

int getCrankAngle_4G63(void)
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
      crankAngle += 360;
    }
  }

  return crankAngle;
}

void triggerSetEndTeeth_4G63(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
  ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

  if (configPage2.nCylinders == 4)
  {
    if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
    {
      ignition1.endTooth = 8;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4;
      ignition4.endTooth = 6;
    }
    else
    {
      ignition1.endTooth = 4;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4; //Not used
      ignition4.endTooth = 2;
    }
  }
  else if (configPage2.nCylinders == 6)
  {
    if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
    {
      //This should never happen as 6 cylinder sequential not supported
      ignition1.endTooth = 8;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4;
      ignition4.endTooth = 6;
    }
    else
    {
      ignition1.endTooth = 6;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4;
      ignition4.endTooth = 2; //Not used
    }
  }
}

static void attach_interrupts(void)
{
  byte const primaryTriggerEdge = CHANGE;
  byte const secondaryTriggerEdge = FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_4G63, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_4G63, secondaryTriggerEdge);
}

decoder_handler_st const trigger_4G63 PROGMEM =
{
  .setup = triggerSetup_4G63,
  .primaryToothHandler = triggerPri_4G63,
  .secondaryToothHandler = triggerSec_4G63,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_4G63,
  .get_crank_angle = getCrankAngle_4G63,
  .set_end_teeth = triggerSetEndTeeth_4G63,
  .attach_interrupts = attach_interrupts,
};

