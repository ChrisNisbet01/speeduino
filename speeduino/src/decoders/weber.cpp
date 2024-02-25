#include "weber.h"
#include "dual_wheel.h"
#include "decoders.h"
#include "triggers.h"
#include "bit_macros.h"
#include "crankMaths.h"
#include "null_trigger.h"
#include "ignition_control.h"
#include "auxiliary_pins.h"
#include "utilities.h"

/** Weber-Marelli trigger setup with 2 wheels, 4 teeth 90deg apart on crank and 2 90deg apart on cam.
Uses DualWheel decoders, There can be no missing teeth on the primary wheel.
* @defgroup dec_weber_marelli Weber-Marelli
* @{
*/
void triggerPri_Webber(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    if (checkSyncToothCount > 0)
    {
      checkSyncToothCount++;
    }
    if (triggerSecFilterTime <= curGap) //150% crank tooth
    {
      triggerSecFilterTime = curGap + (curGap >> 1);
    }
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    if (currentStatus.hasSync)
    {
      if (toothCurrentCount == 1 || toothCurrentCount > configPage4.triggerTeeth)
      {
        toothCurrentCount = 1;
        revolutionOne = !revolutionOne; //Flip sequential revolution tracker
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.startRevolutions++; //Counter
      }

      setFilter(curGap); //Recalc the new filter value
    }
    else
    {
      if (secondaryToothCount == 1 && checkSyncToothCount == 4)
      {
        toothCurrentCount = 2;
        currentStatus.hasSync = true;
        revolutionOne = 0; //Sequential revolution reset
      }
    }

    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

      if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
          && revolutionOne
          && configPage4.TrigSpeed == CRANK_SPEED)
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount));
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

void triggerSec_Webber(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  if (curGap2 >= triggerSecFilterTime)
  {
    toothLastSecToothTime = curTime2;

    if (secondaryToothCount == 2 && checkSyncToothCount == 3)
    {
      if (!currentStatus.hasSync)
      {
        toothLastToothTime = micros();
        //Fixes RPM at 10rpm until a full revolution has taken place
        toothLastMinusOneToothTime = micros() - 1500000;
        toothCurrentCount = configPage4.triggerTeeth - 1;

        currentStatus.hasSync = true;
      }
      else
      {
        if (toothCurrentCount != configPage4.triggerTeeth - 1U
            && currentStatus.startRevolutions > 2U)
        {
          //Indicates likely sync loss.
          currentStatus.syncLossCounter++;
        }
        if (configPage4.useResync == 1)
        {
          toothCurrentCount = configPage4.triggerTeeth - 1;
        }
      }
      revolutionOne = 1; //Sequential revolution reset
      triggerSecFilterTime = curGap << 2; //4 crank teeth
      secondaryToothCount = 1; //Next tooth should be first
    } //Running, on first CAM pulse restart crank teeth count, on second the counter should be 3
    else if (!currentStatus.hasSync
             && toothCurrentCount >= 3
             && secondaryToothCount == 0)
    {
      toothLastToothTime = micros();
      //Fixes RPM at 10rpm until a full revolution has taken place
      toothLastMinusOneToothTime = micros() - 1500000;
      toothCurrentCount = 1;
      revolutionOne = 1; //Sequential revolution reset

      currentStatus.hasSync = true;
    } //First start, between gaps on CAM pulses have 2 teeth, sync on first CAM pulse if seen 3 teeth or more
    else
    {
      triggerSecFilterTime = curGap + (curGap >> 1); //150% crank tooth
      secondaryToothCount++;
      checkSyncToothCount = 1; //Tooth 1 considered as already been seen
    } //First time might fall here, second CAM tooth will
  }
  else
  {
    triggerSecFilterTime = curGap + (curGap >> 1); //Noise region, using 150% of crank tooth
    checkSyncToothCount = 1; //Reset tooth counter
  } //Trigger filter
}

static void attach_interrupts(void)
{
  //Weber-Marelli
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_Webber, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_Webber, secondaryTriggerEdge);
}

decoder_handler_st const trigger_weber PROGMEM =
{
  .setup = triggerSetup_DualWheel,
  .primaryToothHandler = triggerPri_Webber,
  .secondaryToothHandler = triggerSec_Webber,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_DualWheel,
  .get_crank_angle = getCrankAngle_DualWheel,
  .set_end_teeth = triggerSetEndTeeth_DualWheel,
  .attach_interrupts = attach_interrupts,
};

