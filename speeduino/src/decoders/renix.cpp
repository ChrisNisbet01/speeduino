#include "renix.h"
#include "missing_tooth.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"
#include "../../globals.h"

/** Renix 44-2-2  and 66-2-2-2 decoder.
* Renix trigger wheel doesn't decode into 360 degrees nicely (360/44 = 8.18 degrees or 360/66 = 5.454545).
* Speeduino can't handle any teeth that have a decimal point.
* Solution is to count teeth, every 11 teeth = a proper angle.
* For 66 tooth decoder its 60 degrees per 11 teeth,
* for 44 tooth decoder it's 90 degrees per 11 teeth.
* This means the system sees 4 teeth on the 44 tooth wheel and 6 teeth on the 66 tooth wheel.
* Double missing tooth in the pattern is actually a large tooth and a large gap.
* If the trigger is set to rising you'll see the start of the large tooth
* then the gap. If its not set to rising the code won't work due to seeing two gaps
*
*
* @defgroup dec_renix Renix decoder
* @{
*/
void triggerSetup_Renix(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  if (configPage2.nCylinders == 4)
  {
    //The number of degrees that passes from tooth to tooth (primary) this
    //changes between 41 and 49 degrees
    triggerToothAngle = 90;
    // wheel has 44 teeth but we use these to work out which tooth angle to use,
    // therefore speeduino thinks we only have 8 teeth.
    configPage4.triggerTeeth = 4;
    configPage4.triggerMissingTeeth = 0;
    //The number of teeth we're pretending physically existing on the wheel.
    triggerActualTeeth = 4;
    //Trigger filter time is the shortest possible time (in uS) that there can
    //be between crank teeth (ie at max RPM). Any pulses that occur faster than this
    //time will be disgarded as noise
    triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 44U);
  }
  else if (configPage2.nCylinders == 6)
  {
    triggerToothAngle = 60;
    // wheel has 44 teeth but we use these to work out which tooth angle to use,
    // therefore speeduino thinks we only have 6 teeth.
    configPage4.triggerTeeth = 6;
    configPage4.triggerMissingTeeth = 0;
    //The number of teeth we're pretending physically existing on the wheel.
    triggerActualTeeth = 6;
    //Trigger filter time is the shortest possible time (in uS) that there can
    //be between crank teeth (ie at max RPM).
    //Any pulses that occur faster than this time will be disgarded as noise
    triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 66U);
  }

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm).
  //Largest gap between teeth is 90 or 60 degrees depending on decoder.
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle;
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);

  toothSystemCount = 1;
  toothCurrentCount = 1;
  toothLastToothTime = 0;
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

// variables used to help calculate gap on the physical 44 or 66 teeth we're
// pretending don't exist in most of the speeduino code
// reusing existing variables to save storage space as these aren't used in the
// code for their original purpose.
#define renixSystemLastToothTime         toothLastToothRisingTime
#define renixSystemLastMinusOneToothTime toothLastSecToothRisingTime

void triggerPri_Renix(void)
{
  curTime = micros();
  curGap = curTime - renixSystemLastToothTime;

  if (curGap >= triggerFilterTime)
  {
    toothSystemCount++;

    if (renixSystemLastToothTime != 0 && renixSystemLastMinusOneToothTime != 0)
    {
      targetGap = 2 * (renixSystemLastToothTime - renixSystemLastMinusOneToothTime);
    }  // in real world the physical 2 tooth gap is bigger than 2 teeth - more like 2.5
    else
    {
      // random large number to stop system thinking we have a gap for the first
      // few teeth on start up
      targetGap = 100000000L;
    }

    if (curGap >= targetGap)
    {
      /* add two teeth to account for the gap we've just seen */
      toothSystemCount++;
      toothSystemCount++;

      if (toothSystemCount != 12)
      {
        // if not 12 (the first tooth after the gap) then we've lost sync
        // lost sync
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        toothSystemCount = 1; // first tooth after gap is always 1
        toothCurrentCount = 1; // Reset as we've lost sync
      }
    }
    else
    {
      //Recalc the new filter value, only do this on the single gap tooth
      setFilter(curGap);
    }
    // needed for target gap calculation
    renixSystemLastMinusOneToothTime = renixSystemLastToothTime;
    renixSystemLastToothTime = curTime;

    if (toothSystemCount == 12  || toothLastToothTime == 0)
    {
      // toothLastToothTime used to ensure we set the value so the code that
      // handles the fuel pump in speeduino.ini has a value to use once the engine is running.
      toothCurrentCount++;

      // 6 Pretend teeth on the 66 tooth wheel, if get to severn rotate round back to first tooth
      // 4 Pretend teeth on the 44 tooth wheel, if get to five rotate round back to first tooth
      if ((configPage2.nCylinders == 6 && toothCurrentCount == 7)
          || (configPage2.nCylinders == 4 && toothCurrentCount == 5))
      {
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.hasSync = true;
        currentStatus.startRevolutions++; //Counter
        revolutionOne = !revolutionOne;
        toothCurrentCount = 1;
      }

      toothSystemCount = 1;
      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;


      //NEW IGNITION MODE
      if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
      {
        int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

        crankAngle = ignitionLimits(crankAngle);
        if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
            && revolutionOne
            && configPage4.TrigSpeed == CRANK_SPEED)
        {
          crankAngle += 360;
          checkPerToothTiming(crankAngle, configPage4.triggerTeeth + toothCurrentCount);
        }
        else
        {
          checkPerToothTiming(crankAngle, toothCurrentCount);
        }
      }
    }
  }
}

static uint16_t __attribute__((noinline))
calcEndTeeth_Renix(int ignitionAngle, uint8_t toothAdder)
{
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;

#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divTriggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerToothAngle;
#endif
  tempEndTooth = tempEndTooth - 1;
  // Clamp to tooth count

  return clampToActualTeeth(clampToToothCount(tempEndTooth, toothAdder), toothAdder);
}

static void calcEndTeeth_Renix_ignition(ignition_context_st &ignition)
{
  byte toothAdder = 0;

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = configPage4.triggerTeeth;
  }

  ignition.endTooth = calcEndTeeth_Renix(ignition.endAngle, toothAdder);
}

void triggerSetEndTeeth_Renix(void)
{
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel1));
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel2));
  currentStatus.canin[1] = ignitions.ignition(ignChannel2).endTooth;
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel3));
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel4));
#if IGN_CHANNELS >= 5
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel5));
#endif
#if IGN_CHANNELS >= 6
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel6));
#endif
#if IGN_CHANNELS >= 7
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel7));
#endif
#if IGN_CHANNELS >= 8
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel8));
#endif
}

static void attach_interrupts(void)
{
  //Renault 44 tooth decoder
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_Renix, primaryTriggerEdge);
}

decoder_handler_st const trigger_renix PROGMEM =
{
  .setup = triggerSetup_Renix,
  .primaryToothHandler = triggerPri_Renix,
  .secondaryToothHandler = nullTriggerHandler,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_missingTooth,
  .get_crank_angle = getCrankAngle_missingTooth,
  .set_end_teeth = triggerSetEndTeeth_Renix,
  .attach_interrupts = attach_interrupts,
};

