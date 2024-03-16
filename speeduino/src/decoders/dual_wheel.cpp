#include "dual_wheel.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../utilities.h"
#include "../../auxiliary_pins.h"
#include "../../crank.h"

/** @} */

/** Dual wheels - 2 wheels located either both on the crank or with the primary
*   on the crank and the secondary on the cam.
Note: There can be no missing teeth on the primary wheel.
* @defgroup dec_dual Dual wheels
* @{
*/
/** Dual Wheel Setup.
 *
 * */
void triggerSetup_DualWheel(bool initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth
  triggerToothAngle = 360 / configPage4.triggerTeeth;
  if (configPage4.TrigSpeed == CAM_SPEED) //Account for cam speed
  {
    triggerToothAngle = 720 / configPage4.triggerTeeth;
  }
  toothCurrentCount = 255; //Default value
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth);
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
   //This is always true for this pattern
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);

  unsigned const minimum_rpm = 50;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle;

#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

/** Dual Wheel Primary.
 *
 * */
void triggerPri_DualWheel(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    if (currentStatus.hasSync)
    {
      if (toothCurrentCount == 1 || toothCurrentCount > configPage4.triggerTeeth)
      {
        toothCurrentCount = 1;
        //Flip sequential revolution tracker
        revolutionOne = !revolutionOne;
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        //Add an extra revolution count if we're running at cam speed
        currentStatus.startRevolutions += 1 + (configPage4.TrigSpeed == CAM_SPEED);
      }

      setFilter(curGap); //Recalc the new filter value
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
        checkPerToothTiming(crankAngle, configPage4.triggerTeeth + toothCurrentCount);
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  } //Trigger filter
}

/** Dual Wheel Secondary.
 *
 * */
void triggerSec_DualWheel(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  if (curGap2 >= triggerSecFilterTime)
  {
    toothLastSecToothTime = curTime2;
    triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed

    if (!currentStatus.hasSync
        || currentStatus.startRevolutions <= configPage4.StgCycles)
    {
      toothLastToothTime = micros();
      //Fixes RPM at 10rpm until a full revolution has taken place

      unsigned const fixed_rpm = 10;

      toothLastMinusOneToothTime =
        micros() - ((MICROS_PER_MIN / fixed_rpm) / configPage4.triggerTeeth);
      toothCurrentCount = configPage4.triggerTeeth;
      //Need to turn the filter off here otherwise the first primary tooth after
      //achieving sync is ignored
      triggerFilterTime = 0;

      currentStatus.hasSync = true;
    }
    else
    {
      if (toothCurrentCount != configPage4.triggerTeeth
          && currentStatus.startRevolutions > 2) //Indicates likely sync loss.
      {
        currentStatus.syncLossCounter++;
      }
      if (configPage4.useResync == 1)
      {
        toothCurrentCount = configPage4.triggerTeeth;
      }
    }

    revolutionOne = true; //Sequential revolution reset
  }
  else
  {
    //Set filter at 25% of the current cam speed. This needs to be performed
    //here to prevent a situation where the RPM and triggerSecFilterTime get out
    //of alignment and curGap2 never exceeds the filter value
    triggerSecFilterTime = crank.revolutionTime >> 1;
  } //Trigger filter
}
/** Dual Wheel - Get RPM.
 *
 * */
uint16_t getRPM_DualWheel(void)
{
  if (currentStatus.hasSync)
  {
    //Account for cam speed
    if (currentStatus.RPM < currentStatus.crankRPM)
    {
      return crankingGetRPM(configPage4.triggerTeeth, configPage4.TrigSpeed == CAM_SPEED);
    }
    else
    {
      return stdGetRPM(configPage4.TrigSpeed == CAM_SPEED);
    }
  }
  return 0U;
}

/** Dual Wheel - Get Crank angle.
 *
 * */
int getCrankAngle_DualWheel(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  uint32_t tempToothLastToothTime;
  int tempToothCurrentCount;
  bool tempRevolutionOne;
  //Grab some variables that are used in the trigger code and assign them to temp variables.
  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  tempRevolutionOne = revolutionOne;
  uint32_t const lastCrankAngleCalc = micros();

  interrupts();

  //Handle case where the secondary tooth was the last one seen
  if (tempToothCurrentCount == 0)
  {
    tempToothCurrentCount = configPage4.triggerTeeth;
  }

  //Number of teeth that have passed since tooth 1, multiplied by the angle each
  //tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy
  //only to the nearest tooth.
  int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
  uint32_t const elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;

  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
  if (tempRevolutionOne && configPage4.TrigSpeed == CRANK_SPEED)
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
    crankAngle += CRANK_ANGLE_MAX;
  }

  return crankAngle;
}

static uint16_t __attribute__((noinline))
calcEndTeeth_DualWheel(int ignitionAngle, uint8_t toothAdder)
{
  int16_t tempEndTooth =
#ifdef USE_LIBDIVIDE
    libdivide::libdivide_s16_do(ignitionAngle - configPage4.triggerAngle, &divTriggerToothAngle);
#else
    (ignitionAngle - (int16_t)configPage4.triggerAngle) / (int16_t)triggerToothAngle;
#endif
  return clampToToothCount(tempEndTooth, toothAdder);
}

/** Dual Wheel - Set End Teeth.
 *
 * */
void triggerSetEndTeeth_DualWheel(void)
{
  //The toothAdder variable is used for when a setup is running sequentially, but the primary wheel is running at crank speed. This way the count of teeth will go up to 2* the number of primary teeth to allow for a sequential count.
  byte toothAdder = 0;
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = configPage4.triggerTeeth;
  }

  ignition_contexts[ignChannel1].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel1].endAngle, toothAdder);
  ignition_contexts[ignChannel2].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel2].endAngle, toothAdder);
  ignition_contexts[ignChannel3].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel3].endAngle, toothAdder);
  ignition_contexts[ignChannel4].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel4].endAngle, toothAdder);
#if IGN_CHANNELS >= 5
  ignition_contexts[ignChannel5].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel5].endAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 6
  ignition_contexts[ignChannel6].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel6].endAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 7
  ignition_contexts[ignChannel7].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel7].endAngle, toothAdder);
#endif
#if IGN_CHANNELS >= 8
  ignition_contexts[ignChannel8].endTooth =
    calcEndTeeth_DualWheel(ignition_contexts[ignChannel8].endAngle, toothAdder);
#endif
}

static void attach_interrupts(void)
{
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_DualWheel, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_DualWheel, secondaryTriggerEdge);
}

decoder_handler_st const trigger_dual_wheel PROGMEM =
{
  .setup = triggerSetup_DualWheel,
  .primaryToothHandler = triggerPri_DualWheel,
  .secondaryToothHandler = triggerSec_DualWheel,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_DualWheel,
  .get_crank_angle = getCrankAngle_DualWheel,
  .set_end_teeth = triggerSetEndTeeth_DualWheel,
  .attach_interrupts = attach_interrupts,
};

