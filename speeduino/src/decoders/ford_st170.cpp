#include "ford_st170.h"
#include "missing_tooth.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"
#include "../../crank.h"

/** Ford ST170 - a dedicated decoder for 01-04 Ford Focus ST170/SVT engine.
Standard 36-1 trigger wheel running at crank speed and 8-3 trigger wheel running at cam speed.
* @defgroup dec_ford_st170 Ford ST170 (01-04 Focus)
* @{
*/
void triggerSetup_FordST170(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Set these as we are using the existing missing tooth primary decoder and these will never change.
  configPage4.triggerTeeth = 36;
  configPage4.triggerMissingTeeth = 1;
  configPage4.TrigSpeed = CRANK_SPEED;

  //The number of degrees that passes from tooth to tooth
  triggerToothAngle = 360 / configPage4.triggerTeeth;
  //The number of physical teeth on the wheel.
  //Doing this here saves us a calculation each time in the interrupt
  triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth;
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth));

  //Cam pattern is 8-3, so 2 nearest teeth are 90 deg crank angle apart.
  //Cam can be advanced by 60 deg, so going from fully retarded to fully
  //advanced closes the gap to 30 deg. Zetec cam pulleys aren't keyed from
  //factory, so I subtracted additional 10 deg to avoid filter to be too
  //aggressive. And there you have it 720/20=36.
  triggerSecFilterTime = MICROS_PER_MIN / MAX_RPM / 8U / 2U;

  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  checkSyncToothCount = (36) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  secondaryToothCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * (1U + 1U));
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

void triggerSec_FordST170(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if (toothLastSecToothTime == 0)
  {
    curGap2 = 0;
    toothLastSecToothTime = curTime2;
  }

  if (curGap2 >= triggerSecFilterTime)
  {
    //If the time between the current tooth and the last is greater than 1.5x
    //the time between the last tooth and the tooth before that, we make the
    //assertion that we must be at the first tooth after the gap.
    uint32_t const deltaT = toothLastSecToothTime - toothLastMinusOneSecToothTime;

    targetGap2 = deltaT + (deltaT >> 1);
    toothLastMinusOneSecToothTime = toothLastSecToothTime;
    if (curGap2 >= targetGap2 || secondaryToothCount == 5)
    {
      secondaryToothCount = 1;
      revolutionOne = 1; //Sequential revolution reset
      //This is used to prevent a condition where serious intermittent signals
      //(e.g. someone furiously plugging the sensor wire in and out) can leave
      //the filter in an unrecoverable state
      triggerSecFilterTime = 0;
    }
    else
    {
      //Set filter at 25% of the current speed. Filter can only be recalculated
      //for the regular teeth, not the missing one.
      triggerSecFilterTime = curGap2 >> 2;
      secondaryToothCount++;
    }

    toothLastSecToothTime = curTime2;

    //Record the VVT Angle
    //We use the first tooth after the long gap as our reference, this remains in the same engine
    //cycle even when the VVT is at either end of its full swing.
    if (configPage6.vvtEnabled > 0 && revolutionOne && secondaryToothCount == 1)
    {
      int16_t curAngle = decoder.handler.get_crank_angle();

      while (curAngle > 360)
      {
        curAngle -= 360;
      }
      if (configPage6.vvtMode == VVT_MODE_CLOSED_LOOP)
      {
        curAngle = ANGLE_FILTER(curAngle << 1, configPage4.ANGLEFILTER_VVT, curAngle);
        currentStatus.vvt1Angle = 360 - curAngle - configPage10.vvtCL0DutyAng;
      }
    }
  } //Trigger filter
}

uint16_t getRPM_FordST170(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (toothCurrentCount != 1)
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes
      //the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_FordST170(void)
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
  tempRevolutionOne = revolutionOne;
  tempToothLastToothTime = toothLastToothTime;

  interrupts();

  //Number of teeth that have passed since tooth 1, multiplied by the angle each
  //tooth represents, plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

  //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
  if (tempRevolutionOne && configPage4.TrigSpeed == CRANK_SPEED)
  {
    crankAngle += 360;
  }

  uint32_t const lastCrankAngleCalc = micros();
  uint32_t const elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += crank.timeToAngleDegPerMicroSec(elapsedTime);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  else if (crankAngle > CRANK_ANGLE_MAX)
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
calcSetEndTeeth_FordST170(int ignitionAngle, uint8_t toothAdder)
{
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;
#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divTriggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerToothAngle;
#endif
  tempEndTooth = nudge(1, 36U + toothAdder,  tempEndTooth - 1, 36U + toothAdder);

  return clampToActualTeeth((uint16_t)tempEndTooth, toothAdder);
}

static void calcSetEndTeeth_FordST170_ignition(ignition_context_st &ignition)
{
  byte toothAdder = 0;

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
      && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = 36;
  }

  ignition.endTooth = calcSetEndTeeth_FordST170(ignition.endAngle, toothAdder);
}

void triggerSetEndTeeth_FordST170(void)
{
  calcSetEndTeeth_FordST170_ignition(ignition_contexts[ignChannel1]);
  calcSetEndTeeth_FordST170_ignition(ignition_contexts[ignChannel2]);
  calcSetEndTeeth_FordST170_ignition(ignition_contexts[ignChannel3]);
  calcSetEndTeeth_FordST170_ignition(ignition_contexts[ignChannel4]);
  // Removed ign channels >4 as an ST170 engine is a 4 cylinder
}

static void attach_interrupts(void)
{
  //Ford ST170
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_missingTooth, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_FordST170, secondaryTriggerEdge);
}

decoder_handler_st const trigger_st170 PROGMEM =
{
  .setup = triggerSetup_FordST170,
  .primaryToothHandler = triggerPri_missingTooth,
  .secondaryToothHandler = triggerSec_FordST170,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_FordST170,
  .get_crank_angle = getCrankAngle_FordST170,
  .set_end_teeth = triggerSetEndTeeth_FordST170,
  .attach_interrupts = attach_interrupts,
};

