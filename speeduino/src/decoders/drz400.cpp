#include "drz400.h"
#include "dual_wheel.h"
#include "decoders.h"
#include "triggers.h"
#include "../../bit_macros.h"
#include "../../crankMaths.h"
#include "null_trigger.h"
#include "../../ignition_control.h"
#include "../../auxiliary_pins.h"
#include "../../utilities.h"
#include "../../crank.h"

void triggerSetup_DRZ400(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth
  triggerToothAngle = 360 / configPage4.triggerTeeth;
  if (configPage4.TrigSpeed == 1) //Account for cam speed
  {
    triggerToothAngle = 720 / configPage4.triggerTeeth;
  }
  toothCurrentCount = 255; //Default value
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth));
  //Same as above, but fixed at 2 teeth on the secondary input
  triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U));
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //This is always true for this pattern
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
}

void triggerSec_DRZ400(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  if (curGap2 >= triggerSecFilterTime)
  {
    toothLastSecToothTime = curTime2;

    if (!currentStatus.hasSync)
    {
      toothLastToothTime = micros();
      //Fixes RPM at 10rpm until a full revolution has taken place
      toothLastMinusOneToothTime = micros() - ((MICROS_PER_MIN / 10U) / configPage4.triggerTeeth);
      toothCurrentCount = configPage4.triggerTeeth;
      currentStatus.syncLossCounter++;
      currentStatus.hasSync = true;
    }
    else
    {
      // have rotation, set tooth to six so next tooth is 1 & dual wheel
      // rotation code kicks in
      toothCurrentCount = 6;
    }
  }

  triggerSecFilterTime = (toothOneTime - toothOneMinusOneTime) >> 1; //Set filter at 50% of the current crank speed.
}

static void attach_interrupts(void)
{
  primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
  secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

  attachInterrupt(digitalPinToInterrupt(Trigger.pin), triggerPri_DualWheel, primaryTriggerEdge);
  attachInterrupt(digitalPinToInterrupt(Trigger2.pin), triggerSec_DRZ400, secondaryTriggerEdge);
}

decoder_handler_st const trigger_drz400 PROGMEM =
{
  .setup = triggerSetup_DRZ400,
  .primaryToothHandler = triggerPri_DualWheel,
  .secondaryToothHandler = triggerSec_DRZ400,
  .tertiaryToothHandler = nullTriggerHandler,
  .get_rpm = getRPM_DualWheel,
  .get_crank_angle = getCrankAngle_DualWheel,
  .set_end_teeth = triggerSetEndTeeth_DualWheel,
  .attach_interrupts = attach_interrupts,
};

