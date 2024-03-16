#include "globals.h"
#include "crankMaths.h"
#include "src/decoders/decoders.h"
#include "bit_macros.h"
#include "crank.h"

#define SECOND_DERIV_ENABLED 0

//These are only part of the experimental 2nd deriv calcs
#if SECOND_DERIV_ENABLED!=0
byte deltaToothCount = 0; //The last tooth that was used with the deltaV calc
int rpmDelta;
distance = v0.t + 1/2.a(0).t^2 + 1/6.a(t).t^3
#endif

uint32_t angleToTimeIntervalTooth(uint16_t angle)
{
  uint32_t time_us;

  noInterrupts();

  if(BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
  {
    uint32_t const toothTime = toothLastToothTime - toothLastMinusOneToothTime;
    // triggerToothAngle is set by interrupts
    uint16_t const tempTriggerToothAngle = triggerToothAngle;

    interrupts();

    time_us = (toothTime * (uint32_t)angle) / tempTriggerToothAngle;
  }
  else
  {
    //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
    interrupts();

    time_us = crank.angleToTimeMicroSecPerDegree(angle);
  }

  return time_us;
}

uint16_t timeToAngleIntervalTooth(uint32_t time)
{
  uint16_t angle;

  noInterrupts();

  //Still uses a last interval method (ie retrospective), but bases the interval
  //on the gap between the 2 most recent teeth rather than the last full revolution
  if (BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
  {
    uint32_t const toothTime = toothLastToothTime - toothLastMinusOneToothTime;
    // triggerToothAngle is set by interrupts
    uint16_t const tempTriggerToothAngle = triggerToothAngle;

    interrupts();

    angle = (time * tempTriggerToothAngle) / toothTime;
  }
  else
  {
    interrupts();

    //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
    angle = crank.timeToAngleDegPerMicroSec(time);
  }

  return angle;
}

#if SECOND_DERIV_ENABLED != 0
void doCrankSpeedCalcs(void)
{
     //********************************************************
      //How fast are we going? Need to know how long (uS) it will take to get from one tooth to the next.
      //We then use that to estimate how far we are between the last tooth and the next one
      //We use a 1st Deriv acceleration prediction, but only when there is an even spacing between primary sensor teeth
      //Any decoder that has uneven spacing has its triggerToothAngle set to 0
      //THIS IS CURRENTLY DISABLED FOR ALL DECODERS! It needs more work.
      if (BIT_CHECK(decoderState, BIT_DECODER_2ND_DERIV)
          && toothHistoryIndex >= 3 //toothHistoryIndex must be greater than or equal to 3 as we need the last 3 entries.
          && currentStatus.RPM < 2000) // Currently this mode only runs below 2000 rpm
      {
        //Only recalculate deltaV if the tooth has changed since last time (DeltaV stays the same until the next tooth)
        //if (deltaToothCount != toothCurrentCount)
        {
          deltaToothCount = toothCurrentCount;
          int angle1, angle2; //These represent the crank angles that are travelled for the last 2 pulses
          if(configPage4.TrigPattern == 4)
          {
            //Special case for 70/110 pattern on 4g63
            angle2 = triggerToothAngle; //Angle 2 is the most recent
            angle1 = angle2 == 70 ? 110 : 70;
          }
          else if(configPage4.TrigPattern == 0)
          {
            //Special case for missing tooth decoder where the missing tooth was one of the last 2 seen
            if (toothCurrentCount == 1)
            {
              angle2 = 2*triggerToothAngle;
              angle1 = triggerToothAngle;
            }
            else if (toothCurrentCount == 2)
            {
              angle1 = 2*triggerToothAngle;
              angle2 = triggerToothAngle;
            }
            else
            {
              angle1 = triggerToothAngle;
              angle2 = triggerToothAngle;
            }
          }
          else
          {
            angle1 = triggerToothAngle;
            angle2 = triggerToothAngle;
          }

          uint32_t toothDeltaV =
            (MICROS_PER_SEC * angle2 / toothHistory[toothHistoryIndex])
            - (MICROS_PER_SEC * angle1 / toothHistory[toothHistoryIndex-1]);
          uint32_t toothDeltaT = toothHistory[toothHistoryIndex];
          //long timeToLastTooth = micros() - toothLastToothTime;

          rpmDelta = (toothDeltaV << 10) / (6 * toothDeltaT);
        }
          //This gives accuracy down to 0.1 of a degree and can provide
          //noticeably better timing results on low resolution triggers
          timePerDegreex16 = ldiv( 2666656L, currentStatus.RPM + rpmDelta).quot;
      }
}
#endif