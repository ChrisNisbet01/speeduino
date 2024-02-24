#pragma once

#include "ignition_contexts.h"
#include "bit_macros.h"

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
extern libdivide::libdivide_s16_t divTriggerToothAngle;
#endif

extern volatile unsigned int thirdToothCount;
extern volatile unsigned long triggerThirdFilterTime;

//The current number of teeth
//(Once sync has been achieved, this can never actually be 0)
extern volatile uint16_t toothCurrentCount;

bool UpdateRevolutionTimeFromTeeth(bool isCamTeeth);

/**
On decoders that are enabled for per-tooth based timing adjustments,
this function performs the timer compare changes on the schedules themselves
For each ignition channel, a check is made whether we're at the relevant
tooth and whether that ignition schedule is currently running
Only if both these conditions are met will the schedule be updated with the
latest timing information.
If it's the correct tooth, but the schedule is not yet started, calculate an
end compare value (This situation occurs when both the start and end of the
ignition pulse happen after the end tooth, but before the next tooth)
*/
static inline void checkPerToothTiming(int16_t crankAngle, uint16_t currentTooth)
{
  if (fixedCrankingOverride == 0 && currentStatus.RPM > 0)
  {
    ignitions.adjustCrankAngle(crankAngle, currentTooth);
  }
}

/**
 * Sets the new filter time based on the current settings.
 * This ONLY works for even spaced decoders.
 */
static inline void setFilter(unsigned long curGap)
{
  if (configPage4.triggerFilter == 1)
  {
    //Lite filter level is 25% of previous gap
    triggerFilterTime = curGap >> 2;
  }
  else if (configPage4.triggerFilter == 2)
  {
    //Medium filter level is 50% of previous gap
    triggerFilterTime = curGap >> 1;
  }
  else if (configPage4.triggerFilter == 3)
  {
    //Aggressive filter level is 75% of previous gap
    triggerFilterTime = (curGap * 3) >> 2;
  }
  else //trigger filter is turned off.
  {
    triggerFilterTime = 0;
  }
}

/** @brief At 1 RPM, each degree of angular rotation takes this many microseconds */
#define MICROS_PER_DEG_1_RPM INT32_C(166667)

/** @brief The maximum rpm that the ECU will attempt to run at.
 *
 * It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be
 * allowed to run. Lower number gives better performance
 **/
#define MAX_RPM INT16_C(18000)

/** @brief Absolute minimum RPM that the crank math (& therefore all of Speeduino) can be used with
 *
 * This is dictated by the use of uint16_t as the base type for storing
 * angle<->time conversion factor (degreesPerMicro)
*/
#define MIN_RPM ((MICROS_PER_DEG_1_RPM/(UINT16_MAX/16UL))+1UL)

static inline uint16_t clampToToothCount(int16_t toothNum, uint8_t toothAdder)
{
  int16_t toothRange = (int16_t)configPage4.triggerTeeth + (int16_t)toothAdder;

  return (uint16_t)nudge(1, toothRange, toothNum, toothRange);
}

static inline uint16_t clampToActualTeeth(uint16_t toothNum, uint8_t toothAdder)
{
  if (toothNum > triggerActualTeeth && toothNum <= configPage4.triggerTeeth)
  {
    toothNum = triggerActualTeeth;
  }
  return min(toothNum, (uint16_t)(triggerActualTeeth + toothAdder));
}

static inline uint16_t clampRpm(uint16_t rpm)
{
  return (rpm >= MAX_RPM) ? currentStatus.RPM : rpm;
}

__attribute__((noinline))
bool SetRevolutionTime(uint32_t revTime);

static inline uint16_t RpmFromRevolutionTimeUs(uint32_t revTime)
{
  uint16_t rpm;

  if (revTime < UINT16_MAX)
  {
    rpm = udiv_32_16_closest(MICROS_PER_MIN, revTime);
  }
  else
  {
    //Calc RPM based on last full revolution time (Faster as /)
    rpm = UDIV_ROUND_CLOSEST(MICROS_PER_MIN, revTime, uint32_t);
  }

  return clampRpm(rpm);
}

/** Compute RPM.
* As nearly all the decoders use a common method of determining RPM
* (The time the last full revolution took), a common function is simpler.
* @param isCamTeeth - Indicates that this is a cam wheel tooth.
* Some patterns have a tooth #1 every crank rev, others are every cam rev.
* @return RPM
*/
static __attribute__((noinline)) uint16_t
stdGetRPM(bool isCamTeeth)
{
  if (UpdateRevolutionTimeFromTeeth(isCamTeeth))
  {
    return RpmFromRevolutionTimeUs(revolutionTime);
  }

  return currentStatus.RPM;
}

/**
This is a special case of RPM measure that is based on the time between the last
2 teeth rather than the time of the last full revolution.
This gives much more volatile reading, but is quite useful during cranking,
particularly on low resolution patterns.
It can only be used on patterns where the teeth are evenly spaced.
It takes an argument of the full (COMPLETE) number of teeth per revolution.
For a missing tooth wheel, this is the number if the tooth had NOT been missing (Eg 36-1 = 36)
*/
static __attribute__((noinline))int crankingGetRPM(byte totalTeeth, bool isCamTeeth)
{
  if (currentStatus.startRevolutions >= configPage4.StgCycles
      && (currentStatus.hasSync || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)))
  {
    if (toothLastMinusOneToothTime > 0 && toothLastToothTime > toothLastMinusOneToothTime)
    {
      noInterrupts();

      uint32_t const temp = ((toothLastToothTime - toothLastMinusOneToothTime) * totalTeeth) >> isCamTeeth;

      bool newRevtime = SetRevolutionTime(temp);

      interrupts();

      if (newRevtime)
      {
        return RpmFromRevolutionTimeUs(revolutionTime);
      }
    }
  }

  return currentStatus.RPM;
}

