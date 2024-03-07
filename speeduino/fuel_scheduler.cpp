/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the project's root directory
*/
/** @file
 * Injector and Ignition (on/off) scheduling (functions).
 * There is usually 8 functions for cylinders 1-8 with same naming pattern.
 *
 * ## Scheduling structures
 *
 * Structures @ref FuelSchedule and @ref Schedule describe (from scheduler.h) describe the scheduling info for Fuel and Ignition respectively.
 * They contain duration, current activity status, start timing, end timing, callbacks to carry out action, etc.
 *
 * ## Scheduling Functions
 *
 * For Injection:
 * - setFuelSchedule*(tout,dur) - **Setup** schedule for (next) injection on the channel
 * - inj*StartFunction() - Execute **start** of injection (Interrupt handler)
 * - inj*EndFunction() - Execute **end** of injection (interrupt handler)
 *
 * For Ignition (has more complex schedule setup):
 * - setIgnitionSchedule*(cb_st,tout,dur,cb_end) - **Setup** schedule for (next) ignition on the channel
 * - ign*StartFunction() - Execute **start** of ignition (Interrupt handler)
 * - ign*EndFunction() - Execute **end** of ignition (Interrupt handler)
 */
#include "fuel_scheduler.h"
#include "globals.h"
#include "timers.h"

FuelSchedule fuelSchedules[injChannelCount] =
{
  FuelSchedule(FUEL1_COUNTER, FUEL1_COMPARE, FUEL1_TIMER_DISABLE, FUEL1_TIMER_ENABLE),
  FuelSchedule(FUEL2_COUNTER, FUEL2_COMPARE, FUEL2_TIMER_DISABLE, FUEL2_TIMER_ENABLE),
  FuelSchedule(FUEL3_COUNTER, FUEL3_COMPARE, FUEL3_TIMER_DISABLE, FUEL3_TIMER_ENABLE),
  FuelSchedule(FUEL4_COUNTER, FUEL4_COMPARE, FUEL4_TIMER_DISABLE, FUEL4_TIMER_ENABLE),
#if (INJ_CHANNELS >= 5)
  FuelSchedule(FUEL5_COUNTER, FUEL5_COMPARE, FUEL5_TIMER_DISABLE, FUEL5_TIMER_ENABLE),
#endif
#if (INJ_CHANNELS >= 6)
  FuelSchedule(FUEL6_COUNTER, FUEL6_COMPARE, FUEL6_TIMER_DISABLE, FUEL6_TIMER_ENABLE),
#endif
#if (INJ_CHANNELS >= 7)
  FuelSchedule(FUEL7_COUNTER, FUEL7_COMPARE, FUEL7_TIMER_DISABLE, FUEL7_TIMER_ENABLE),
#endif
#if (INJ_CHANNELS >= 8)
  FuelSchedule(FUEL8_COUNTER, FUEL8_COMPARE, FUEL8_TIMER_DISABLE, FUEL8_TIMER_ENABLE),
#endif
};

void nullInjCallback(void)
{
  /* Do nothing. */
}

void FuelSchedule::reset(void)
{
  noInterrupts();

  Status = OFF;
  start.pCallback = nullInjCallback;
  end.pCallback = nullInjCallback;
  pTimerDisable();

  interrupts();
}

void _setFuelScheduleRunning(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  schedule.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
  COMPARE_TYPE timeout_timer_compare;

  if (timeout > MAX_TIMER_PERIOD)
  {
    // If the timeout is >4x (Each tick represents 4uS on a mega2560,
    // other boards will be different) the maximum allowed value of unsigned int (65535),
    // the timer compare value will overflow when applied causing erratic
    // behaviour such as erroneous squirts.
    timeout_timer_compare = uS_TO_TIMER_COMPARE(MAX_TIMER_PERIOD - 1);
  }
  else
  {
    //Normal case.
    timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout);
  }


  //The following must be enclosed in the noInterupts block to avoid contention
  //caused if the relevant interrupt fires before the state is fully set
  noInterrupts();

  schedule.startCompare = schedule.counter + timeout_timer_compare;
  schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration);
  SET_COMPARE(schedule.compare, schedule.startCompare); //Use the B compare unit of timer 3
  schedule.Status = PENDING; //Turn this schedule on

  schedule.pTimerEnable();

  interrupts();
}

void _setFuelScheduleNext(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule.counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
  schedule.hasNextSchedule = true;
}

// Shared ISR function for all fuel timers.
static void fuelScheduleISR(FuelSchedule &schedule)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    schedule.start.pCallback();
    //Set the status to be in progress (ie The start callback has been called,
    //but not the end callback)
    schedule.Status = RUNNING;
    //Doing this here prevents a potential overflow on restarts
    SET_COMPARE(schedule.compare,
                schedule.counter + uS_TO_TIMER_COMPARE(schedule.duration) );
  }
  else if (schedule.Status == RUNNING)
  {
      schedule.end.pCallback();
      schedule.Status = OFF; //Turn off the schedule

      //If there is a next schedule queued up, activate it
      if(schedule.hasNextSchedule)
      {
        SET_COMPARE(schedule.compare, schedule.nextStartCompare);
        SET_COMPARE(schedule.endCompare, schedule.nextEndCompare);
        schedule.Status = PENDING;
        schedule.hasNextSchedule = false;
      }
      else
      {
        schedule.pTimerDisable();
      }
  }
  else if (schedule.Status == OFF)
  {
    //Safety check. Turn off this output compare unit and return without
    //performing any action
    schedule.pTimerDisable();
  }
}

/*******************************************************************************************************************************************************************************************************/
/** fuelSchedule*Interrupt (All 8 ISR functions below) get called (as timed interrupts)
*   when either the start time or the duration time are reached.
* This calls the relevant callback function (startCallback or endCallback) depending
*  on the status (PENDING => Needs to run, RUNNING => Needs to stop) of the schedule.
* The status of schedule is managed here based on startCallback /endCallback function called:
* - startCallback - change scheduler into RUNNING state
* - endCallback - change scheduler into OFF state (or PENDING if schedule.hasNextSchedule is set)
*/
//Timer3A (fuel schedule 1) Compare Vector
//AVR chips use the ISR for this
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
//fuelSchedules 1 and 5
ISR(TIMER3_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule1Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel1]);
  }

//AVR chips use the ISR for this
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule2Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel2]);
  }

//AVR chips use the ISR for this
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule3Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel3]);
  }

//AVR chips use the ISR for this
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule4Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel4]);
  }

#if INJ_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule5Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel5]);
  }
#endif

#if INJ_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule6Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel6]);
  }
#endif

#if INJ_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule7Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel7]);
  }
#endif

#if INJ_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule8Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedules[injChannel8]);
  }
#endif

void disablePendingFuelSchedule(byte const channel)
{
  if (channel < injChannelCount)
  {
    noInterrupts();

    if (fuelSchedules[channel].Status == PENDING)
    {
      fuelSchedules[channel].Status = OFF;
    }

    interrupts();
  }
}

