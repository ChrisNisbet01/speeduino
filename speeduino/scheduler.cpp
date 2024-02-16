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
#include "globals.h"
#include "scheduler.h"
#include "timers.h"
#include "schedule_calcs.h"
#include "injector_contexts.h"
#include "ignition_contexts.h"

#define DWELL_SMOOTHED_ALPHA 30
#define DWELL_SMOOTHED(current_dwell, input) ((((long)(input) * (256 - DWELL_SMOOTHED_ALPHA) + ((long)(current_dwell) * DWELL_SMOOTHED_ALPHA))) >> 8)
//#define DWELL_SMOOTHED(current_dwell, input) (current_dwell) //Can be use to disable the above for testing


FuelSchedule fuelSchedule1(FUEL1_COUNTER, FUEL1_COMPARE, FUEL1_TIMER_DISABLE, FUEL1_TIMER_ENABLE);
FuelSchedule fuelSchedule2(FUEL2_COUNTER, FUEL2_COMPARE, FUEL2_TIMER_DISABLE, FUEL2_TIMER_ENABLE);
FuelSchedule fuelSchedule3(FUEL3_COUNTER, FUEL3_COMPARE, FUEL3_TIMER_DISABLE, FUEL3_TIMER_ENABLE);
FuelSchedule fuelSchedule4(FUEL4_COUNTER, FUEL4_COMPARE, FUEL4_TIMER_DISABLE, FUEL4_TIMER_ENABLE);

#if (INJ_CHANNELS >= 5)
FuelSchedule fuelSchedule5(FUEL5_COUNTER, FUEL5_COMPARE, FUEL5_TIMER_DISABLE, FUEL5_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 6)
FuelSchedule fuelSchedule6(FUEL6_COUNTER, FUEL6_COMPARE, FUEL6_TIMER_DISABLE, FUEL6_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 7)
FuelSchedule fuelSchedule7(FUEL7_COUNTER, FUEL7_COMPARE, FUEL7_TIMER_DISABLE, FUEL7_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 8)
FuelSchedule fuelSchedule8(FUEL8_COUNTER, FUEL8_COMPARE, FUEL8_TIMER_DISABLE, FUEL8_TIMER_ENABLE);
#endif

IgnitionSchedule ignitionSchedule1(IGN1_COUNTER, IGN1_COMPARE, IGN1_TIMER_DISABLE, IGN1_TIMER_ENABLE);
IgnitionSchedule ignitionSchedule2(IGN2_COUNTER, IGN2_COMPARE, IGN2_TIMER_DISABLE, IGN2_TIMER_ENABLE);
IgnitionSchedule ignitionSchedule3(IGN3_COUNTER, IGN3_COMPARE, IGN3_TIMER_DISABLE, IGN3_TIMER_ENABLE);
IgnitionSchedule ignitionSchedule4(IGN4_COUNTER, IGN4_COMPARE, IGN4_TIMER_DISABLE, IGN4_TIMER_ENABLE);

#if IGN_CHANNELS >= 5
IgnitionSchedule ignitionSchedule5(IGN5_COUNTER, IGN5_COMPARE, IGN5_TIMER_DISABLE, IGN5_TIMER_ENABLE);
#endif
#if IGN_CHANNELS >= 6
IgnitionSchedule ignitionSchedule6(IGN6_COUNTER, IGN6_COMPARE, IGN6_TIMER_DISABLE, IGN6_TIMER_ENABLE);
#endif
#if IGN_CHANNELS >= 7
IgnitionSchedule ignitionSchedule7(IGN7_COUNTER, IGN7_COMPARE, IGN7_TIMER_DISABLE, IGN7_TIMER_ENABLE);
#endif
#if IGN_CHANNELS >= 8
IgnitionSchedule ignitionSchedule8(IGN8_COUNTER, IGN8_COMPARE, IGN8_TIMER_DISABLE, IGN8_TIMER_ENABLE);
#endif

static void initialiseFuelSchedules(void)
{
  injectors.injector(injChannel1).fuelSchedule = &fuelSchedule1;
  injectors.injector(injChannel2).fuelSchedule = &fuelSchedule2;
  injectors.injector(injChannel3).fuelSchedule = &fuelSchedule3;
  injectors.injector(injChannel4).fuelSchedule = &fuelSchedule4;
#if INJ_CHANNELS >= 5
  injectors.injector(injChannel5).fuelSchedule = &fuelSchedule5;
#endif
#if INJ_CHANNELS >= 6
  injectors.injector(injChannel6).fuelSchedule = &fuelSchedule6;
#endif
#if INJ_CHANNELS >= 7
  injectors.injector(injChannel7).fuelSchedule = &fuelSchedule7;
#endif
#if INJ_CHANNELS >= 8
  injectors.injector(injChannel8).fuelSchedule = &fuelSchedule8;
#endif
}

static void initialiseIgnitionSchedules(void)
{
  ignitions.ignition(ignChannel1).ignitionSchedule = &ignitionSchedule1;
  ignitions.ignition(ignChannel2).ignitionSchedule = &ignitionSchedule2;
  ignitions.ignition(ignChannel3).ignitionSchedule = &ignitionSchedule3;
  ignitions.ignition(ignChannel4).ignitionSchedule = &ignitionSchedule4;
#if IGN_CHANNELS >= 5
  ignitions.ignition(ignChannel5).ignitionSchedule = &ignitionSchedule5;
#endif
#if IGN_CHANNELS >= 6
  ignitions.ignition(ignChannel6).ignitionSchedule = &ignitionSchedule6;
#endif
#if IGN_CHANNELS >= 7
  ignitions.ignition(ignChannel7).ignitionSchedule = &ignitionSchedule7;
#endif
#if IGN_CHANNELS >= 8
  ignitions.ignition(ignChannel8).ignitionSchedule = &ignitionSchedule8;
#endif
}

void initialiseSchedulers(void)
{
  initialiseFuelSchedules();
  initialiseIgnitionSchedules();

  for (size_t i = injChannel1; i < injChannelCount; i++)
  {
    injector_context_st &injector = injectors.injector((injectorChannelID_t)i);

    injector.reset();
  }

  for (size_t i = ignChannel1; i < ignChannelCount; i++)
  {
    ignition_context_st &ignition = ignitions.ignition((ignitionChannelID_t)i);

    ignition.reset();
  }
}

void _setFuelScheduleRunning(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  schedule.duration = duration;

  //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
  noInterrupts();

  schedule.startCompare = schedule.counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration);
  SET_COMPARE(schedule.compare, schedule.startCompare); //Use the B compare unit of timer 3
  schedule.Status = PENDING; //Turn this schedule on

  interrupts();

  schedule.pTimerEnable();
}

void _setFuelScheduleNext(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule.counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
  schedule.hasNextSchedule = true;
}

void _setIgnitionScheduleRunning(
  IgnitionSchedule &schedule, unsigned long timeout, unsigned long durationMicrosecs)
{
  schedule.duration = durationMicrosecs;

  //Need to check that the timeout doesn't exceed the overflow
  COMPARE_TYPE timeout_timer_compare;
  // If the timeout is >4x (Each tick represents 4uS) the maximum allowed
  // value of unsigned int (65535), the timer compare value will overflow when
  // applied causing erratic behaviour such as erroneous sparking.
  if (timeout > MAX_TIMER_PERIOD)
  {
    timeout_timer_compare = uS_TO_TIMER_COMPARE(MAX_TIMER_PERIOD - 1);
  }
  else //Normal case
  {
    timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout);
  }

  noInterrupts();

  schedule.startCompare = schedule.counter + timeout_timer_compare;

  //The .endCompare value may be set by the per-tooth timing in decoders.cpp.
  //The check here is to ensure that per-tooth control is not overridden.
  if (!schedule.endScheduleSetByDecoder)
  {
    schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(durationMicrosecs);
  }

  SET_COMPARE(schedule.compare, schedule.startCompare);
  schedule.Status = PENDING; //Turn this schedule on
  interrupts();

  schedule.pTimerEnable();
}

void _setIgnitionScheduleNext(IgnitionSchedule &schedule, unsigned long timeout, unsigned long durationMicrosecs)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule.counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(durationMicrosecs);
  schedule.hasNextSchedule = true;
}


void refreshIgnitionSchedule1(unsigned long timeToEnd)
{
  //Must have the threshold check here otherwise it can cause a condition where
  //the compare fires twice, once after the other, both for the end

  noInterrupts();

  if (ignitionSchedule1.Status == RUNNING && timeToEnd < ignitionSchedule1.duration)
  {
    ignitionSchedule1.endCompare = IGN1_COUNTER + uS_TO_TIMER_COMPARE(timeToEnd);
    SET_COMPARE(IGN1_COMPARE, ignitionSchedule1.endCompare);
  }

  interrupts();
}

/** Perform the injector priming pulses.
 * Set these to run at an arbitrary time in the future (100us).
 * The prime pulse value is in ms*10, so need to multiple by 100 to get to uS
 */
extern void beginInjectorPriming(void)
{
  static unsigned long const priming_delay_us = 100;
  unsigned long primingValue =
    table2D_getValue(&PrimingPulseTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);

  if (primingValue > 0 && currentStatus.TPS < configPage4.floodClear)
  {
    // To achieve long enough priming pulses, the values in tuner studio are divided by 0.5 instead of 0.1,
    // so multiplier of 5 is required.
    // XXX - Should that be _multiplied_ by 0.5, which also means the value should be multiplied by 2?
    static unsigned config_multiplier = 5;
    primingValue = primingValue * priming_delay_us * config_multiplier;

    for (size_t i = 0; i < injectors.maxOutputs; i++)
    {
      setFuelSchedule(
        *injectors.injector((injectorChannelID_t)i).fuelSchedule, priming_delay_us, primingValue);
    }
  }
}

// Shared ISR function for all fuel timers.
static void fuelScheduleISR(FuelSchedule &schedule)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    schedule.start.pCallback(schedule.start.injector_ids[0], schedule.start.injector_ids[1]);
    //Set the status to be in progress (ie The start callback has been called,
    //but not the end callback)
    schedule.Status = RUNNING;
    //Doing this here prevents a potential overflow on restarts
    SET_COMPARE(schedule.compare,
                schedule.counter + uS_TO_TIMER_COMPARE(schedule.duration) );
  }
  else if (schedule.Status == RUNNING)
  {
      schedule.end.pCallback(schedule.end.injector_ids[0], schedule.end.injector_ids[1]);
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
    fuelScheduleISR(*injectors.injector(injChannel1).fuelSchedule);
  }

//AVR chips use the ISR for this
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule2Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(*injectors.injector(injChannel2).fuelSchedule);
  }

//AVR chips use the ISR for this
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule3Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(*injectors.injector(injChannel3).fuelSchedule);
  }

//AVR chips use the ISR for this
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule4Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(*injectors.injector(injChannel4).fuelSchedule);
  }

#if INJ_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule5Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(*injectors.injector(injChannel5).fuelSchedule);
  }
#endif

#if INJ_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule6Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(*injectors.injector(injChannel6).fuelSchedule);
  }
#endif

#if INJ_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule7Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(*injectors.injector(injChannel7).fuelSchedule);
  }
#endif

#if INJ_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule8Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(*injectors.injector(injChannel8).fuelSchedule);
  }
#endif

// Shared ISR function for all ignition timers.
static void ignitionScheduleISR(IgnitionSchedule &schedule)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    schedule.start.pCallback(schedule.start.coil_ids[0], schedule.start.coil_ids[1]);
    //Set the status to be in progress (ie The start callback has been called,
    //but not the end callback)
    schedule.Status = RUNNING;
    schedule.startTime = micros();
    if(schedule.endScheduleSetByDecoder)
    {
        SET_COMPARE(schedule.compare, schedule.endCompare);
    }
    else
    {
        SET_COMPARE(schedule.compare,
                    schedule.counter + uS_TO_TIMER_COMPARE(schedule.duration));
    } //Doing this here prevents a potential overflow on restarts
  }
  else if (schedule.Status == RUNNING)
  {
    schedule.end.pCallback(schedule.end.coil_ids[0], schedule.end.coil_ids[1]);
    schedule.endScheduleSetByDecoder = false;
    ignitionCount = ignitionCount + 1; //Increment the ignition counter
    currentStatus.actualDwell =
      DWELL_SMOOTHED(currentStatus.actualDwell, micros() - schedule.startTime);

    //If there is a next schedule queued up, activate it
    if (schedule.hasNextSchedule)
    {
      SET_COMPARE(schedule.compare, schedule.nextStartCompare);
      schedule.Status = PENDING;
      schedule.hasNextSchedule = false;
    }
    else
    {
      schedule.Status = OFF; //Turn off the schedule
      schedule.pTimerDisable();
    }
  }
  else if (schedule.Status == OFF)
  {
    //Catch any spurious interrupts. This really shouldn't ever be called, but there as a safety
    schedule.pTimerDisable();
  }
}

#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule1Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule1);
  }

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule2Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule2);
  }
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule3Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule3);
  }
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule4Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule4);
  }
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule5Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule5);
  }
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule6Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule6);
  }
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule7Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule7);
  }
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule8Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule8);
  }
#endif

void disablePendingFuelSchedule(byte channel)
{
  if (channel < injChannelCount)
  {
    noInterrupts();

    injector_context_st &injector =
      injectors.injector((injectorChannelID_t)channel);

    if (injector.fuelSchedule->Status == PENDING)
    {
      injector.fuelSchedule->Status = OFF;
    }

    interrupts();
  }
}

void disablePendingIgnSchedule(byte channel)
{
  if (channel < ignChannelCount)
  {
    noInterrupts();

    ignition_context_st &ignition =
      ignitions.ignition((ignitionChannelID_t)channel);

    if(ignition.ignitionSchedule->Status == PENDING)
    {
      ignition.ignitionSchedule->Status = OFF;
    }

    interrupts();
  }
}
