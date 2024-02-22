/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
Timers are used for having actions performed repeatedly at a fixed interval (Eg every 100ms)
They should not be confused with Schedulers, which are for performing an action once at a given point of time in the future

Timers are typically low resolution (Compared to Schedulers), with maximum frequency currently being approximately every 10ms
*/
#include "timers.h"
#include "globals.h"
#include "sensors.h"
#include "scheduler.h"
#include "ignition_contexts.h"
#include "ignition_control.h"
#include "injector_control.h"
#include "speeduino.h"
#include "scheduler.h"
#include "auxiliaries.h"
#include "comms.h"
#include "maths.h"
#include "fuel_pump.h"

#if defined(CORE_AVR)
  #include <avr/wdt.h>
#endif

volatile uint16_t lastRPM_100ms; //Need to record this for rpmDOT calculation
volatile byte loop5ms;
volatile byte loop33ms;
volatile byte loop66ms;
volatile byte loop100ms;
volatile byte loop250ms;
volatile int loopSec;

volatile unsigned int dwellLimit_uS;

volatile uint8_t tachoEndTime; //The time (in ms) that the tacho pulse needs to end at
volatile TachoOutputStatus tachoOutputFlag;
volatile uint16_t tachoSweepIncr;
volatile uint16_t tachoSweepAccum;
volatile uint8_t testInjectorPulseCount = 0;
volatile uint8_t testIgnitionPulseCount = 0;

#if defined (CORE_TEENSY)
  IntervalTimer lowResTimer;
#endif

void initialiseTimers(void)
{
  lastRPM_100ms = 0;
  loop5ms = 0;
  loop33ms = 0;
  loop66ms = 0;
  loop100ms = 0;
  loop250ms = 0;
  loopSec = 0;
  tachoOutputFlag = TACHO_INACTIVE;
}

static void
updateTacho(void)
{
  //Tacho is flagged as being ready for a pulse by the ignition outputs,
  //or the sweep interval upon startup

  // See if we're in power-on sweep mode
  if (currentStatus.tachoSweepEnabled)
  {
    // Stop the sweep after SWEEP_TIME, or if real tach signals have started
    if (currentStatus.engine != 0 || ms_counter >= TACHO_SWEEP_TIME_MS)
    {
      currentStatus.tachoSweepEnabled = false;
    }
    else
    {
      // Ramp the needle smoothly to the max over the SWEEP_RAMP time
      if (ms_counter < TACHO_SWEEP_RAMP_MS)
      {
        tachoSweepAccum += map(ms_counter, 0, TACHO_SWEEP_RAMP_MS, 0, tachoSweepIncr);
      }
      else
      {
        tachoSweepAccum += tachoSweepIncr;
      }

      // Each time it rolls over, it's time to pulse the Tach
      if (tachoSweepAccum >= MS_PER_SEC)
      {
        tachoOutputFlag = READY;
        tachoSweepAccum -= MS_PER_SEC;
      }
    }
  }

  //Tacho output check. This code will not do anything if tacho pulse duration is fixed to coil dwell.
  if(tachoOutputFlag == READY)
  {
    //Check for half speed tacho
    if (configPage2.tachoDiv == 0 || currentStatus.tachoAlt)
    {
      TachOut.off();
      //ms_counter is cast down to a byte as the tacho duration can only be in
      //the range of 1-6, so no extra resolution above that is required
      tachoEndTime = (uint8_t)ms_counter + configPage2.tachoDuration;
      tachoOutputFlag = ACTIVE;
    }
    else
    {
      //Don't run on this pulse (Half speed tacho)
      tachoOutputFlag = TACHO_INACTIVE;
    }
    //Flip the alternating value in case half speed tacho is in use.
    currentStatus.tachoAlt = !currentStatus.tachoAlt;
  }
  else if(tachoOutputFlag == ACTIVE)
  {
    //If the tacho output is already active, check whether it's reached it's end time
    if((uint8_t)ms_counter == tachoEndTime)
    {
      TachOut.on();
      tachoOutputFlag = TACHO_INACTIVE;
    }
  }
}

static void
updateFlex(void)
{
  byte tempEthPct;

  if (flexCounter < 50)
  {
    //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol).
    //Subtracting 50 from the frequency therefore gives the ethanol percentage.
    tempEthPct = 0;
  }
  else if (flexCounter > 151) //1 pulse buffer
  {
    if (flexCounter < 169)
    {
      tempEthPct = 100;
    }
    else
    {
      //This indicates an error condition. Spec of the sensor is that errors are above 170Hz)
      tempEthPct = 0;
    }
  }
  else
  {
    //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol).
    //Subtracting 50 from the frequency therefore gives the ethanol percentage.
    tempEthPct = flexCounter - 50;
  }
  flexCounter = 0;

  //Off by 1 error check
  if (tempEthPct == 1)
  {
    tempEthPct = 0;
  }

  currentStatus.ethanolPct =
    ADC_FILTER(tempEthPct, configPage4.FILTER_FLEX, currentStatus.ethanolPct);

  //Continental flex sensor fuel temperature can be read with following formula:
  //(Temperature = (41.25 * pulse width(ms)) - 81.25). 1000μs = -40C and 5000μs = 125C
  if (flexPulseWidth > 5000)
  {
    flexPulseWidth = 5000;
  }
  else if (flexPulseWidth < 1000)
  {
    flexPulseWidth = 1000;
  }
  currentStatus.fuelTemp = div100((int16_t)(((4224 * (long)flexPulseWidth) >> 10) - 8125));
}

//Timer2 Overflow Interrupt Vector, called when the timer overflows.
//Executes every ~1ms.
#if defined(CORE_AVR) //AVR chips use the ISR for this
//This MUST be no block. Turning NO_BLOCK off messes with timing accuracy.
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //cppcheck-suppress misra-c2012-8.2
#else
void oneMSInterval(void) //Most ARM chips can simply call a function
#endif
{
  BIT_SET(TIMER_mask, BIT_TIMER_1KHZ);
  ms_counter++;

  //Increment Loop Counters
  loop5ms++;
  loop33ms++;
  loop66ms++;
  loop100ms++;
  loop250ms++;
  loopSec++;

  //Overdwell check
  //Set a target time in the past that all coil charging must have begun after.
  //If the coil charge began before this time, it's been running too long
  uint32_t targetOverdwellTime = micros() - dwellLimit_uS;
  //Dwell limiter is disabled during cranking on setups using the locked cranking timing.
  //WE HAVE to do the RPM check here as relying on the engine cranking bit can be
  //potentially too slow in updating
  bool isCrankLocked = configPage4.ignCranklock && currentStatus.RPM < currentStatus.crankRPM;

  if (configPage4.useDwellLim == 1 && isCrankLocked != true)
  {
    for (size_t i = 0; i < ignChannelCount; i++)
    {
      ignitions.ignition((ignitionChannelID_t)i).applyOverDwellCheck(targetOverdwellTime);
    }
  }

  updateTacho();

  //200Hz loop
  if (loop5ms == 5)
  {
    loop5ms = 0; //Reset counter
    BIT_SET(TIMER_mask, BIT_TIMER_200HZ);
  }

  //30Hz loop
  if (loop33ms == 33)
  {
    loop33ms = 0;

    //Pulse fuel and ignition test outputs are set at 30Hz
    if (BIT_CHECK(currentStatus.testOutputs, 1) && (currentStatus.RPM == 0))
    {
      //Check for pulsed injector output test
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ1_CMD_BIT))
      {
        openSingleInjector(injector_id_1);
      }
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ2_CMD_BIT))
      {
        openSingleInjector(injector_id_2);
      }
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ3_CMD_BIT))
      {
        openSingleInjector(injector_id_3);
      }
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ4_CMD_BIT))
      {
        openSingleInjector(injector_id_4);
      }
#if INJ_CHANNELS >= 5
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ5_CMD_BIT))
      {
        openSingleInjector(injector_id_5);
      }
#endif
#if INJ_CHANNELS >= 6
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ6_CMD_BIT))
      {
        openSingleInjector(injector_id_6);
      }
#endif
#if INJ_CHANNELS >= 7
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ7_CMD_BIT))
      {
        openSingleInjector(injector_id_7);
      }
#endif
#if INJ_CHANNELS >= 8
      if (BIT_CHECK(HWTest_INJ_Pulsed, INJ8_CMD_BIT))
      {
        openSingleInjector(injector_id_8);
      }
#endif
      testInjectorPulseCount = 0;

      //Check for pulsed ignition output test
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN1_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_1);
      }
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN2_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_2);
      }
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN3_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_3);
      }
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN4_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_4);
      }
#if IGN_CHANNELS >= 5
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN5_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_5);
      }
#endif
#if IGN_CHANNELS >= 6
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN6_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_6);
      }
#endif
#if IGN_CHANNELS >= 7
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN7_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_7);
      }
#endif
#if IGN_CHANNELS >= 8
      if (BIT_CHECK(HWTest_IGN_Pulsed, IGN8_CMD_BIT))
      {
        singleCoilBeginCharge(ignition_id_8);
      }
#endif
      testIgnitionPulseCount = 0;
    }

    BIT_SET(TIMER_mask, BIT_TIMER_30HZ);
  }

  //15Hz loop
  if (loop66ms == 66)
  {
    loop66ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_15HZ);
  }

  //10Hz loop
  if (loop100ms == 100)
  {
    loop100ms = 0; //Reset counter
    BIT_SET(TIMER_mask, BIT_TIMER_10HZ);

    //This is the RPM per second that the engine has accelerated/decelerated in the last loop
    currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10;
    lastRPM_100ms = currentStatus.RPM; //Record the current RPM for next calc

    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
    {
      runSecsX10++;
    }
    else
    {
      runSecsX10 = 0;
    }

    if (!currentStatus.injPrimed
        && seclx10 == configPage2.primingDelay
        && currentStatus.RPM == 0)
    {
      beginInjectorPriming();
      currentStatus.injPrimed = true;
    }
    seclx10++;
  }

  //4Hz loop
  if (loop250ms == 250)
  {
    loop250ms = 0; //Reset Counter
    BIT_SET(TIMER_mask, BIT_TIMER_4HZ);
#if defined(CORE_STM32) //debug purpose, only visual for running code
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif
  }

  //1Hz loop
  if (loopSec == 1000)
  {
    loopSec = 0; //Reset counter.
    BIT_SET(TIMER_mask, BIT_TIMER_1HZ);

    dwellLimit_uS = (1000 * configPage4.dwellLimit); //Update uS value in case setting has changed
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10);

    //**************************************************************************************************************************************************
    //This updates the runSecs variable
    //If the engine is running or cranking, we need to update the run time counter.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
    {
      //NOTE - There is a potential for a ~1sec gap between engine crank starting
      //and the runSec number being incremented. This may delay ASE!
      //Ensure we cap out at 255 and don't overflow. (which would reset ASE and
      //cause problems with the closed loop fuelling (Which has to wait for the O2 to warmup))
      if (currentStatus.runSecs <= 254)
      {
        currentStatus.runSecs++;
      }
    }
    //**************************************************************************************************************************************************
    //This records the number of main loops the system has completed in the last second
    currentStatus.loopsPerSecond = mainLoopCount;
    mainLoopCount = 0;
    //**************************************************************************************************************************************************
    //increment secl (secl is simply a counter that increments every second and is used to track whether the system has unexpectedly reset
    currentStatus.secl++;
    //**************************************************************************************************************************************************
    //Check the fan output status
    if (configPage2.fanEnable >= 1)
    {
       fanControl(); // Function to turn the cooling fan on/off
    }

    //**************************************************************************************************************************************************
    //Set the flex reading (if enabled). The flexCounter is updated with every pulse from the sensor.
    //If cleared once per second, we get a frequency reading
    if (configPage2.flexEnabled)
    {
      updateFlex();
    }
  }

  //Turn off any of the pulsed testing outputs if they are active and have been running for long enough
  if( BIT_CHECK(currentStatus.testOutputs, 1) )
  {
    //Check for pulsed injector output test
    if( (HWTest_INJ_Pulsed > 0)  )
    {
      if(testInjectorPulseCount >= configPage13.hwTestInjDuration)
      {
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ1_CMD_BIT)) { closeSingleInjector(injector_id_1); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ2_CMD_BIT)) { closeSingleInjector(injector_id_2); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ3_CMD_BIT)) { closeSingleInjector(injector_id_3); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ4_CMD_BIT)) { closeSingleInjector(injector_id_4); }
#if INJ_CHANNELS >= 5
        if (BIT_CHECK(HWTest_INJ_Pulsed, INJ5_CMD_BIT))
        {
          closeSingleInjector(injector_id_5);
        }
#endif
#if INJ_CHANNELS >= 6
        if (BIT_CHECK(HWTest_INJ_Pulsed, INJ6_CMD_BIT))
        {
          closeSingleInjector(injector_id_6);
        }
#endif
#if INJ_CHANNELS >= 7
        if (BIT_CHECK(HWTest_INJ_Pulsed, INJ7_CMD_BIT))
        {
          closeSingleInjector(injector_id_7);
        }
#endif
#if INJ_CHANNELS >= 8
        if (BIT_CHECK(HWTest_INJ_Pulsed, INJ8_CMD_BIT))
        {
          closeSingleInjector(injector_id_8);
        }
#endif
        testInjectorPulseCount = 0;
      }
      else { testInjectorPulseCount++; }
    }


    //Check for pulsed ignition output test
    if( (HWTest_IGN_Pulsed > 0) )
    {
      if(testIgnitionPulseCount >= configPage13.hwTestIgnDuration)
      {
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN1_CMD_BIT)) { singleCoilEndCharge(ignition_id_1); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN2_CMD_BIT)) { singleCoilEndCharge(ignition_id_2); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN3_CMD_BIT)) { singleCoilEndCharge(ignition_id_3); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN4_CMD_BIT)) { singleCoilEndCharge(ignition_id_4); }
#if IGN_CHANNELS >= 5
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN5_CMD_BIT)) { singleCoilEndCharge(ignition_id_5); }
#endif
#if IGN_CHANNELS >= 6
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN6_CMD_BIT)) { singleCoilEndCharge(ignition_id_6); }
#endif
#if IGN_CHANNELS >= 7
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN7_CMD_BIT)) { singleCoilEndCharge(ignition_id_7); }
#endif
#if IGN_CHANNELS >= 8
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN8_CMD_BIT)) { singleCoilEndCharge(ignition_id_8); }
#endif
        testIgnitionPulseCount = 0;
      }
      else
      {
        testIgnitionPulseCount++;
      }
    }
  }

#if defined(CORE_AVR) //AVR chips use the ISR for this
    //Reset Timer2 to trigger in another ~1ms
    TCNT2 = 131;            //Preload timer2 with 100 cycles, leaving 156 till overflow.
#endif
}
