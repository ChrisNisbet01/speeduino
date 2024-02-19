/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/** @file
 * Speeduino initialisation and main loop.
 */
#include <stdint.h>

#include "globals.h"
#include "speeduino.h"
#include "scheduler.h"
#include "comms.h"
#include "comms_legacy.h"
#include "comms_secondary.h"
#include "maths.h"
#include "corrections.h"
#include "timers.h"
#include "decoders.h"
#include "idle.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "storage.h"
#include "crankMaths.h"
#include "init.h"
#include "utilities.h"
#include "engineProtection.h"
#include "secondaryTables.h"
#include "comms_CAN.h"
#include "SD_logger.h"
#include "schedule_calcs.h"
#include "auxiliaries.h"
#include "engine_load_calcs.h"
#include "injector_contexts.h"
#include "ignition_contexts.h"
#include "fuel_pump.h"
#include "calculateInjectorStaging.h"

uint16_t req_fuel_uS = 0; /**< The required fuel variable (As calculated by TunerStudio) in uS */
uint16_t inj_opentime_uS = 0;

uint32_t rollingCutLastRev = 0; /**< Tracks whether we're on the same or a different rev for the rolling cut */

uint16_t staged_req_fuel_mult_pri = 0;
uint16_t staged_req_fuel_mult_sec = 0;
#ifndef UNIT_TEST // Scope guard for unit testing
void setup(void)
{
  currentStatus.initialisationComplete = false; //Tracks whether the initialiseAll() function has run completely
  initialiseAll();
}

/** Speeduino main loop.
 *
 * Main loop chores (roughly in the order that they are performed):
 * - Check if serial comms or tooth logging are in progress (send or receive, prioritise communication)
 * - Record loop timing vars
 * - Check tooth time, update @ref statuses (currentStatus) variables
 * - Read sensors
 * - get VE for fuel calcs and spark advance for ignition
 * - Check crank/cam/tooth/timing sync (skip remaining ops if out-of-sync)
 * - execute doCrankSpeedCalcs()
 *
 * single byte variable @ref LOOP_TIMER plays a big part here as:
 * - it contains expire-bits for interval based frequency driven events (e.g. 15Hz, 4Hz, 1Hz)
 * - Can be tested for certain frequency interval being expired by (eg) BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)
 *
 */
void loop(void)
{
  mainLoopCount++;
  LOOP_TIMER = TIMER_mask;

  //SERIAL Comms
  //Initially check that the last serial send values request is not still outstanding
  if (serialTransmitInProgress())
  {
    serialTransmit();
  }

  //Check for any new or in-progress requests from serial.
  if (Serial.available() > 0 || serialRecieveInProgress())
  {
    serialReceive();
  }

  //Check for any CAN comms requiring action
#     if defined(secondarySerial_AVAILABLE)
  //if can or secondary serial interface is enabled then check for requests.
  if (configPage9.enable_secondarySerial == 1)  //secondary serial interface enabled
  {
    if (((mainLoopCount & 31) == 1) || (secondarySerial.available() > SERIAL_BUFFER_THRESHOLD))
    {
      if (secondarySerial.available() > 0)
      {
        secondserial_Command();
      }
    }
  }
#     endif
#     if defined (NATIVE_CAN_AVAILABLE)
  if (configPage9.enable_intcan == 1) // use internal can module
  {
    //check local can module
    // if ( BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ) or (CANbus0.available())
    while (CAN_read())
    {
      can_Command();
      readAuxCanBus();
      if (configPage2.canWBO > 0)
      {
        receiveCANwbo();
      }
    }
  }
#     endif

  if (currentLoopTime > micros_safe())
  {
    //Occurs when micros() has overflowed
    deferEEPROMWritesUntil = 0; //Required to ensure that EEPROM writes are not deferred indefinitely
  }

  currentLoopTime = micros_safe();
  uint32_t timeToLastTooth = (currentLoopTime - toothLastToothTime);

  //Check how long ago the last tooth was seen compared to now. If it was more
  //than MAX_STALL_TIME ago then the engine is probably stopped.
  //toothLastToothTime can be greater than currentLoopTime if a pulse occurs
  //between getting the latest time and doing the comparison.
  if (timeToLastTooth < MAX_STALL_TIME
      || toothLastToothTime > currentLoopTime)
  {
    currentStatus.longRPM = getRPM(); //Long RPM is included here
    currentStatus.RPM = currentStatus.longRPM;
    currentStatus.RPMdiv100 = div100(currentStatus.RPM);
    fuelPump.turnOn();
  }
  else
  {
    //We reach here if the time between teeth is too great.
    //This VERY likely means the engine has stopped.
    currentStatus.RPM = 0;
    currentStatus.RPMdiv100 = 0;
    injectors.injector(injChannel1).PW = 0;
    currentStatus.VE = 0;
    currentStatus.VE2 = 0;
    toothLastToothTime = 0;
    toothLastSecToothTime = 0;
    //toothLastMinusOneToothTime = 0;
    currentStatus.hasSync = false;
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
    currentStatus.runSecs = 0; //Reset the counter for number of seconds running.
    currentStatus.startRevolutions = 0;
    toothSystemCount = 0;
    secondaryToothCount = 0;
    MAPcurRev = 0;
    MAPcount = 0;
    currentStatus.rpmDOT = 0;
    AFRnextCycle = 0;
    ignitionCount = 0;

    ignitions.setAllOff();
    injectors.setAllOff();

    //Turn off the fuel pump if the priming is complete.
    if (!fuelPriming.arePriming())
    {
      fuelPump.turnOff();
    }

    if (!configPage6.iacPWMrun)
    {
      //Turn off the idle PWM
      disableIdle();
    }
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK); //Clear cranking bit (Can otherwise get stuck 'on' even with 0 rpm)
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP); //Same as above except for WUE
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN); //Same as above except for RUNNING status
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Same as above except for ASE status
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Same as above but the accel enrich (If using MAP accel enrich a stall will cause this to trigger)
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC); //Same as above but the decel enleanment
    //This is a safety check. If for some reason the interrupts have got screwed up (Leading to 0rpm), this resets them.
    //It can possibly be run much less frequently.
    //This should only be run if the high speed logger are off because it will change the trigger interrupts back to defaults rather than the logger versions
    if (!currentStatus.toothLogEnabled && currentStatus.compositeTriggerUsed == 0)
    {
      initialiseTriggers();
    }

    VVT1_PIN_OFF();
    VVT2_PIN_OFF();
    DISABLE_VVT_TIMER();
    boostDisable();
    if (configPage4.ignBypassEnabled > 0)
    {
      //Reset the ignition bypass ready for next crank attempt
      IgnBypass.off();
    }
  }

  //***Perform sensor reads***
  //-----------------------------------------------------------------------------------------------------
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1KHZ)) //Every 1ms. NOTE: This is NOT guaranteed to run at 1kHz on AVR systems. It will run at 1kHz if possible or as fast as loops/s allows if not.
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_1KHZ);
    readMAP();
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_200HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_200HZ);
#     if defined(ANALOG_ISR)
    //ADC in free running mode does 1 complete conversion of all 16 channels and then the interrupt is disabled. Every 200Hz we re-enable the interrupt to get another conversion cycle
    BIT_SET(ADCSRA, ADIE); //Enable ADC interrupt
#     endif
  }

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)) //Every 32 loops
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_15HZ);
#if TPS_READ_FREQUENCY == 15
    readTPS(); //TPS reading to be performed every 32 loops (any faster and it can upset the TPSdot sampling time)
#endif
#if defined(CORE_TEENSY35)
    if (configPage9.enable_intcan == 1) // use internal can module
    {
      // this is just to test the interface is sending
      //sendCancommand(3,((configPage9.realtime_base_address & 0x3FF)+ 0x100),currentStatus.TPS,0,0x200);
    }
#endif

    checkLaunchAndFlatShift(); //Check for launch control and flat shift being active

    //And check whether the tooth log buffer is ready
    if (toothHistoryIndex > TOOTH_LOG_SIZE)
    {
      BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
    }



  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) //10 hertz
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_10HZ);
    //updateFullStatus();
    checkProgrammableIO();
    idleControl(); //Perform any idle related actions. This needs to be run at 10Hz to align with the idle taper resolution of 0.1s

    // Air conditioning control
    airConControl();

    currentStatus.vss = getSpeed();
    currentStatus.gear = getGear();

#     ifdef SD_LOGGING
    if (configPage13.onboard_log_file_rate == LOGGER_RATE_10HZ)
    {
      writeSDLogEntry();
    }
#     endif
  }
  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_30HZ)) //30 hertz
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_30HZ);
    //Most boost tends to run at about 30Hz,
    //so placing it here ensures a new target time is fetched frequently enough
    boostControl();

    //VVT may eventually need to be synced with the cam readings
    //(ie run once per cam rev) but for now run at 30Hz
    vvtControl();

    //Water methanol injection
    wmiControl();

#if defined(NATIVE_CAN_AVAILABLE)
    if (configPage2.canBMWCluster)
    {
      sendBMWCluster();
    }
    if (configPage2.canVAGCluster)
    {
      sendVAGCluster();
    }
#endif

#if TPS_READ_FREQUENCY == 30
    readTPS();
#endif
    if (configPage2.canWBO == 0)
    {
      readO2();
      readO2_2();
    }

    readO2();
    readO2_2();

#ifdef SD_LOGGING
    if (configPage13.onboard_log_file_rate == LOGGER_RATE_30HZ)
    {
      writeSDLogEntry();
    }
#endif

    //Check for any outstanding EEPROM writes.
    if (isEepromWritePending()
        && serialStatusFlag == SERIAL_INACTIVE
        && micros() > deferEEPROMWritesUntil)
    {
      writeAllConfig();
    }
  }

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ))
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_4HZ);
    //The IAT and CLT readings can be done less frequently (4 times per second)
    readCLT();
    readIAT();
    readBat();
    nitrousControl();

    //Lookup the current target idle RPM. This is aligned with coolant and so needs to be calculated at the same rate CLT is read
    if (configPage2.idleAdvEnabled >= 1 || configPage6.iacAlgorithm != IAC_ALGORITHM_NONE)
    {
      currentStatus.CLIdleTarget =
        (byte)table2D_getValue(&idleTargetTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
      if (BIT_CHECK(currentStatus.airConStatus, BIT_AIRCON_TURNING_ON)) //Adds Idle Up RPM amount if active
      {
        currentStatus.CLIdleTarget += configPage15.airConIdleUpRPMAdder;
      }
    }

#ifdef SD_LOGGING
    if (configPage13.onboard_log_file_rate == LOGGER_RATE_4HZ)
    {
      writeSDLogEntry();
    }
    syncSDLog(); //Sync the SD log file to the card 4 times per second.
#endif

    currentStatus.fuelPressure = getFuelPressure();
    currentStatus.oilPressure = getOilPressure();

    if (auxIsEnabled)
    {
      //TODO dazq to clean this right up :)
      //check through the Aux input channels if enabled for Can or local use
      for (byte AuxinChan = 0; AuxinChan < 16; AuxinChan++)
      {
        currentStatus.current_caninchannel = AuxinChan;

        if (((configPage9.caninput_sel[currentStatus.current_caninchannel] & 12) == 4)
            && (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 0) && (configPage9.intcan_available == 1)))
                || ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1)) &&
                    ((configPage9.caninput_sel[currentStatus.current_caninchannel] & 64) == 0))
                || ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0)))))
        { //if current input channel is enabled as external & secondary serial enabled & internal can disabled(but internal can is available)
          // or current input channel is enabled as external & secondary serial enabled & internal can enabled(and internal can is available)
          //currentStatus.canin[13] = 11;  Dev test use only!
          if (configPage9.enable_secondarySerial == 1)  // megas only support can via secondary serial
          {
            sendCancommand(2, 0, currentStatus.current_caninchannel, 0, ((configPage9.caninput_source_can_address[currentStatus.current_caninchannel] & 2047) + 0x100));
            //send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from secondarySerial
          }
        }
        else if (((configPage9.caninput_sel[currentStatus.current_caninchannel] & 12) == 4)
                 && (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1)) &&
                      ((configPage9.caninput_sel[currentStatus.current_caninchannel] & 64) == 64))
                     || ((configPage9.enable_secondarySerial == 0) && ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1)) &&
                         ((configPage9.caninput_sel[currentStatus.current_caninchannel] & 128) == 128))))
        { //if current input channel is enabled as external for canbus & secondary serial enabled & internal can enabled(and internal can is available)
          // or current input channel is enabled as external for canbus & secondary serial disabled & internal can enabled(and internal can is available)
          //currentStatus.canin[13] = 12;  Dev test use only!
#if defined(CORE_STM32) || defined(CORE_TEENSY)
          if (configPage9.enable_intcan == 1) //  if internal can is enabled
          {
            sendCancommand(3, configPage9.speeduino_tsCanId, currentStatus.current_caninchannel, 0, ((configPage9.caninput_source_can_address[currentStatus.current_caninchannel] & 2047) + 0x100));
            //send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from internal canbus
          }
#endif
        }
        else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel] & 12) == 8)
                 || (((configPage9.enable_secondarySerial == 0) && ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0))) && (configPage9.caninput_sel[currentStatus.current_caninchannel] & 3) == 2)
                 || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel] & 3) == 2)))
        { //if current input channel is enabled as analog local pin
          //read analog channel specified
          //currentStatus.canin[13] = (configPage9.Auxinpina[currentStatus.current_caninchannel]&63);  Dev test use only!127
          currentStatus.canin[currentStatus.current_caninchannel] = readAuxanalog(pinTranslateAnalog(configPage9.Auxinpina[currentStatus.current_caninchannel] & 63));
        }
        else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel] & 12) == 12)
                 || (((configPage9.enable_secondarySerial == 0) && ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0))) && (configPage9.caninput_sel[currentStatus.current_caninchannel] & 3) == 3)
                 || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel] & 3) == 3)))
        { //if current input channel is enabled as digital local pin
          //read digital channel specified
          //currentStatus.canin[14] = ((configPage9.Auxinpinb[currentStatus.current_caninchannel]&63)+1);  Dev test use only!127+1
          currentStatus.canin[currentStatus.current_caninchannel] = readAuxdigital((configPage9.Auxinpinb[currentStatus.current_caninchannel] & 63) + 1);
        } //Channel type
      } //For loop going through each channel
    } //aux channels are enabled
  } //4Hz timer

  if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ)) //Once per second)
  {
    BIT_CLEAR(TIMER_mask, BIT_TIMER_1HZ);
    //Infrequent baro readings are not an issue.
    readBaro(currentStatus.initialisationComplete);

    if (configPage10.wmiEnabled > 0 && configPage10.wmiIndicatorEnabled > 0)
    {
      // water tank empty
      if (BIT_CHECK(currentStatus.status4, BIT_STATUS4_WMI_EMPTY))
      {
        // flash with 1sec interval
        WMIIndicator.toggle();
      }
      else
      {
        WMIIndicator.write(configPage10.wmiIndicatorPolarity ? HIGH : LOW);
      }
    }

    //Check whether fuel pump priming is complete.
    fuelPriming.update(currentStatus.secl, configPage2.fpPrime);

#ifdef SD_LOGGING
    if (configPage13.onboard_log_file_rate == LOGGER_RATE_1HZ)
    {
      writeSDLogEntry();
    }
#endif

  } //1Hz timer

  if (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL
      || configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL
      || configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL)
  {
    idleControl(); //Run idlecontrol every loop for stepper idle.
  }


  //VE and advance calculation were moved outside the sync/RPM check so that the fuel and ignition load value will be accurately shown when RPM=0
  currentStatus.VE1 = getVE1();
  currentStatus.VE = currentStatus.VE1; //Set the final VE value to be VE 1 as a default. This may be changed in the section below

  currentStatus.advance1 = getAdvance1();
  currentStatus.advance = currentStatus.advance1; //Set the final advance value to be advance 1 as a default. This may be changed in the section below

  calculateSecondaryFuel();
  calculateSecondarySpark();

  //Always check for sync
  //Main loop runs within this clause
  if ((currentStatus.hasSync || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) && currentStatus.RPM > 0)
  {
    //Check whether running or cranking
    //Crank RPM in the config is stored as a x10. currentStatus.crankRPM is set
    //in timers.ino and represents the true value
    if (currentStatus.RPM > currentStatus.crankRPM)
    {
      BIT_SET(currentStatus.engine, BIT_ENGINE_RUN); //Sets the engine running bit
      //Only need to do anything if we're transitioning from cranking to running
      if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
      {
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
        if (configPage4.ignBypassEnabled > 0)
        {
          IgnBypass.on();
        }
      }
    }
    else
    {
      if (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN)
          || currentStatus.RPM < currentStatus.crankRPM - CRANK_RUN_HYSTER)
      {
        //Sets the engine cranking bit, clears the engine running bit
        BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN);
        currentStatus.runSecs = 0; //We're cranking (hopefully), so reset the engine run time to prompt ASE.
        if (configPage4.ignBypassEnabled > 0)
        {
          IgnBypass.off();
        }

        //Check whether the user has selected to disable to the fan during cranking
        if (configPage2.fanWhenCranking == 0)
        {
          FAN_OFF();
        }
      }
    }
    //END SETTING ENGINE STATUSES
    //-----------------------------------------------------------------------------------------------------

    //Begin the fuel calculation
    //Calculate an injector pulsewidth from the VE
    currentStatus.corrections = correctionsFuel();

    uint32_t injector_pulsewidth =
      calculateTotalInjectorPW(req_fuel_uS, currentStatus.VE, currentStatus.MAP, currentStatus.corrections, inj_opentime_uS);

    //Manual adder for nitrous. These are not in correctionsFuel() because they
    //are direct adders to the ms value, not % based
    if (currentStatus.nitrous_status == NITROUS_STAGE1 || currentStatus.nitrous_status == NITROUS_BOTH)
    {
      int16_t adderRange = (configPage10.n2o_stage1_maxRPM - configPage10.n2o_stage1_minRPM) * 100;
      //The percentage of the way through the RPM range
      int16_t adderPercent = ((currentStatus.RPM - (configPage10.n2o_stage1_minRPM * 100)) * 100) / adderRange;

      adderPercent = 100 - adderPercent; //Flip the percentage as we go from a higher adder to a lower adder as the RPMs rise
      injector_pulsewidth +=
        (configPage10.n2o_stage1_adderMax
         + percentage(adderPercent, (configPage10.n2o_stage1_adderMin - configPage10.n2o_stage1_adderMax))
         ) * 100; //Calculate the above percentage of the calculated ms value.
    }

    if (currentStatus.nitrous_status == NITROUS_STAGE2 || currentStatus.nitrous_status == NITROUS_BOTH)
    {
      int16_t adderRange = (configPage10.n2o_stage2_maxRPM - configPage10.n2o_stage2_minRPM) * 100;
      int16_t adderPercent = ((currentStatus.RPM - (configPage10.n2o_stage2_minRPM * 100)) * 100) / adderRange; //The percentage of the way through the RPM range
      adderPercent = 100 - adderPercent; //Flip the percentage as we go from a higher adder to a lower adder as the RPMs rise
      injector_pulsewidth +=
        (configPage10.n2o_stage2_adderMax
         + percentage(adderPercent, (configPage10.n2o_stage2_adderMin - configPage10.n2o_stage2_adderMax))
         ) * 100; //Calculate the above percentage of the calculated ms value.
    }

    int injector1StartAngle = 0;
    uint16_t injector2StartAngle = 0;
    uint16_t injector3StartAngle = 0;
    uint16_t injector4StartAngle = 0;

#if INJ_CHANNELS >= 5
    uint16_t injector5StartAngle = 0;
#endif
#if INJ_CHANNELS >= 6
    uint16_t injector6StartAngle = 0;
#endif
#if INJ_CHANNELS >= 7
    uint16_t injector7StartAngle = 0;
#endif
#if INJ_CHANNELS >= 8
    uint16_t injector8StartAngle = 0;
#endif

    //Check that the duty cycle of the chosen pulsewidth isn't too high.
    uint16_t pwLimit = calculatePWLimit();

    //Apply the pwLimit if staging is disabled and engine is not cranking
    if (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)
        && !configPage10.stagingEnabled)
    {
      if (injector_pulsewidth > pwLimit)
      {
        injector_pulsewidth = pwLimit;
      }
    }

    calculateInjectorStaging(injector_pulsewidth, pwLimit);

    //***********************************************************************************************
    //BEGIN INJECTION TIMING

    currentStatus.injAngle = table2D_getValue(&injectorAngleTable, currentStatus.RPMdiv100);
    if (currentStatus.injAngle > uint16_t(CRANK_ANGLE_MAX_INJ))
    {
      currentStatus.injAngle = uint16_t(CRANK_ANGLE_MAX_INJ);
    }

    //How many crank degrees the calculated PW will take at the current speed
    unsigned int PWdivTimerPerDegree =
      timeToAngleDegPerMicroSec(injectors.injector(injChannel1).PW, degreesPerMicro);

    injector1StartAngle =
      injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
    bool const staging_is_required =
      configPage10.stagingEnabled && BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);

    //Repeat the above for each cylinder
    switch (configPage2.nCylinders)
    {
      //Single cylinder
    case 1:
      //The only thing that needs to be done for single cylinder is to check for staging.
      if (staging_is_required)
      {
        //Need to redo this for PW2 as it will be dramatically different to PW1 when staging.
        PWdivTimerPerDegree =
          timeToAngleDegPerMicroSec(injectors.injector(injChannel2).PW, degreesPerMicro);
        injector2StartAngle =
          injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
      }
      break;

      //2 cylinders
    case 2:
      injector2StartAngle =
        injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);

      if (configPage2.injLayout == INJ_SEQUENTIAL && configPage6.fuelTrimEnabled > 0)
      {
        injectors.applyFuelTrimToPW(injChannel1, &trim1Table, currentStatus.fuelLoad, currentStatus.RPM);
        injectors.applyFuelTrimToPW(injChannel2, &trim2Table, currentStatus.fuelLoad, currentStatus.RPM);
      }
      else if (staging_is_required)
      {
        //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
        PWdivTimerPerDegree =
          timeToAngleDegPerMicroSec(injectors.injector(injChannel3).PW, degreesPerMicro);
        injector3StartAngle =
          injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
        injector4StartAngle =
          injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);

        //Phase this either 180 or 360 degrees out from inj3 (In reality this will
        //always be 180 as you can't have sequential and staged currently)
        injector4StartAngle = injector3StartAngle + (CRANK_ANGLE_MAX_INJ / 2);
        if (injector4StartAngle > (uint16_t)CRANK_ANGLE_MAX_INJ)
        {
          injector4StartAngle -= CRANK_ANGLE_MAX_INJ;
        }
      }
      break;

      //3 cylinders
    case 3:
      injector2StartAngle =
        injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
      injector3StartAngle =
        injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);

      if (configPage2.injLayout == INJ_SEQUENTIAL && configPage6.fuelTrimEnabled > 0)
      {
        injectors.applyFuelTrimToPW(injChannel1, &trim1Table, currentStatus.fuelLoad, currentStatus.RPM);
        injectors.applyFuelTrimToPW(injChannel2, &trim2Table, currentStatus.fuelLoad, currentStatus.RPM);
        injectors.applyFuelTrimToPW(injChannel3, &trim3Table, currentStatus.fuelLoad, currentStatus.RPM);

#if INJ_CHANNELS >= 6
        if (staging_is_required)
        {
          //Need to redo this for PW4 as it will be dramatically different to PW1 when staging
          PWdivTimerPerDegree =
            timeToAngleDegPerMicroSec(injectors.injector(injChannel4).PW, degreesPerMicro);
          injector4StartAngle =
            injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
          injector5StartAngle =
            injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
          injector6StartAngle =
            injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
        }
#endif
      }
      else if (staging_is_required)
      {
        //Need to redo this for PW4 as it will be dramatically different to PW1 when staging
        PWdivTimerPerDegree =
          timeToAngleDegPerMicroSec(injectors.injector(injChannel4).PW, degreesPerMicro);
        injector4StartAngle =
          injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
#if INJ_CHANNELS >= 6
        injector5StartAngle =
          injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
        injector6StartAngle =
          injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
#endif
      }
      break;

      //4 cylinders
    case 4:
      //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
      injector2StartAngle =
        injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);

      if ((configPage2.injLayout == INJ_SEQUENTIAL) && currentStatus.hasSync)
      {
        if (CRANK_ANGLE_MAX_INJ != 720)
        {
          changeHalfToFullSync();
        }

        injector3StartAngle =
          injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
        injector4StartAngle =
          injectors.calculateInjectorStartAngle(injChannel4, PWdivTimerPerDegree, currentStatus.injAngle);
#if INJ_CHANNELS >= 8
        if (staging_is_required)
        {
          //Need to redo this for PW5 as it will be dramatically different to PW1 when staging
          PWdivTimerPerDegree =
            timeToAngleDegPerMicroSec(injectors.injector(injChannel5).PW, degreesPerMicro);
          injector5StartAngle =
            injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
          injector6StartAngle =
            injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
          injector7StartAngle =
            injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
          injector8StartAngle =
            injectors.calculateInjectorStartAngle(injChannel4, PWdivTimerPerDegree, currentStatus.injAngle);
        }
#endif

        if (configPage6.fuelTrimEnabled > 0)
        {
          injectors.applyFuelTrimToPW(injChannel1, &trim1Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel2, &trim2Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel3, &trim3Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel4, &trim4Table, currentStatus.fuelLoad, currentStatus.RPM);
        }
      }
      else if (staging_is_required)
      {
        //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
        PWdivTimerPerDegree =
          timeToAngleDegPerMicroSec(injectors.injector(injChannel3).PW, degreesPerMicro);
        injector3StartAngle =
          injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
        injector4StartAngle =
          injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
      }
      else
      {
        if (BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && CRANK_ANGLE_MAX_INJ != 360)
        {
          changeFullToHalfSync();
        }
      }
      break;

      //5 cylinders
    case 5:
      injector2StartAngle =
        injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
      injector3StartAngle =
        injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
      injector4StartAngle =
        injectors.calculateInjectorStartAngle(injChannel4, PWdivTimerPerDegree, currentStatus.injAngle);
#if INJ_CHANNELS >= 5
      injector5StartAngle =
        injectors.calculateInjectorStartAngle(injChannel5, PWdivTimerPerDegree, currentStatus.injAngle);
#endif

      //Staging is possible by using the 6th channel if available
#if INJ_CHANNELS >= 6
      if (staging_is_required)
      {
        PWdivTimerPerDegree =
          timeToAngleDegPerMicroSec(injectors.injector(injChannel6).PW, degreesPerMicro);
        injector6StartAngle =
          injectors.calculateInjectorStartAngle(injChannel6, PWdivTimerPerDegree, currentStatus.injAngle);
      }
#endif

      break;

      //6 cylinders
    case 6:
      injector2StartAngle =
        injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
      injector3StartAngle =
        injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);

#if INJ_CHANNELS >= 6
      if (configPage2.injLayout == INJ_SEQUENTIAL && currentStatus.hasSync)
      {
        if (CRANK_ANGLE_MAX_INJ != 720)
        {
          changeHalfToFullSync();
        }

        injector4StartAngle =
          injectors.calculateInjectorStartAngle(injChannel4, PWdivTimerPerDegree, currentStatus.injAngle);
        injector5StartAngle =
          injectors.calculateInjectorStartAngle(injChannel5, PWdivTimerPerDegree, currentStatus.injAngle);
        injector6StartAngle =
          injectors.calculateInjectorStartAngle(injChannel6, PWdivTimerPerDegree, currentStatus.injAngle);

        if (configPage6.fuelTrimEnabled > 0)
        {
          injectors.applyFuelTrimToPW(injChannel1, &trim1Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel2, &trim2Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel3, &trim3Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel4, &trim4Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel5, &trim5Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel6, &trim6Table, currentStatus.fuelLoad, currentStatus.RPM);
        }

        //Staging is possible with sequential on 8 channel boards by using outputs 7 + 8 for the staged injectors
#if INJ_CHANNELS >= 8
        if (staging_is_required)
        {
          //Need to redo this for staging PW as it will be dramatically different to PW1 when staging
          PWdivTimerPerDegree =
            timeToAngleDegPerMicroSec(injectors.injector(injChannel7).PW, degreesPerMicro);
          injector7StartAngle =
            injectors.calculateInjectorStartAngle(injChannel7, PWdivTimerPerDegree, currentStatus.injAngle);

          PWdivTimerPerDegree =
            timeToAngleDegPerMicroSec(injectors.injector(injChannel8).PW, degreesPerMicro);
          injector8StartAngle =
            injectors.calculateInjectorStartAngle(injChannel8, PWdivTimerPerDegree, currentStatus.injAngle);
        }
#endif
      }
      else
      {
        if (BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && CRANK_ANGLE_MAX_INJ != 360)
        {
          changeFullToHalfSync();
        }

        if (staging_is_required)
        {
          //Need to redo this for staging PW as it will be dramatically different to PW1 when staging
          PWdivTimerPerDegree =
            timeToAngleDegPerMicroSec(injectors.injector(injChannel4).PW, degreesPerMicro);
          injector4StartAngle =
            injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
          injector5StartAngle =
            injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
          injector6StartAngle =
            injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
        }
      }
#         endif
      break;

      //8 cylinders
    case 8:
      injector2StartAngle =
        injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
      injector3StartAngle =
        injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
      injector4StartAngle =
        injectors.calculateInjectorStartAngle(injChannel4, PWdivTimerPerDegree, currentStatus.injAngle);

#if INJ_CHANNELS >= 8
      if (configPage2.injLayout == INJ_SEQUENTIAL && currentStatus.hasSync)
      {
        if (CRANK_ANGLE_MAX_INJ != 720)
        {
          changeHalfToFullSync();
        }

        injector5StartAngle =
          injectors.calculateInjectorStartAngle(injChannel5, PWdivTimerPerDegree, currentStatus.injAngle);
        injector6StartAngle =
          injectors.calculateInjectorStartAngle(injChannel6, PWdivTimerPerDegree, currentStatus.injAngle);
        injector7StartAngle =
          injectors.calculateInjectorStartAngle(injChannel7, PWdivTimerPerDegree, currentStatus.injAngle);
        injector8StartAngle =
          injectors.calculateInjectorStartAngle(injChannel8, PWdivTimerPerDegree, currentStatus.injAngle);

        if (configPage6.fuelTrimEnabled > 0)
        {
          injectors.applyFuelTrimToPW(injChannel1, &trim1Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel2, &trim2Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel3, &trim3Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel4, &trim4Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel5, &trim5Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel6, &trim6Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel7, &trim7Table, currentStatus.fuelLoad, currentStatus.RPM);
          injectors.applyFuelTrimToPW(injChannel8, &trim8Table, currentStatus.fuelLoad, currentStatus.RPM);
        }
      }
      else
      {
        if (BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && CRANK_ANGLE_MAX_INJ != 360)
        {
          changeFullToHalfSync();
        }

        if (staging_is_required)
        {
          //Need to redo this for PW5 as it will be dramatically different to PW1 when staging
          PWdivTimerPerDegree =
            timeToAngleDegPerMicroSec(injectors.injector(injChannel5).PW, degreesPerMicro);
          injector5StartAngle =
            injectors.calculateInjectorStartAngle(injChannel1, PWdivTimerPerDegree, currentStatus.injAngle);
          injector6StartAngle =
            injectors.calculateInjectorStartAngle(injChannel2, PWdivTimerPerDegree, currentStatus.injAngle);
          injector7StartAngle =
            injectors.calculateInjectorStartAngle(injChannel3, PWdivTimerPerDegree, currentStatus.injAngle);
          injector8StartAngle =
            injectors.calculateInjectorStartAngle(injChannel4, PWdivTimerPerDegree, currentStatus.injAngle);
        }
      }

#endif
      break;

      //Will hit the default case on 1 cylinder or >8 cylinders. Do nothing in these cases
    default:
      break;
    }

    //***********************************************************************************************
    //| BEGIN IGNITION CALCULATIONS

    //Set dwell
    //Dwell is stored as ms * 10. ie Dwell of 4.3ms would be 43 in configPage4.
    //This number therefore needs to be multiplied by 100 to get dwell in uS
    unsigned const dwell_multiplier = 100;

    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      //use cranking dwell
      currentStatus.dwell =  (configPage4.dwellCrank * dwell_multiplier);
    }
    else
    {
      if (configPage2.useDwellMap)
      {
        //use running dwell from map
        currentStatus.dwell = (get3DTableValue(&dwellTable, currentStatus.ignLoad, currentStatus.RPM) * dwell_multiplier);
      }
      else
      {
        //use fixed running dwell
        currentStatus.dwell =  (configPage4.dwellRun * dwell_multiplier);
      }
    }
    currentStatus.dwell = correctionsDwell(currentStatus.dwell);

    //Convert the dwell time to dwell angle based on the current engine speed
    int dwellAngle = timeToAngleDegPerMicroSec(currentStatus.dwell, degreesPerMicro);

    calculateIgnitionAngles(dwellAngle);

    //If ignition timing is being tracked per tooth, perform the calcs to get the end teeth
    //This only needs to be run if the advance figure has changed, otherwise the end teeth will still be the same
    if (configPage2.perToothIgn)
    {
      triggerSetEndTeeth();
    }

    //***********************************************************************************************
    //| BEGIN FUEL SCHEDULES
    //Finally calculate the time (uS) until we reach the firing angles and set the schedules
    //We only need to set the shcedule if we're BEFORE the open angle
    //This may potentially be called a number of times as we get closer and closer to the opening time

    //Determine the current crank angle
    int crankAngle = injectorLimits(getCrankAngle());

    //Check for any of the engine protections or rev limiters being turned on
    uint16_t maxAllowedRPM = checkRevLimit(); //The maximum RPM allowed by all the potential limiters (Engine protection, 2-step, flat shift etc). Divided by 100. `checkRevLimit()` returns the current maximum RPM allow (divided by 100) based on either the fixed hard limit or the current coolant temp
    //Check each of the functions that has an RPM limit. Update the max allowed RPM if the function is active and has a lower RPM than already set
    if (checkEngineProtect() && (configPage4.engineProtectMaxRPM < maxAllowedRPM))
    {
      maxAllowedRPM = configPage4.engineProtectMaxRPM;
    }
    if (currentStatus.launchingHard && configPage6.lnchHardLim < maxAllowedRPM)
    {
      maxAllowedRPM = configPage6.lnchHardLim;
    }
    maxAllowedRPM = maxAllowedRPM * 100; //All of the above limits are divided by 100, convert back to RPM
    if (currentStatus.flatShiftingHard && currentStatus.clutchEngagedRPM < maxAllowedRPM)
    {
      //Flat shifting is a special case as the RPM limit is based on when the clutch was engaged.
      //It is not divided by 100 as it is set with the actual RPM
      maxAllowedRPM = currentStatus.clutchEngagedRPM;
    }

    if (configPage2.hardCutType == HARD_CUT_FULL && currentStatus.RPM > maxAllowedRPM)
    {
      //Full hard cut turns outputs off completely.
      switch (configPage6.engineProtectType)
      {
      case PROTECT_CUT_OFF:
        //Make sure all channels are turned on
        ignitions.setAllOn();
        injectors.setAllOn();

        currentStatus.engineProtectStatus = 0;
        break;

      case PROTECT_CUT_IGN:
        ignitions.setAllOff();
        break;

      case PROTECT_CUT_FUEL:
        injectors.setAllOff();
        break;

      case PROTECT_CUT_BOTH:
      default:
        ignitions.setAllOff();
        injectors.setAllOff();
        break;
      }
    } //Hard cut check
    else if (configPage2.hardCutType == HARD_CUT_ROLLING
             && currentStatus.RPM > (maxAllowedRPM + configPage15.rollingProtRPMDelta[0] * 10))
    {
      //Limit for rolling is the max allowed RPM minus the lowest value in the
      //delta table (Delta values are negative!)
      uint8_t revolutionsToCut = 1;

      if (configPage2.strokes == FOUR_STROKE) //4 stroke needs to cut for at least 2 revolutions
      {
        revolutionsToCut *= 2;
      }
      //4 stroke and non-sequential will cut for 4 revolutions minimum.
      //This is to ensure no half fuel ignition cycles take place
      if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL || configPage2.injLayout != INJ_SEQUENTIAL)
      {
        revolutionsToCut *= 2;
      }

      if (rollingCutLastRev == 0) //First time check
      {
        rollingCutLastRev = currentStatus.startRevolutions;
      }

      //If current RPM is over the max allowed RPM always cut,
      //otherwise check if the required number of revolutions have passed since
      //the last cut
      if (currentStatus.startRevolutions >= rollingCutLastRev + revolutionsToCut
          || currentStatus.RPM > maxAllowedRPM)
      {
        uint8_t cutPercent = 0;
        int16_t rpmDelta = currentStatus.RPM - maxAllowedRPM;
        if (rpmDelta >= 0) //If the current RPM is over the max allowed RPM then cut is full (100%)
        {
          cutPercent = 100;
        }
        else //
        {
          cutPercent = table2D_getValue(&rollingCutTable, (rpmDelta / 10));
        }

        for (uint8_t x = 0; x < max(ignitions.maxOutputs, injectors.maxOutputs); x++)
        {
          if (cutPercent == 100 || random1to100() < cutPercent)
          {
            switch (configPage6.engineProtectType)
            {
            case PROTECT_CUT_OFF:
              //Make sure all channels are turned on
              ignitions.setAllOn();
              injectors.setAllOn();
              break;

            case PROTECT_CUT_IGN:
              ignitions.setOff((ignitionChannelID_t)x);
              disablePendingIgnSchedule(x);
              break;

            case PROTECT_CUT_FUEL:
              injectors.setOff((injectorChannelID_t)x);
              disablePendingFuelSchedule(x);
              break;

            case PROTECT_CUT_BOTH:
            default:
              ignitions.setOff((ignitionChannelID_t)x);
              injectors.setOff((injectorChannelID_t)x);
              disablePendingFuelSchedule(x);
              disablePendingIgnSchedule(x);
              break;
            }
          }
          else
          {
            //Turn fuel and ignition channels on

            //Special case for non-sequential, 4-stroke where both fuel and ignition are cut. The ignition pulses should wait 1 cycle after the fuel channels are turned back on before firing again
            if (revolutionsToCut == 4 //4 stroke and non-sequential
                && !injectors.isOperational((injectorChannelID_t)x)
                && configPage6.engineProtectType == PROTECT_CUT_BOTH //Both fuel and ignition are cut
               )
            {
              //Fuel on this channel is currently off, meaning it is the first
              //revolution after a cut
              //Set this ignition channel as pending
              BIT_SET(ignitions.channelsPending, x);
            }
            else
            {
              //Turn on this ignition channel
              ignitions.setOn((ignitionChannelID_t)x);
            }

            injectors.setOn((injectorChannelID_t)x);
          }
        }
        rollingCutLastRev = currentStatus.startRevolutions;
      }

      //Check whether there are any ignition channels that are waiting for
      //injection pulses to occur before being turned back on.
      //This can only occur when at least 2 revolutions have taken place since
      //the fuel was turned back on.
      //Note that ignitions.channelsPending can only be >0 on 4 stroke,
      //non-sequential fuel when protect type is Both.
      if (ignitions.channelsPending > 0
          && currentStatus.startRevolutions >= rollingCutLastRev + 2)
      {
        ignitions.setChannelsOnMask(injectors.channelsOnMask());
        ignitions.channelsPending = 0;
      }
    } //Rolling cut check
    else
    {
      currentStatus.engineProtectStatus = 0;
      //No engine protection active, so turn all the channels on
      if (currentStatus.startRevolutions >= configPage4.StgCycles)
      {
        //Enable the fuel and ignition, assuming staging revolutions are complete
        ignitions.setAllOn();
        injectors.setAllOn();
      }
    }

#if INJ_CHANNELS >= 1
    injectors.applyInjectorControl(injChannel1, inj_opentime_uS, injector1StartAngle, crankAngle);
#endif

    /*-----------------------------------------------------------------------------------------
    | A Note on tempCrankAngle and tempStartAngle:
    |   The use of tempCrankAngle/tempStartAngle is described below.
    |   It is then used in the same way for channels 2, 3 and 4+ on both injectors and ignition
    |   Essentially, these 2 variables are used to realign the current crank angle
    |   and the desired start angle around 0 degrees for the given cylinder/output.
    |   Eg: If cylinder 2 TDC is 180 degrees after cylinder 1 (Eg a standard 4 cylinder engine),
    |   then tempCrankAngle is 180* less than the current crank angle and
    |   tempStartAngle is the desired open time less 180*.
    |   Thus the cylinder is being treated relative to its own TDC, regardless of its offset
    |
    |   This is done to avoid problems with very short or very long times until tempStartAngle.
    |------------------------------------------------------------------------------------------
    */
#if INJ_CHANNELS >= 2
    injectors.applyInjectorControl(injChannel2, inj_opentime_uS, injector2StartAngle, crankAngle);
#endif

#if INJ_CHANNELS >= 3
    injectors.applyInjectorControl(injChannel3, inj_opentime_uS, injector3StartAngle, crankAngle);
#endif

#if INJ_CHANNELS >= 4
    injectors.applyInjectorControl(injChannel4, inj_opentime_uS, injector4StartAngle, crankAngle);
#endif

#if INJ_CHANNELS >= 5
    injectors.applyInjectorControl(injChannel5, inj_opentime_uS, injector5StartAngle, crankAngle);
#endif

#if INJ_CHANNELS >= 6
    injectors.applyInjectorControl(injChannel6, inj_opentime_uS, injector6StartAngle, crankAngle);
#endif

#if INJ_CHANNELS >= 7
    injectors.applyInjectorControl(injChannel7, inj_opentime_uS, injector7StartAngle, crankAngle);
#endif

#if INJ_CHANNELS >= 8
    injectors.applyInjectorControl(injChannel8, inj_opentime_uS, injector8StartAngle, crankAngle);
#endif

    //***********************************************************************************************
    //| BEGIN IGNITION SCHEDULES
    //Same as above, except for ignition

    //fixedCrankingOverride is used to extend the dwell during cranking so that
    //the decoder can trigger the spark upon seeing a certain tooth.
    //Currently only available on the basic distributor and 4g63 decoders.
    if (configPage4.ignCranklock
        && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)
        && BIT_CHECK(decoderState, BIT_DECODER_HAS_FIXED_CRANKING))
    {
      fixedCrankingOverride = currentStatus.dwell * 3;
      //This is a safety step to prevent the ignition start time occurring AFTER
      //the target tooth pulse has already occurred.
      //It simply moves the start time forward a little, which is compensated
      //for by the increase in the dwell time
      if (currentStatus.RPM < 250)
      {
        ignitions.adjustStartAngle(-5);
      }
    }
    else
    {
      fixedCrankingOverride = 0;
    }

    if (ignitions.channelsOn != 0)
    {
      //Refresh the current crank angle info
      crankAngle = ignitionLimits(getCrankAngle());

      ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);

#if IGN_CHANNELS >= 1
      ignitions.applyIgnitionControl(ignChannel1, crankAngle);
#endif

#if defined(USE_IGN_REFRESH)
      if (ignition1.ignitionSchedule->Status == RUNNING
          && ignition1.endAngle > crankAngle
          && configPage4.StgCycles == 0
          && !configPage2.perToothIgn)
      {
        unsigned long uSToEnd = 0;

        //Refresh the crank angle info
        crankAngle = ignitionLimits(getCrankAngle());

        //ONLY ONE OF THE BELOW SHOULD BE USED (PROBABLY THE FIRST):
        //*********
        if (ignition1.endAngle > crankAngle)
        {
          uSToEnd = angleToTimeMicroSecPerDegree(ignition1.endAngle - crankAngle);
        }
        else
        {
          uSToEnd = angleToTimeMicroSecPerDegree(360 + ignition1.endAngle - crankAngle);
        }
        //*********
        //uSToEnd = ((ignition1.endAngle - crankAngle) * (toothLastToothTime - toothLastMinusOneToothTime)) / triggerToothAngle;
        //*********

        refreshIgnitionSchedule1(uSToEnd + fixedCrankingOverride);
      }
# endif

#if IGN_CHANNELS >= 2
      ignitions.applyIgnitionControl(ignChannel2, crankAngle);
#endif

#if IGN_CHANNELS >= 3
      ignitions.applyIgnitionControl(ignChannel3, crankAngle);
#endif

#if IGN_CHANNELS >= 4
      ignitions.applyIgnitionControl(ignChannel4, crankAngle);
#endif

#if IGN_CHANNELS >= 5
      ignitions.applyIgnitionControl(ignChannel5, crankAngle);
#endif

#if IGN_CHANNELS >= 6
      ignitions.applyIgnitionControl(ignChannel6, crankAngle);
#endif

#if IGN_CHANNELS >= 7
      ignitions.applyIgnitionControl(ignChannel7, crankAngle);
#endif

#if IGN_CHANNELS >= 8
      ignitions.applyIgnitionControl(ignChannel8, crankAngle);
#endif

    } //Ignition schedules on

    if (!BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT)
        && resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING)
    {
      //Reset prevention is supposed to be on while the engine is running but isn't. Fix that.
      digitalWrite(pinResetControl, HIGH);
      BIT_SET(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
    }
  } //Has sync and RPM
  else if (BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT)
           && resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING)
  {
    digitalWrite(pinResetControl, LOW);
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
  }
} //loop()
#endif //Unit test guard

/**
 * @brief This function calculates the required pulsewidth time (in us) given the current system state
 *
 * @param REQ_FUEL The required fuel value in uS, as calculated by TunerStudio
 * @param VE Lookup from the main fuel table. This can either have been MAP or TPS based, depending on the algorithm used
 * @param MAP In KPa, read from the sensor (This is used when performing a multiply of the map only. It is applicable in both Speed density and Alpha-N)
 * @param corrections Sum of Enrichment factors (Cold start, acceleration). This is a multiplication factor (Eg to add 10%, this should be 110)
 * @param injOpen Injector opening time. The time the injector takes to open minus the time it takes to close (Both in uS)
 * @return uint16_t The injector pulse width in uS
 */
uint16_t calculateTotalInjectorPW(int REQ_FUEL, byte VE, long MAP, uint16_t corrections, int injOpen)
{
  //Standard float version of the calculation
  //return (REQ_FUEL * (float)(VE/100.0) * (float)(MAP/100.0) * (float)(TPS/100.0) * (float)(corrections/100.0) + injOpen);
  //Note: The MAP and TPS portions are currently disabled, we use VE and corrections only
  uint16_t iVE, iCorrections;
  uint16_t iMAP = 100;
  uint16_t iAFR = 147;

  //100% float free version, does sacrifice a little bit of accuracy, but not much.

  //If corrections are huge, use less bitshift to avoid overflow.
  //Sacrifices a bit more accuracy (basically only during very cold temp cranking)
  byte bitShift = 7;
  if (corrections > 511)
  {
    bitShift = 6;
  }
  if (corrections > 1023)
  {
    bitShift = 5;
  }

  iVE = div100((uint16_t)VE << 7);

  //Check whether either of the multiply MAP modes is turned on
  if (configPage2.multiplyMAP == MULTIPLY_MAP_MODE_100)
  {
    iMAP = div100((uint16_t)MAP << 7U);
  }
  else if (configPage2.multiplyMAP == MULTIPLY_MAP_MODE_BARO)
  {
    iMAP = ((unsigned int)MAP << 7) / currentStatus.baro;
  }

  if (configPage2.includeAFR
      && configPage6.egoType == EGO_TYPE_WIDE
      && currentStatus.runSecs > configPage6.ego_sdelay)
  {
    //Include AFR (vs target) if enabled
    iAFR = ((unsigned int)currentStatus.O2 << 7) / currentStatus.afrTarget;
  }
  if (configPage2.incorporateAFR && !configPage2.includeAFR)
  {
    //Incorporate stoich vs target AFR, if enabled.
    iAFR = ((unsigned int)configPage2.stoich << 7) / currentStatus.afrTarget;
  }
  iCorrections = div100(corrections << bitShift);


  //Need to use an intermediate value to avoid overflowing the long
  uint32_t intermediate = ((uint32_t)REQ_FUEL * (uint32_t)iVE) >> 7;
  if (configPage2.multiplyMAP > 0)
  {
    intermediate = (intermediate * (uint32_t)iMAP) >> 7;
  }

  if (configPage2.includeAFR
      && configPage6.egoType == EGO_TYPE_WIDE
      && currentStatus.runSecs > configPage6.ego_sdelay)
  {
    //EGO type must be set to wideband and the AFR warmup time must've elapsed for this to be used
    intermediate = (intermediate * (uint32_t)iAFR) >> 7;
  }
  if (configPage2.incorporateAFR && !configPage2.includeAFR)
  {
    intermediate = (intermediate * (uint32_t)iAFR) >> 7;
  }

  intermediate = (intermediate * (uint32_t)iCorrections) >> bitShift;
  if (intermediate != 0)
  {
    //If intermediate is not 0, we need to add the opening time
    //(0 typically indicates that one of the full fuel cuts is active)
    intermediate += injOpen; //Add the injector opening time
    //AE calculation only when ACC is active.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC))
    {
      //AE Adds % of req_fuel
      if (configPage2.aeApplyMode == AE_MODE_ADDER)
      {
        intermediate += div100(((uint32_t)REQ_FUEL) * (currentStatus.AEamount - 100U));
      }
    }

    //Make sure this won't overflow when we convert to uInt.
    //This means the maximum pulsewidth possible is 65.535mS
    if (intermediate > UINT16_MAX)
    {
      intermediate = UINT16_MAX;
    }
  }

  return intermediate;
}

/** Lookup the current VE value from the primary 3D fuel map.
 * The Y axis value used for this lookup varies based on the fuel algorithm selected (speed density, alpha-n etc).
 *
 * @return byte The current VE value
 */
byte getVE1(void)
{
  currentStatus.fuelLoad = calculate_engine_load((load_source_t)configPage2.fuelAlgorithm, currentStatus);
  //Perform lookup into fuel map for RPM vs MAP value
  byte tempVE = get3DTableValue(&fuelTable, currentStatus.fuelLoad, currentStatus.RPM);

  return tempVE;
}

/** Lookup the ignition advance from 3D ignition table.
 * The values used to look this up will be RPM and whatever load source the user has configured.
 *
 * @return byte The current target advance value in degrees
 */
byte getAdvance1(void)
{
  currentStatus.ignLoad = calculate_engine_load((load_source_t)configPage2.ignAlgorithm, currentStatus);
  //As for VE, but for ignition advance
  byte tempAdvance = get3DTableValue(&ignitionTable, currentStatus.ignLoad, currentStatus.RPM) - OFFSET_IGNITION;
  tempAdvance = correctionsIgn(tempAdvance);

  return tempAdvance;
}

/** Calculate the Ignition angles for all cylinders (based on @ref config2.nCylinders).
 * both start and end angles are calculated for each channel.
 * Also the mode of ignition firing - wasted spark vs. dedicated spark per cyl. - is considered here.
 */
void calculateIgnitionAngles(int dwellAngle)
{
  //This test for more cylinders and do the same thing
  switch (configPage2.nCylinders)
  {
    //1 cylinder
  case 1:
    ignitions.ignition(ignChannel1).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    break;

    //2 cylinders
  case 2:
    ignitions.ignition(ignChannel1).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel2).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    break;

    //3 cylinders
  case 3:
    ignitions.ignition(ignChannel1).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel2).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel3).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    break;

    //4 cylinders
  case 4:
    ignitions.ignition(ignChannel1).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel2).calculateIgnitionAngle(dwellAngle, currentStatus.advance);

#if IGN_CHANNELS >= 4
    if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && currentStatus.hasSync)
    {
      if (CRANK_ANGLE_MAX_IGN != 720)
      {
        changeHalfToFullSync();
      }

      ignitions.ignition(ignChannel3).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
      ignitions.ignition(ignChannel4).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    }
    else if (configPage4.sparkMode == IGN_MODE_ROTARY)
    {
      byte splitDegrees = 0;
      splitDegrees = table2D_getValue(&rotarySplitTable, currentStatus.ignLoad);

      //The trailing angles are set relative to the leading ones
      calculateIgnitionTrailingRotary(
        dwellAngle, splitDegrees,
        ignitions.ignition(ignChannel1).endAngle,
        &ignitions.ignition(ignChannel3).endAngle,
        &ignitions.ignition(ignChannel3).startAngle);
      calculateIgnitionTrailingRotary(
        dwellAngle, splitDegrees,
        ignitions.ignition(ignChannel2).endAngle,
        &ignitions.ignition(ignChannel4).endAngle,
        &ignitions.ignition(ignChannel4).startAngle);
    }
    else
    {
      if (BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && CRANK_ANGLE_MAX_IGN != 360)
      {
        changeFullToHalfSync();
      }
    }
#endif
    break;

    //5 cylinders
  case 5:
    ignitions.ignition(ignChannel1).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel2).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel3).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel4).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
#if (IGN_CHANNELS >= 5)
    ignitions.ignition(ignChannel5).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
#endif
    break;

    //6 cylinders
  case 6:
    ignitions.ignition(ignChannel1).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel2).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel3).calculateIgnitionAngle(dwellAngle, currentStatus.advance);

#if IGN_CHANNELS >= 6
    if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && currentStatus.hasSync)
    {
      if (CRANK_ANGLE_MAX_IGN != 720)
      {
        changeHalfToFullSync();
      }

      ignitions.ignition(ignChannel4).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
      ignitions.ignition(ignChannel5).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
      ignitions.ignition(ignChannel6).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    }
    else
    {
      if (BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && CRANK_ANGLE_MAX_IGN != 360)
      {
        changeFullToHalfSync();
      }
    }
#endif
    break;

    //8 cylinders
  case 8:
    ignitions.ignition(ignChannel1).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel2).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel3).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    ignitions.ignition(ignChannel4).calculateIgnitionAngle(dwellAngle, currentStatus.advance);

#if IGN_CHANNELS >= 8
    if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && currentStatus.hasSync)
    {
      if (CRANK_ANGLE_MAX_IGN != 720)
      {
        changeHalfToFullSync();
      }

      ignitions.ignition(ignChannel5).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
      ignitions.ignition(ignChannel6).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
      ignitions.ignition(ignChannel7).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
      ignitions.ignition(ignChannel8).calculateIgnitionAngle(dwellAngle, currentStatus.advance);
    }
    else
    {
      if (BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && CRANK_ANGLE_MAX_IGN != 360)
      {
        changeFullToHalfSync();
      }
    }
#endif
    break;

  default:
    //Will hit the default case on other cylinder counts. Do nothing in these cases.
    break;
  }
}

uint16_t calculatePWLimit(void)
{
  //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by
  //the total time it takes to perform 1 revolution
  uint32_t tempLimit = percentage(configPage2.dutyLim, revolutionTime);

  //Handle multiple squirts per rev
  if (configPage2.strokes == FOUR_STROKE)
  {
    tempLimit = tempLimit * 2;
  }

  //Optimise for power of two divisions where possible
  switch(currentStatus.nSquirts)
  {
    case 1:
      //No action needed
      break;
    case 2:
      tempLimit = tempLimit / 2;
      break;
    case 4:
      tempLimit = tempLimit / 4;
      break;
    case 8:
      tempLimit = tempLimit / 8;
      break;
    default:
      //Non-PoT squirts value. Perform (slow) uint32_t division
      tempLimit = tempLimit / currentStatus.nSquirts;
      break;
  }
  if(tempLimit > UINT16_MAX)
  {
    tempLimit = UINT16_MAX;
  }

  return tempLimit;
}

void checkLaunchAndFlatShift(void)
{
  //Check for launching/flat shift (clutch) based on the current and previous clutch states
  currentStatus.previousClutchTrigger = currentStatus.clutchTrigger;
  //Only check for pinLaunch if any function using it is enabled. Else pins might break starting a board
  if (configPage6.flatSEnable || configPage6.launchEnabled)
  {
    currentStatus.clutchTrigger = (configPage6.launchHiLo == 0) ^ digitalRead(pinLaunch);
  }

  //Check whether the clutch has been engaged or disengaged and store the
  //current RPM if so.
  if (currentStatus.clutchTrigger
      && currentStatus.previousClutchTrigger != currentStatus.clutchTrigger)
  {
    currentStatus.clutchEngagedRPM = currentStatus.RPM;
  }

  //Default flags to off
  currentStatus.launchingHard = false;
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_HLAUNCH);
  currentStatus.flatShiftingHard = false;

  if (configPage6.launchEnabled
      && currentStatus.clutchTrigger
      && currentStatus.clutchEngagedRPM < (unsigned int)configPage6.flatSArm * 100
      && currentStatus.TPS >= configPage10.lnchCtrlTPS)
  {
    //Check whether RPM is above the launch limit
    uint16_t launchRPMLimit = configPage6.lnchHardLim * 100;
    //Add the rolling cut delta if enabled (Delta is a negative value)
    if (configPage2.hardCutType == HARD_CUT_ROLLING)
    {
      launchRPMLimit += configPage15.rollingProtRPMDelta[0] * 10;
    }

    if (currentStatus.RPM > launchRPMLimit)
    {
      //HardCut rev limit for 2-step launch control.
      currentStatus.launchingHard = true;
      BIT_SET(currentStatus.spark, BIT_SPARK_HLAUNCH);
    }
  }
  else
  {
    //If launch is not active, check whether flat shift should be active
    if (configPage6.flatSEnable
        && currentStatus.clutchTrigger
        && currentStatus.clutchEngagedRPM >= (unsigned int)configPage6.flatSArm * 100)
    {
      uint16_t flatRPMLimit = currentStatus.clutchEngagedRPM;

      //Add the rolling cut delta if enabled (Delta is a negative value)
      if (configPage2.hardCutType == HARD_CUT_ROLLING)
      {
        flatRPMLimit += configPage15.rollingProtRPMDelta[0] * 10;
      }

      if (currentStatus.RPM > flatRPMLimit)
      {
        //Flat shift rev limit
        currentStatus.flatShiftingHard = true;
      }
    }
  }
}
