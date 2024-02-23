/** @file
 * Speeduino Initialisation (called at Arduino setup()).
 */
#include "globals.h"
#include "init.h"
#include "storage.h"
#include "updates.h"
#include "speeduino.h"
#include "timers.h"
#include "comms_secondary.h"
#include "comms_CAN.h"
#include "utilities.h"
#include "ignition_schedule.h"
#include "ignition_control.h"
#include "injector_control.h"
#include "scheduler.h"
#include "schedule_calcs.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "decoders/null_trigger.h"
#include "corrections.h"
#include "idle.h"
#include "table2d.h"
#include "acc_mc33810.h"
#include BOARD_H //Note that this is not a real file, it is defined in globals.h.
#if defined(EEPROM_RESET_PIN)
  #include EEPROM_LIB_H
#endif
#ifdef SD_LOGGING
  #include "SD_logger.h"
  #include "rtc_common.h"
#endif
#include "board_ids.h"
#include "injector_contexts.h"
#include "ignition_contexts.h"
#include "fuel_pump.h"
#include "auxiliary_pins.h"
#include "ignition_pins.h"
#include "injector_pins.h"
#include "bit_macros.h"

static uint16_t req_fuel_init_uS = 0; /**< The original value of req_fuel_uS to reference when changing to/from half sync. */

/** Initialise Speeduino for the main loop.
 * Top level init entry point for all initialisations:
 * - Initialise and set sizes of 3D tables
 * - Load config from EEPROM, update config structures to current version of SW if needed.
 * - Initialise board (The initBoard() is for board X implemented in board_X.ino file)
 * - Initialise timers (See timers.ino)
 * - Perform optional SD card and RTC battery inits
 * - Load calibration tables from EEPROM
 * - Perform pin mapping (calling @ref setPinMapping() based on @ref config2.pinMapping)
 * - Stop any coil charging and close injectors
 * - Initialise schedulers, Idle, Fan, auxPWM, Corrections, AD-conversions, Programmable I/O
 * - Initialise baro (ambient pressure) by reading MAP (before engine runs)
 * - Initialise triggers (by @ref initialiseTriggers() )
 * - Perform cyl. count based initialisations (@ref config2.nCylinders)
 * - Perform injection and spark mode based setup
 *   - Assign injector open/close and coil charge begin/end functions to their dedicated global vars
 * - Perform fuel pressure priming by turning fuel pump on
 * - Read CLT and TPS sensors to have cranking pulsewidths computed correctly
 * - Mark Initialisation completed (this flag-marking is used in code to prevent after-init changes)
 */
void initialiseAll(void)
{
  currentStatus.injPrimed = false;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if defined(CORE_STM32)
  // device has internal canbus
  configPage9.intcan_available = 1;
  //STM32 can not currently enabled
#endif

/*
***********************************************************************************************************
* EEPROM reset
*/
#if defined(EEPROM_RESET_PIN)
  uint32_t start_time = millis();
  byte exit_erase_loop = false;
  pinMode(EEPROM_RESET_PIN, INPUT_PULLUP);

  // only start routine when this pin is low because it is pulled low
  while (digitalRead(EEPROM_RESET_PIN) != HIGH && (millis() - start_time) < 1050)
  {
    // make sure the key is pressed for at least 0.5 second
    if ((millis() - start_time) > 500)
    {
      // if key is pressed afterboot for 0.5 second make led turn off
      digitalWrite(LED_BUILTIN, HIGH);

      // see if the user reacts to the led turned off with removing the keypress within 1 second
      while (((millis() - start_time) < 1000) && (exit_erase_loop != true))
      {

        // if user let go of key within 1 second erase eeprom
        if (digitalRead(EEPROM_RESET_PIN) != LOW)
        {
#if defined(FLASH_AS_EEPROM_h)
          EEPROM.read(0); // needed for SPI eeprom emulation.
          EEPROM.clear();
#else
          for (int i = 0; i < EEPROM.length(); i++)
          {
            EEPROM.write(i, 255);
          }
#endif
          // if erase done exit while loop.
          exit_erase_loop = true;
        }
      }
    }
  }
#endif

    // Unit tests should be independent of any stored configuration on the board!
#if !defined(UNIT_TEST)
    loadConfig();
    doUpdates(); //Check if any data items need updating (Occurs with firmware updates)
#endif


    //Always start with a clean slate on the bootloader capabilities level
    //This should be 0 until we hear otherwise from the 16u2
    configPage4.bootloaderCaps = 0;

    initialiseTimers();
    initialiseSchedulers();
    // This calls the board-specific init function.
    // See the board_xxx.ino files for these.
    initBoard();

#ifdef SD_LOGGING
    initRTC();
    initSD();
#endif

    //Serial.begin(115200);
    BIT_SET(currentStatus.status4, BIT_STATUS4_ALLOW_LEGACY_COMMS); // Flag legacy comms as being allowed on startup

    //Repoint the 2D table structs to the config pages that were just loaded
    taeTable.valueSize = SIZE_BYTE; //Set this table to use byte values
    taeTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    taeTable.xSize = 4;
    taeTable.values = configPage4.taeValues;
    taeTable.axisX = configPage4.taeBins;
    maeTable.valueSize = SIZE_BYTE; //Set this table to use byte values
    maeTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    maeTable.xSize = 4;
    maeTable.values = configPage4.maeRates;
    maeTable.axisX = configPage4.maeBins;
    WUETable.valueSize = SIZE_BYTE; //Set this table to use byte values
    WUETable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    WUETable.xSize = 10;
    WUETable.values = configPage2.wueValues;
    WUETable.axisX = configPage4.wueBins;
    ASETable.valueSize = SIZE_BYTE;
    ASETable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    ASETable.xSize = 4;
    ASETable.values = configPage2.asePct;
    ASETable.axisX = configPage2.aseBins;
    ASECountTable.valueSize = SIZE_BYTE;
    ASECountTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    ASECountTable.xSize = 4;
    ASECountTable.values = configPage2.aseCount;
    ASECountTable.axisX = configPage2.aseBins;
    PrimingPulseTable.valueSize = SIZE_BYTE;
    PrimingPulseTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    PrimingPulseTable.xSize = 4;
    PrimingPulseTable.values = configPage2.primePulse;
    PrimingPulseTable.axisX = configPage2.primeBins;
    crankingEnrichTable.valueSize = SIZE_BYTE;
    crankingEnrichTable.axisSize = SIZE_BYTE;
    crankingEnrichTable.xSize = 4;
    crankingEnrichTable.values = configPage10.crankingEnrichValues;
    crankingEnrichTable.axisX = configPage10.crankingEnrichBins;

    dwellVCorrectionTable.valueSize = SIZE_BYTE;
    dwellVCorrectionTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    dwellVCorrectionTable.xSize = 6;
    dwellVCorrectionTable.values = configPage4.dwellCorrectionValues;
    dwellVCorrectionTable.axisX = configPage6.voltageCorrectionBins;
    injectorVCorrectionTable.valueSize = SIZE_BYTE;
    injectorVCorrectionTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    injectorVCorrectionTable.xSize = 6;
    injectorVCorrectionTable.values = configPage6.injVoltageCorrectionValues;
    injectorVCorrectionTable.axisX = configPage6.voltageCorrectionBins;
    injectorAngleTable.valueSize = SIZE_INT;
    injectorAngleTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    injectorAngleTable.xSize = 4;
    injectorAngleTable.values = configPage2.injAng;
    injectorAngleTable.axisX = configPage2.injAngRPM;
    IATDensityCorrectionTable.valueSize = SIZE_BYTE;
    IATDensityCorrectionTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    IATDensityCorrectionTable.xSize = 9;
    IATDensityCorrectionTable.values = configPage6.airDenRates;
    IATDensityCorrectionTable.axisX = configPage6.airDenBins;
    baroFuelTable.valueSize = SIZE_BYTE;
    baroFuelTable.axisSize = SIZE_BYTE;
    baroFuelTable.xSize = 8;
    baroFuelTable.values = configPage4.baroFuelValues;
    baroFuelTable.axisX = configPage4.baroFuelBins;
    IATRetardTable.valueSize = SIZE_BYTE;
    IATRetardTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    IATRetardTable.xSize = 6;
    IATRetardTable.values = configPage4.iatRetValues;
    IATRetardTable.axisX = configPage4.iatRetBins;
    CLTAdvanceTable.valueSize = SIZE_BYTE;
    CLTAdvanceTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    CLTAdvanceTable.xSize = 6;
    CLTAdvanceTable.values = (byte*)configPage4.cltAdvValues;
    CLTAdvanceTable.axisX = configPage4.cltAdvBins;
    idleTargetTable.valueSize = SIZE_BYTE;
    idleTargetTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    idleTargetTable.xSize = 10;
    idleTargetTable.values = configPage6.iacCLValues;
    idleTargetTable.axisX = configPage6.iacBins;
    idleAdvanceTable.valueSize = SIZE_BYTE;
    idleAdvanceTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    idleAdvanceTable.xSize = 6;
    idleAdvanceTable.values = (byte*)configPage4.idleAdvValues;
    idleAdvanceTable.axisX = configPage4.idleAdvBins;
    rotarySplitTable.valueSize = SIZE_BYTE;
    rotarySplitTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    rotarySplitTable.xSize = 8;
    rotarySplitTable.values = configPage10.rotarySplitValues;
    rotarySplitTable.axisX = configPage10.rotarySplitBins;

    flexFuelTable.valueSize = SIZE_BYTE;
    flexFuelTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    flexFuelTable.xSize = 6;
    flexFuelTable.values = configPage10.flexFuelAdj;
    flexFuelTable.axisX = configPage10.flexFuelBins;
    flexAdvTable.valueSize = SIZE_BYTE;
    flexAdvTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    flexAdvTable.xSize = 6;
    flexAdvTable.values = configPage10.flexAdvAdj;
    flexAdvTable.axisX = configPage10.flexAdvBins;
    flexBoostTable.valueSize = SIZE_INT;
    flexBoostTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins (NOTE THIS IS DIFFERENT TO THE VALUES!!)
    flexBoostTable.xSize = 6;
    flexBoostTable.values = configPage10.flexBoostAdj;
    flexBoostTable.axisX = configPage10.flexBoostBins;
    fuelTempTable.valueSize = SIZE_BYTE;
    fuelTempTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    fuelTempTable.xSize = 6;
    fuelTempTable.values = configPage10.fuelTempValues;
    fuelTempTable.axisX = configPage10.fuelTempBins;

    knockWindowStartTable.valueSize = SIZE_BYTE;
    knockWindowStartTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    knockWindowStartTable.xSize = 6;
    knockWindowStartTable.values = configPage10.knock_window_angle;
    knockWindowStartTable.axisX = configPage10.knock_window_rpms;
    knockWindowDurationTable.valueSize = SIZE_BYTE;
    knockWindowDurationTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    knockWindowDurationTable.xSize = 6;
    knockWindowDurationTable.values = configPage10.knock_window_dur;
    knockWindowDurationTable.axisX = configPage10.knock_window_rpms;

    oilPressureProtectTable.valueSize = SIZE_BYTE;
    oilPressureProtectTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    oilPressureProtectTable.xSize = 4;
    oilPressureProtectTable.values = configPage10.oilPressureProtMins;
    oilPressureProtectTable.axisX = configPage10.oilPressureProtRPM;

    coolantProtectTable.valueSize = SIZE_BYTE;
    coolantProtectTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    coolantProtectTable.xSize = 6;
    coolantProtectTable.values = configPage9.coolantProtRPM;
    coolantProtectTable.axisX = configPage9.coolantProtTemp;


    fanPWMTable.valueSize = SIZE_BYTE;
    fanPWMTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    fanPWMTable.xSize = 4;
    fanPWMTable.values = configPage9.PWMFanDuty;
    fanPWMTable.axisX = configPage6.fanPWMBins;

    rollingCutTable.valueSize = SIZE_BYTE;
    rollingCutTable.axisSize = SIZE_SIGNED_BYTE; //X axis is SIGNED for this table.
    rollingCutTable.xSize = 4;
    rollingCutTable.values = configPage15.rollingProtCutPercent;
    rollingCutTable.axisX = configPage15.rollingProtRPMDelta;

    wmiAdvTable.valueSize = SIZE_BYTE;
    wmiAdvTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    wmiAdvTable.xSize = 6;
    wmiAdvTable.values = configPage10.wmiAdvAdj;
    wmiAdvTable.axisX = configPage10.wmiAdvBins;

    cltCalibrationTable.valueSize = SIZE_INT;
    cltCalibrationTable.axisSize = SIZE_INT;
    cltCalibrationTable.xSize = 32;
    cltCalibrationTable.values = cltCalibration_values;
    cltCalibrationTable.axisX = cltCalibration_bins;

    iatCalibrationTable.valueSize = SIZE_INT;
    iatCalibrationTable.axisSize = SIZE_INT;
    iatCalibrationTable.xSize = 32;
    iatCalibrationTable.values = iatCalibration_values;
    iatCalibrationTable.axisX = iatCalibration_bins;

    o2CalibrationTable.valueSize = SIZE_BYTE;
    o2CalibrationTable.axisSize = SIZE_INT;
    o2CalibrationTable.xSize = 32;
    o2CalibrationTable.values = o2Calibration_values;
    o2CalibrationTable.axisX = o2Calibration_bins;

    //Setup the calibration tables
    loadCalibration();

    //Set the pin mappings
    if((configPage2.pinMapping == 255) || (configPage2.pinMapping == 0)) //255 = EEPROM value in a blank AVR; 0 = EEPROM value in new FRAM
    {
      //First time running on this board
      resetConfigPages();
      configPage4.triggerTeeth = 4; //Avoid divide by 0 when start decoders
      setPinMapping(3); //Force board to v0.4
    }
    else
    {
      setPinMapping(configPage2.pinMapping);
    }

#if defined(NATIVE_CAN_AVAILABLE)
      initCAN();
#endif

    //Must come after setPinMapping() as secondary serial can be changed on a per board basis
#if defined(secondarySerial_AVAILABLE)
    if (configPage9.enable_secondarySerial == 1)
    {
      secondarySerial.begin(115200);
    }
#endif

    //End all coil charges to ensure no stray sparks on startup
    singleCoilEndCharge(ignition_id_1);
    singleCoilEndCharge(ignition_id_2);
    singleCoilEndCharge(ignition_id_3);
    singleCoilEndCharge(ignition_id_4);
#if (IGN_CHANNELS >= 5)
    singleCoilEndCharge(ignition_id_5);
#endif
#if (IGN_CHANNELS >= 6)
    singleCoilEndCharge(ignition_id_6);
#endif
#if (IGN_CHANNELS >= 7)
    singleCoilEndCharge(ignition_id_7);
#endif
#if (IGN_CHANNELS >= 8)
    singleCoilEndCharge(ignition_id_8);
#endif

    //Similar for injectors, make sure they're turned off
    closeSingleInjector(injector_id_1);
    closeSingleInjector(injector_id_2);
    closeSingleInjector(injector_id_3);
    closeSingleInjector(injector_id_4);
#if (INJ_CHANNELS >= 5)
    closeSingleInjector(injector_id_5);
#endif
#if (INJ_CHANNELS >= 6)
    closeSingleInjector(injector_id_6);
#endif
#if (INJ_CHANNELS >= 7)
    closeSingleInjector(injector_id_7);
#endif
#if (INJ_CHANNELS >= 8)
    closeSingleInjector(injector_id_8);
#endif

    //Perform all initialisations
    //initialiseDisplay();
    initialiseIdle(true);
    initialiseFan();
    initialiseAirCon();
    initialiseAuxPWM();
    initialiseCorrections();
    BIT_CLEAR(currentStatus.engineProtectStatus, PROTECT_IO_ERROR); //Clear the I/O error bit. The bit will be set in initialiseADC() if there is problem in there.
    initialiseADC();
    initialiseProgrammableIO();

    //Check whether the flex sensor is enabled and if so, attach an interrupt for it
    if(Flex.is_configured())
    {
      attachInterrupt(digitalPinToInterrupt(Flex.pin), flexPulse, CHANGE);
      currentStatus.ethanolPct = 0;
    }
    //Same as above, but for the VSS input
    if(VSS.is_configured()) // VSS modes 2 and 3 are interrupt drive (Mode 1 is CAN)
    {
      attachInterrupt(digitalPinToInterrupt(VSS.pin), vssPulse, RISING);
    }

    //Once the configs have been loaded, a number of one time calculations can be completed
    req_fuel_init_uS = configPage2.reqFuel * 100; //Convert to uS and an int. This is the only variable to be used in calculations
    req_fuel_uS = req_fuel_init_uS;
    inj_opentime_uS = configPage2.injOpen * 100; //Injector open time. Comes through as ms*10 (Eg 15.5ms = 155).

    if (configPage10.stagingEnabled)
    {
      uint32_t totalInjector = configPage10.stagedInjSizePri + configPage10.stagedInjSizeSec;
      /*
          These values are a percentage of the req_fuel value that would be required for each injector channel to deliver that much fuel.
          Eg:
          Pri injectors are 250cc
          Sec injectors are 500cc
          Total injector capacity = 750cc

          staged_req_fuel_mult_pri = 300% (The primary injectors would have to run 3x the overall PW in order to be the equivalent of the full 750cc capacity
          staged_req_fuel_mult_sec = 150% (The secondary injectors would have to run 1.5x the overall PW in order to be the equivalent of the full 750cc capacity
      */
      staged_req_fuel_mult_pri = (100 * totalInjector) / configPage10.stagedInjSizePri;
      staged_req_fuel_mult_sec = (100 * totalInjector) / configPage10.stagedInjSizeSec;
    }

    if (configPage4.trigPatternSec == SEC_TRIGGER_POLL && configPage4.TrigPattern == DECODER_MISSING_TOOTH)
    {
      configPage4.TrigEdgeSec = configPage4.PollLevelPolarity;
    } // set the secondary trigger edge automatically to correct working value with poll level mode to enable cam angle detection in closed loop vvt.

    //Explanation: currently cam trigger for VVT is only captured when revolution one == 1. So we need to make sure that the edge trigger happens on the first revolution. So now when we set the poll level to be low
    //on revolution one and it's checked at tooth #1. This means that the cam signal needs to go high during the first revolution to be high on next revolution at tooth #1. So poll level low = cam trigger edge rising.

    //Begin the main crank trigger interrupt pin setup
    //The interrupt numbering is a bit odd - See here for reference: arduino.cc/en/Reference/AttachInterrupt
    //These assignments are based on the Arduino Mega AND VARY BETWEEN BOARDS. Please confirm the board you are using and update accordingly.
    currentStatus.RPM = 0;
    currentStatus.hasSync = false;
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
    currentStatus.runSecs = 0;
    currentStatus.secl = 0;
    //currentStatus.seclx10 = 0;
    currentStatus.startRevolutions = 0;
    currentStatus.syncLossCounter = 0;
    currentStatus.flatShiftingHard = false;
    currentStatus.launchingHard = false;
    //Crank RPM limit (Saves us calculating this over and over again.
    //It's updated once per second in timers.ino)
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10);
    fuelPump.turnOff(); /* Assumes pin mappings have been applied. */
    currentStatus.engineProtectStatus = 0;
    triggerFilterTime = 0; //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise. This is simply a default value, the actual values are set in the setup() functions of each decoder
    dwellLimit_uS = (1000 * configPage4.dwellLimit);
    currentStatus.nChannels = ((uint8_t)INJ_CHANNELS << 4) + IGN_CHANNELS; //First 4 bits store the number of injection channels, 2nd 4 store the number of ignition channels
    ms_counter = 0;
    fixedCrankingOverride = 0;
    timer5_overflow_count = 0;
    toothHistoryIndex = 0;
    toothLastToothTime = 0;

    //Lookup the current MAP reading for barometric pressure
    instantaneousMAPReading(currentStatus.initialisationComplete);
    readBaro(currentStatus.initialisationComplete);

    noInterrupts();
    initialiseTriggers();

    //The secondary input can be used for VSS if nothing else requires it. Allows for the standard VR conditioner to be used for VSS. This MUST be run after the initialiseTriggers() function
    if (VSS_USES_RPM2())
    {
      //Secondary trigger input can safely be used for VSS
      attachInterrupt(digitalPinToInterrupt(VSS.pin), vssPulse, RISING);
    }
    if (FLEX_USES_RPM2())
    {
      //Secondary trigger input can safely be used for Flex sensor
      attachInterrupt(digitalPinToInterrupt(Flex.pin), flexPulse, CHANGE);
    }

    //End crank trigger interrupt attachment
    if(configPage2.strokes == FOUR_STROKE)
    {
      //Default is 1 squirt per revolution, so we halve the given req-fuel figure (Which would be over 2 revolutions)
      req_fuel_init_uS /= 2; //The req_fuel calculation above gives the total required fuel (At VE 100%) in the full cycle. If we're doing more than 1 squirt per cycle then we need to split the amount accordingly. (Note that in a non-sequential 4-stroke setup you cannot have less than 2 squirts as you cannot determine the stroke to make the single squirt on)
      req_fuel_uS = req_fuel_init_uS;
    }

    //Initial values for loop times
    currentLoopTime = micros_safe();
    mainLoopCount = 0;

    if(configPage2.divider == 0) { currentStatus.nSquirts = 2; } //Safety check.
    else { currentStatus.nSquirts = configPage2.nCylinders / configPage2.divider; } //The number of squirts being requested. This is manually overridden below for sequential setups (Due to TS req_fuel calc limitations)
    if(currentStatus.nSquirts == 0) { currentStatus.nSquirts = 1; } //Safety check. Should never happen as TS will give an error, but leave in case tune is manually altered etc.

    //Calculate the number of degrees between cylinders
    //Set some default values. These will be updated below if required.
    CRANK_ANGLE_MAX_IGN = 360;
    CRANK_ANGLE_MAX_INJ = 360;

    // Disable all injectors except channel 1
    injectors.setMaxInjectors(1);

    ignitions.resetEndAngle();

    if(configPage2.strokes == FOUR_STROKE) { CRANK_ANGLE_MAX_INJ = 720 / currentStatus.nSquirts; }
    else { CRANK_ANGLE_MAX_INJ = 360 / currentStatus.nSquirts; }

    switch (configPage2.nCylinders) {
    case 1:
        ignitions.ignition(ignChannel1).ignDegrees = 0;
        ignitions.setMaxIgnitions(1);
        injectors.injector(injChannel1).channelInjDegrees = 0;
        injectors.setMaxInjectors(2);

        //Sequential ignition works identically on a 1 cylinder whether it's odd or even fire.
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage2.strokes == FOUR_STROKE) )
        {
          CRANK_ANGLE_MAX_IGN = 720;
        }

        if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage2.strokes == FOUR_STROKE) )
        {
          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_init_uS * 2;
        }

        //Check if injector staging is enabled
        if (configPage10.stagingEnabled)
        {
          injectors.setMaxInjectors(2);

          injectors.injector(injChannel2).channelInjDegrees =
            injectors.injector(injChannel1).channelInjDegrees;
        }
        break;

    case 2:
      ignitions.ignition(ignChannel1).ignDegrees = 0;
      ignitions.setMaxIgnitions(2);
      injectors.injector(injChannel1).channelInjDegrees = 0;
      injectors.setMaxInjectors(2);

      if (configPage2.engineType == EVEN_FIRE)
      {
        ignitions.ignition(ignChannel2).ignDegrees = 180;
      }
      else
      {
        ignitions.ignition(ignChannel2).ignDegrees = configPage2.oddfire2;
      }

        //Sequential ignition works identically on a 2 cylinder whether it's odd or even fire (With the default being a 180 degree second cylinder).
        if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL && configPage2.strokes == FOUR_STROKE)
        {
          CRANK_ANGLE_MAX_IGN = 720;
        }

        if (configPage2.injLayout == INJ_SEQUENTIAL && configPage2.strokes == FOUR_STROKE)
        {
          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_init_uS * 2;
        }
        //The below are true regardless of whether this is running sequential or not
        if (configPage2.engineType == EVEN_FIRE)
        {
          injectors.injector(injChannel2).channelInjDegrees = 180;
        }
        else
        {
          injectors.injector(injChannel2).channelInjDegrees = configPage2.oddfire2;
        }

        if (!configPage2.injTiming)
        {
          //For simultaneous, all squirts happen at the same time
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 0;
        }

        //Check if injector staging is enabled
        if (configPage10.stagingEnabled)
        {
          injectors.setMaxInjectors(4);

          injectors.injector(injChannel3).channelInjDegrees =
            injectors.injector(injChannel1).channelInjDegrees;
          injectors.injector(injChannel4).channelInjDegrees =
            injectors.injector(injChannel2).channelInjDegrees;
        }

        break;

    case 3:
      ignitions.ignition(ignChannel1).ignDegrees = 0;
      ignitions.setMaxIgnitions(3);
      injectors.setMaxInjectors(3);
      if (configPage2.engineType == EVEN_FIRE)
      {
        // Sequential and Single channel modes both run over 720 crank degrees, but only on 4 stroke engines.
        if ((configPage4.sparkMode == IGN_MODE_SEQUENTIAL || configPage4.sparkMode == IGN_MODE_SINGLE) && configPage2.strokes == FOUR_STROKE)
        {
          ignitions.ignition(ignChannel2).ignDegrees = 240;
          ignitions.ignition(ignChannel3).ignDegrees = 480;

          CRANK_ANGLE_MAX_IGN = 720;
        }
        else
        {
          ignitions.ignition(ignChannel2).ignDegrees = 120;
          ignitions.ignition(ignChannel3).ignDegrees = 240;
        }
        }
        else
        {
          ignitions.ignition(ignChannel2).ignDegrees = configPage2.oddfire2;
          ignitions.ignition(ignChannel3).ignDegrees = configPage2.oddfire3;
        }

        //For alternating injection, the squirt occurs at different times for each channel
        if( (configPage2.injLayout == INJ_SEMISEQUENTIAL) || (configPage2.injLayout == INJ_PAIRED) )
        {
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 120;
          injectors.injector(injChannel3).channelInjDegrees = 240;

          if(configPage2.injType == INJ_TYPE_PORT)
          {
            //Force nSquirts to 2 for individual port injection.
            //This prevents TunerStudio forcing the value to 3 even when this isn't wanted.
            currentStatus.nSquirts = 2;
            if(configPage2.strokes == FOUR_STROKE)
            {
              CRANK_ANGLE_MAX_INJ = 360;
            }
            else
            {
              CRANK_ANGLE_MAX_INJ = 180;
            }
          }

          //Adjust the injection angles based on the number of squirts
          if (currentStatus.nSquirts > 2)
          {
            injectors.injector(injChannel2).channelInjDegrees =
              (injectors.injector(injChannel2).channelInjDegrees * 2) / currentStatus.nSquirts;
            injectors.injector(injChannel3).channelInjDegrees =
              (injectors.injector(injChannel3).channelInjDegrees * 2) / currentStatus.nSquirts;
          }

          if (!configPage2.injTiming)
          {
            //For simultaneous, all squirts happen at the same time
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 0;
            injectors.injector(injChannel3).channelInjDegrees = 0;
          }
        }
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          currentStatus.nSquirts = 1;

          if(configPage2.strokes == TWO_STROKE)
          {
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 120;
            injectors.injector(injChannel3).channelInjDegrees = 240;
            CRANK_ANGLE_MAX_INJ = 360;
          }
          else
          {
            req_fuel_uS = req_fuel_init_uS * 2;
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 240;
            injectors.injector(injChannel3).channelInjDegrees = 480;
            CRANK_ANGLE_MAX_INJ = 720;
          }
        }
        else
        {
          //Should never happen, but default values
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 120;
          injectors.injector(injChannel3).channelInjDegrees = 240;
        }

        //Check if injector staging is enabled
        if (configPage10.stagingEnabled)
        {
#if INJ_CHANNELS >= 6
          injectors.setMaxInjectors(6);

          injectors.injector(injChannel4).channelInjDegrees =
            injectors.injector(injChannel1).channelInjDegrees;
          injectors.injector(injChannel5).channelInjDegrees =
            injectors.injector(injChannel2).channelInjDegrees;
          injectors.injector(injChannel6).channelInjDegrees =
            injectors.injector(injChannel3).channelInjDegrees;
#else
          //Staged output is on channel 4
          injectors.setMaxInjectors(4);

          injectors.injector(injChannel4).channelInjDegrees =
            injectors.injector(injChannel1).channelInjDegrees;
#endif
        }
        break;
    case 4:
        ignitions.ignition(ignChannel1).ignDegrees = 0;
        ignitions.setMaxIgnitions(2);
        injectors.injector(injChannel1).channelInjDegrees = 0;
        injectors.setMaxInjectors(2);

        if (configPage2.engineType == EVEN_FIRE)
        {
          ignitions.ignition(ignChannel2).ignDegrees = 180;

          if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
              && configPage2.strokes == FOUR_STROKE)
          {
            ignitions.ignition(ignChannel3).ignDegrees = 360;
            ignitions.ignition(ignChannel4).ignDegrees = 540;

            CRANK_ANGLE_MAX_IGN = 720;
            ignitions.setMaxIgnitions(4);
          }
          if(configPage4.sparkMode == IGN_MODE_ROTARY)
          {
            //Rotary uses the ign 3 and 4 schedules for the trailing spark. They are offset from the ign 1 and 2 channels respectively and so use the same degrees as them
            ignitions.ignition(ignChannel3).ignDegrees = 0;
            ignitions.ignition(ignChannel4).ignDegrees = 180;
            ignitions.setMaxIgnitions(4);

            configPage4.IgInv = GOING_LOW; //Force Going Low ignition mode (Going high is never used for rotary)
          }
        }
        else
        {
          ignitions.ignition(ignChannel2).ignDegrees = configPage2.oddfire2;
          ignitions.ignition(ignChannel3).ignDegrees = configPage2.oddfire3;
          ignitions.ignition(ignChannel4).ignDegrees = configPage2.oddfire4;
          ignitions.setMaxIgnitions(4);
        }

        //For alternating injection, the squirt occurs at different times for each channel
        if (configPage2.injLayout == INJ_SEMISEQUENTIAL
            || configPage2.injLayout == INJ_PAIRED
            || configPage2.strokes == TWO_STROKE)
        {
          injectors.injector(injChannel2).channelInjDegrees = 180;

          if (!configPage2.injTiming)
          {
            //For simultaneous, all squirts happen at the same time
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 0;
          }
          else if (currentStatus.nSquirts > 2)
          {
            //Adjust the injection angles based on the number of squirts
            injectors.injector(injChannel2).channelInjDegrees =
              (injectors.injector(injChannel2).channelInjDegrees * 2) / currentStatus.nSquirts;
          }
          else
          {
            //Do nothing, default values are correct
          }
        }
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          injectors.injector(injChannel2).channelInjDegrees = 180;
          injectors.injector(injChannel3).channelInjDegrees = 360;
          injectors.injector(injChannel4).channelInjDegrees = 540;

          injectors.setMaxInjectors(4);

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_init_uS * 2;
        }
        else
        {
          //Should never happen, but default values.
          injectors.setMaxInjectors(2);
        }

        //Check if injector staging is enabled
        if (configPage10.stagingEnabled)
        {
          injectors.setMaxInjectors(4);

          if (configPage2.injLayout == INJ_SEQUENTIAL
              || configPage2.injLayout == INJ_SEMISEQUENTIAL)
          {
            //Staging with 4 cylinders semi/sequential requires 8 total channels
            #if INJ_CHANNELS >= 8
              injectors.setMaxInjectors(8);

              injectors.injector(injChannel5).channelInjDegrees =
                injectors.injector(injChannel1).channelInjDegrees;
              injectors.injector(injChannel6).channelInjDegrees =
                injectors.injector(injChannel2).channelInjDegrees;
              injectors.injector(injChannel7).channelInjDegrees =
                injectors.injector(injChannel3).channelInjDegrees;
              injectors.injector(injChannel8).channelInjDegrees =
                injectors.injector(injChannel4).channelInjDegrees;
            #else
              //This is an invalid config as there are not enough outputs to
              //support sequential + staging.
              //Put the staging output to the non-existent channel 5
              #if (INJ_CHANNELS >= 5)
              injectors.setMaxInjectors(5);
              injectors.injector(injChannel5).channelInjDegrees =
                injectors.injector(injChannel1).channelInjDegrees;
              #endif
            #endif
          }
          else
          {
            injectors.injector(injChannel3).channelInjDegrees =
              injectors.injector(injChannel1).channelInjDegrees;
            injectors.injector(injChannel4).channelInjDegrees =
              injectors.injector(injChannel2).channelInjDegrees;
          }
        }

        break;
    case 5:
      ignitions.ignition(ignChannel1).ignDegrees = 0;
      ignitions.ignition(ignChannel2).ignDegrees = 72;
      ignitions.ignition(ignChannel3).ignDegrees = 144;
      ignitions.ignition(ignChannel4).ignDegrees = 216;
#if (IGN_CHANNELS >= 5)
      ignitions.ignition(ignChannel5).ignDegrees = 288;
#endif
      ignitions.setMaxIgnitions(5);
      // Is updated below to 5 if there are enough channels.
      injectors.setMaxInjectors(4);

      if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
      {
        ignitions.ignition(ignChannel2).ignDegrees = 144;
        ignitions.ignition(ignChannel3).ignDegrees = 288;
        ignitions.ignition(ignChannel4).ignDegrees = 432;
#if (IGN_CHANNELS >= 5)
        ignitions.ignition(ignChannel5).ignDegrees = 576;
#endif

          CRANK_ANGLE_MAX_IGN = 720;
      }

        //For alternating injection, the squirt occurs at different times for each channel
        if (configPage2.injLayout == INJ_SEMISEQUENTIAL
            || configPage2.injLayout == INJ_PAIRED
            || configPage2.strokes == TWO_STROKE)
        {
          if (!configPage2.injTiming)
          {
            //For simultaneous, all squirts happen at the same time
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 0;
            injectors.injector(injChannel3).channelInjDegrees = 0;
            injectors.injector(injChannel4).channelInjDegrees = 0;
#if (INJ_CHANNELS >= 5)
            injectors.injector(injChannel5).channelInjDegrees = 0;
#endif
          }
          else
          {
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 72;
            injectors.injector(injChannel3).channelInjDegrees = 144;
            injectors.injector(injChannel4).channelInjDegrees = 216;
#if (INJ_CHANNELS >= 5)
            injectors.injector(injChannel5).channelInjDegrees = 288;
#endif

            //Divide by currentStatus.nSquirts ?
          }
        }
#if INJ_CHANNELS >= 5
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 144;
          injectors.injector(injChannel3).channelInjDegrees = 288;
          injectors.injector(injChannel4).channelInjDegrees = 432;
          injectors.injector(injChannel5).channelInjDegrees = 576;

          injectors.setMaxInjectors(5);

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_init_uS * 2;
        }
#endif

#if INJ_CHANNELS >= 6
        if (configPage10.stagingEnabled)
        {
          injectors.setMaxInjectors(6);
        }
#endif
        break;
    case 6:
        ignitions.ignition(ignChannel1).ignDegrees = 0;
        ignitions.ignition(ignChannel2).ignDegrees = 120;
        ignitions.ignition(ignChannel3).ignDegrees = 240;
        ignitions.setMaxIgnitions(3);
        injectors.setMaxInjectors(3);

#if IGN_CHANNELS >= 6
        if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
        {
        ignitions.ignition(ignChannel4).ignDegrees = 360;
        ignitions.ignition(ignChannel5).ignDegrees = 480;
        ignitions.ignition(ignChannel6).ignDegrees = 600;
        CRANK_ANGLE_MAX_IGN = 720;
        ignitions.setMaxIgnitions(6);
        }
#endif

        //For alternating injection, the squirt occurs at different times for each channel
        if (configPage2.injLayout == INJ_SEMISEQUENTIAL || configPage2.injLayout == INJ_PAIRED)
        {
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 120;
          injectors.injector(injChannel3).channelInjDegrees = 240;
          if (!configPage2.injTiming)
          {
            //For simultaneous, all squirts happen at the same time
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 0;
            injectors.injector(injChannel3).channelInjDegrees = 0;
          }
          else if (currentStatus.nSquirts > 2)
          {
            //Adjust the injection angles based on the number of squirts
            injectors.injector(injChannel2).channelInjDegrees =
              (injectors.injector(injChannel2).channelInjDegrees * 2) / currentStatus.nSquirts;
            injectors.injector(injChannel3).channelInjDegrees =
              (injectors.injector(injChannel3).channelInjDegrees * 2) / currentStatus.nSquirts;
          }
        }

#if INJ_CHANNELS >= 6
        if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 120;
          injectors.injector(injChannel3).channelInjDegrees = 240;
          injectors.injector(injChannel4).channelInjDegrees = 360;
          injectors.injector(injChannel5).channelInjDegrees = 480;
          injectors.injector(injChannel6).channelInjDegrees = 600;

          injectors.setMaxInjectors(6);

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_init_uS * 2;
        }
        else if (configPage10.stagingEnabled) //Check if injector staging is enabled
        {
          injectors.setMaxInjectors(6);

          if (configPage2.injLayout == INJ_SEMISEQUENTIAL)
          {
            //Staging with 6 cylinders semi/sequential requires 8 total channels.
            injectors.injector(injChannel4).channelInjDegrees =
              injectors.injector(injChannel1).channelInjDegrees;
            injectors.injector(injChannel5).channelInjDegrees =
              injectors.injector(injChannel2).channelInjDegrees;
            injectors.injector(injChannel6).channelInjDegrees =
              injectors.injector(injChannel3).channelInjDegrees;
#if INJ_CHANNELS >= 8
              injectors.setMaxInjectors(8);

              injectors.injector(injChannel7).channelInjDegrees =
                injectors.injector(injChannel1).channelInjDegrees;
              injectors.injector(injChannel8).channelInjDegrees =
                injectors.injector(injChannel1).channelInjDegrees;
#else
              //This is an invalid config as there are not enough outputs to
              //support sequential + staging
              //No staging output will be active.
#endif
          }
        }
#endif
        break;
    case 8:
        ignitions.ignition(ignChannel1).ignDegrees = 0;
        ignitions.ignition(ignChannel2).ignDegrees = 90;
        ignitions.ignition(ignChannel3).ignDegrees = 180;
        ignitions.ignition(ignChannel4).ignDegrees = 270;
        ignitions.setMaxIgnitions(4);
        injectors.setMaxInjectors(4);


        if (configPage4.sparkMode == IGN_MODE_SINGLE)
        {
          ignitions.setMaxIgnitions(4);
          CRANK_ANGLE_MAX_IGN = 360;
        }

#if IGN_CHANNELS >= 8
        if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
        {
        ignitions.ignition(ignChannel5).ignDegrees = 360;
        ignitions.ignition(ignChannel6).ignDegrees = 450;
        ignitions.ignition(ignChannel7).ignDegrees = 540;
        ignitions.ignition(ignChannel8).ignDegrees = 630;
        ignitions.setMaxIgnitions(8);
        CRANK_ANGLE_MAX_IGN = 720;
        }
#endif

        //For alternating injection, the squirt occurs at different times for each channel
        if (configPage2.injLayout == INJ_SEMISEQUENTIAL || configPage2.injLayout == INJ_PAIRED)
        {
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 90;
          injectors.injector(injChannel3).channelInjDegrees = 180;
          injectors.injector(injChannel4).channelInjDegrees = 270;

          if (!configPage2.injTiming)
          {
            //For simultaneous, all squirts happen at the same time
            injectors.injector(injChannel1).channelInjDegrees = 0;
            injectors.injector(injChannel2).channelInjDegrees = 0;
            injectors.injector(injChannel3).channelInjDegrees = 0;
            injectors.injector(injChannel4).channelInjDegrees = 0;
          }
          else if (currentStatus.nSquirts > 2)
          {
            //Adjust the injection angles based on the number of squirts
            injectors.injector(injChannel2).channelInjDegrees =
              (injectors.injector(injChannel2).channelInjDegrees * 2) / currentStatus.nSquirts;
            injectors.injector(injChannel3).channelInjDegrees =
              (injectors.injector(injChannel3).channelInjDegrees * 2) / currentStatus.nSquirts;
            injectors.injector(injChannel4).channelInjDegrees =
              (injectors.injector(injChannel4).channelInjDegrees * 2) / currentStatus.nSquirts;
          }
        }

#if INJ_CHANNELS >= 8
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          injectors.injector(injChannel1).channelInjDegrees = 0;
          injectors.injector(injChannel2).channelInjDegrees = 90;
          injectors.injector(injChannel3).channelInjDegrees = 180;
          injectors.injector(injChannel4).channelInjDegrees = 270;
          injectors.injector(injChannel5).channelInjDegrees = 360;
          injectors.injector(injChannel6).channelInjDegrees = 450;
          injectors.injector(injChannel7).channelInjDegrees = 540;
          injectors.injector(injChannel8).channelInjDegrees = 630;

          injectors.setMaxInjectors(8);

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_init_uS * 2;
        }
#endif

        break;
    default: // TODO: Handle this better!!!
        injectors.injector(injChannel1).channelInjDegrees = 0;
        injectors.injector(injChannel2).channelInjDegrees = 180;
        break;
    }
    //Top 3 bits of the status3 variable are the number of squirts.
    //This must be done after the above section due to nSquirts being forced to
    //1 for sequential
    currentStatus.status3 |= currentStatus.nSquirts << BIT_STATUS3_NSQUIRTS1;

    //Special case:
    //3 or 5 squirts per cycle MUST be tracked over 720 degrees.
    //This is because the angles for them (Eg 720/3=240) are not evenly divisible into 360
    //This is ONLY the case on 4 stroke systems
    if(currentStatus.nSquirts == 3 || currentStatus.nSquirts == 5)
    {
      if(configPage2.strokes == FOUR_STROKE)
      {
        CRANK_ANGLE_MAX_INJ = 720U / currentStatus.nSquirts;
      }
    }

    switch(configPage2.injLayout)
    {
    case INJ_SEMISEQUENTIAL:
        //Semi-Sequential injection. Currently possible with 4, 6 and 8 cylinders. 5 cylinder is a special case
      if (configPage2.nCylinders == 4)
      {
        if (configPage4.inj4cylPairing == INJ_PAIR_13_24)
        {
          injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_3);
          injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_4);
        }
        else
        {
          injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_4);
          injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_3);
        }
      }
#if INJ_CHANNELS >= 5
      else if (configPage2.nCylinders == 5)
      {
        //This is similar to the paired injection but uses five injector outputs instead of four.
        injectors.configure_injector_schedule(injChannel1, injector_id_1);
        injectors.configure_injector_schedule(injChannel2, injector_id_2);
        injectors.configure_injector_schedule(injChannel3, injector_id_3, injector_id_5);
        injectors.configure_injector_schedule(injChannel4, injector_id_4);
      }
#endif
#if INJ_CHANNELS >= 6
      else if (configPage2.nCylinders == 6)
      {
        injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_4);
        injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_5);
        injectors.configure_injector_schedule(injChannel3, injector_id_3, injector_id_6);
      }
#endif
#if INJ_CHANNELS >= 8
      else if (configPage2.nCylinders == 8)
      {
        injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_5);
        injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_6);
        injectors.configure_injector_schedule(injChannel3, injector_id_3, injector_id_7);
        injectors.configure_injector_schedule(injChannel4, injector_id_4, injector_id_8);
      }
#endif
      else
      {
        //Fall back to paired injection
        injectors.configure_sequential_injector_schedules(MIN((size_t)injChannelCount, 5));
      }
      break;

    case INJ_SEQUENTIAL:
      //Sequential injection
      injectors.configure_sequential_injector_schedules((size_t)injChannelCount);
      break;

    case INJ_PAIRED:
    default:
      //Paired injection
      injectors.configure_sequential_injector_schedules(MIN((size_t)injChannelCount, 5));
      break;

    }

    switch(configPage4.sparkMode)
    {
    case IGN_MODE_WASTED:
      //Wasted Spark (Normal mode)
      ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
      ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
      ignitions.configure_coil_schedule(ignChannel4, ignition_id_4);
      ignitions.configure_coil_schedule(ignChannel5, ignition_id_5);
      break;

    case IGN_MODE_SINGLE:
      //Single channel mode. All ignition pulses are on channel 1
      ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel2, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel3, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel4, ignition_id_1);
#if IGN_CHANNELS >= 5
      ignitions.configure_coil_schedule(ignChannel5, ignition_id_1);
#endif
#if IGN_CHANNELS >= 6
      ignitions.configure_coil_schedule(ignChannel6, ignition_id_1);
#endif
#if IGN_CHANNELS >= 7
      ignitions.configure_coil_schedule(ignChannel7, ignition_id_1);
#endif
#if IGN_CHANNELS >= 8
      ignitions.configure_coil_schedule(ignChannel8, ignition_id_1);
#endif
      break;

    case IGN_MODE_WASTEDCOP:
      //Wasted COP mode. Note, most of the boards can only run this for 4-cyl only.
      if( configPage2.nCylinders <= 3)
      {
        //1-3 cylinder wasted COP is the same as regular wasted mode
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
      }
      else if( configPage2.nCylinders == 4 )
      {
        //Wasted COP mode for 4 cylinders. Ignition channels 1&3 and 2&4 are paired together
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1, ignition_id_3);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2, ignition_id_4);
        ignitions.inhibit_coil_schedule(ignChannel3);
        ignitions.inhibit_coil_schedule(ignChannel4);
      }
      else if( configPage2.nCylinders == 6 )
      {
        //Wasted COP mode for 6 cylinders. Ignition channels 1&4, 2&5 and 3&6 are paired together
#if IGN_CHANNELS >= 6
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1, ignition_id_4);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2, ignition_id_5);
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_3, ignition_id_6);
        ignitions.inhibit_coil_schedule(ignChannel4);
        ignitions.inhibit_coil_schedule(ignChannel5);
        ignitions.inhibit_coil_schedule(ignChannel6);
#endif
      }
      else if( configPage2.nCylinders == 8 )
      {
        //Wasted COP mode for 8 cylinders. Ignition channels 1&5, 2&6, 3&7 and 4&8 are paired together
#if IGN_CHANNELS >= 8
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1, ignition_id_5);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2, ignition_id_6);
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_3, ignition_id_7);
        ignitions.configure_coil_schedule(ignChannel4, ignition_id_4, ignition_id_8);

        ignitions.inhibit_coil_schedule(ignChannel5);
        ignitions.inhibit_coil_schedule(ignChannel6);
        ignitions.inhibit_coil_schedule(ignChannel7);
        ignitions.inhibit_coil_schedule(ignChannel8);
#endif
      }
      else
      {
        //If the person has inadvertently selected this when running more than
        //4 cylinders or other than 6 cylinders, just use standard Wasted spark mode
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
        ignitions.configure_coil_schedule(ignChannel4, ignition_id_4);
#if IGN_CHANNELS >= 5
        ignitions.configure_coil_schedule(ignChannel5, ignition_id_5);
#endif
      }
      break;

    case IGN_MODE_SEQUENTIAL:
      ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
      ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
      ignitions.configure_coil_schedule(ignChannel4, ignition_id_4);
      ignitions.configure_coil_schedule(ignChannel5, ignition_id_5);
#if IGN_CHANNELS >= 6
      ignitions.configure_coil_schedule(ignChannel6, ignition_id_6);
#endif
#if IGN_CHANNELS >= 7
      ignitions.configure_coil_schedule(ignChannel7, ignition_id_7);
#endif
#if IGN_CHANNELS >= 8
      ignitions.configure_coil_schedule(ignChannel8, ignition_id_8);
#endif
      break;

    case IGN_MODE_ROTARY:
      if(configPage10.rotaryType == ROTARY_IGN_FC)
      {
        //Ignition channel 1 is a wasted spark signal for leading signal on both rotors
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_1);
        ignitions.configure_rotary_fc_trailing_coil_schedules();
      }
      else if(configPage10.rotaryType == ROTARY_IGN_FD)
      {
        //Ignition channel 1 is a wasted spark signal for leading signal on both rotors
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_1);
        //Trailing coils have their own channel each
        //IGN2 = front rotor trailing spark
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_2);
        //IGN3 = rear rotor trailing spark
        ignitions.configure_coil_schedule(ignChannel4, ignition_id_3);

        //IGN4 not used
      }
      else if(configPage10.rotaryType == ROTARY_IGN_RX8)
      {
        //RX8 outputs are simply 1 coil and 1 output per plug

        //IGN1 is front rotor, leading spark
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
        //IGN2 is rear rotor, leading spark
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
        //IGN3 = front rotor trailing spark
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
        //IGN4 = rear rotor trailing spark
        ignitions.configure_coil_schedule(ignChannel4, ignition_id_4);
      }
      else
      {
         //No action for other RX ignition modes (Future expansion / MISRA compliant).
      }
      break;

    default:
      //Wasted spark (Shouldn't ever happen anyway)
      ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
      ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
      ignitions.configure_coil_schedule(ignChannel4, ignition_id_4);
#if IGN_CHANNELS >= 5
      ignitions.configure_coil_schedule(ignChannel5, ignition_id_5);
#endif
      break;
    }

    //Begin priming the fuel pump. This is turned off in the low resolution, 1s interrupt in timers.ino
    //First check that the priming time is not 0.
    if(configPage2.fpPrime > 0)
    {
      fuelPriming.start(currentStatus.secl);
      fuelPump.turnOn();
    }
    else
    {
      //If the user has set 0 for the pump priming, immediately mark the priming
      //as being completed.
      fuelPriming.complete();
    }

    interrupts();

    // Need to read coolant temp to make priming pulsewidth work correctly.
    // The false here disables use of the filter
    readCLT(false);
    readTPS(false); // Need to read tps to detect flood clear state

    /* tacho sweep function. */
    /*
     * SweepMax is stored as a byte, RPM/100. divide by 60 to convert min to sec
     * (net 5/3).  Multiply by ignition pulses per rev.
     * tachoSweepIncr is also the number of tach pulses per second
     */
    tachoSweepIncr = configPage2.tachoSweepMaxRPM * ignitions.maxOutputs * 5 / 3;

    currentStatus.initialisationComplete = true;
    digitalWrite(LED_BUILTIN, HIGH);
}

static bool
digital_pin_is_configured(byte const pin)
{
  bool const is_configured = pin != 0 && pin < BOARD_MAX_IO_PINS;

  return is_configured;
}

static bool
analog_pin_is_configured(byte const enable, byte const pin)
{
  return enable != 0 && pin < BOARD_MAX_IO_PINS;
}

static void
translate_digital_pin_if_configured(byte const pin, byte &translated_pin)
{
  if (digital_pin_is_configured(pin))
  {
    translated_pin = pinTranslate(pin);
  }
}

static void
translate_analog_pin_if_configured(byte const enable, byte const pin, byte &translated_pin)
{
  if (analog_pin_is_configured(enable, pin))
  {
    translated_pin = pinTranslateAnalog(pin);
  }
}

static void
setup_selectable_io(void)
{
  translate_digital_pin_if_configured(configPage6.launchPin, Launch.pin);
  translate_digital_pin_if_configured(configPage4.ignBypassPin, IgnBypass.pin);
  translate_digital_pin_if_configured(configPage2.tachoPin, TachOut.pin);
  translate_digital_pin_if_configured(configPage4.fuelPumpPin, FuelPump.pin);
  translate_digital_pin_if_configured(configPage6.fanPin, Fan.pin);
  translate_digital_pin_if_configured(configPage6.boostPin, Boost.pin);
  translate_digital_pin_if_configured(configPage6.vvt1Pin, VVT_1.pin);

  translate_analog_pin_if_configured(configPage6.useExtBaro, configPage6.baroPin, pinBaro);
  translate_analog_pin_if_configured(configPage6.useEMAP, configPage10.EMAPPin, pinEMAP);

  translate_digital_pin_if_configured(configPage10.fuel2InputPin, Fuel2Input.pin);
  translate_digital_pin_if_configured(configPage10.spark2InputPin, Spark2Input.pin);
  translate_digital_pin_if_configured(configPage2.vssPin, VSS.pin);

  translate_analog_pin_if_configured(configPage10.fuelPressureEnable, configPage10.fuelPressurePin, pinFuelPressure);
  translate_analog_pin_if_configured(configPage10.oilPressureEnable, configPage10.oilPressurePin, pinOilPressure);

  translate_digital_pin_if_configured(configPage10.wmiEmptyPin, WMIEmpty.pin);
  translate_digital_pin_if_configured(configPage10.wmiIndicatorPin, WMIIndicator.pin);
  translate_digital_pin_if_configured(configPage10.wmiEnabledPin, WMIEnabled.pin);
  translate_digital_pin_if_configured(configPage10.vvt2Pin, VVT_2.pin);
  if (configPage13.onboard_log_trigger_Epin != 0)
  {
    translate_digital_pin_if_configured(configPage13.onboard_log_tr5_Epin_pin, SDEnable.pin);
  }

}

static void setResetControlPinState(void)
{
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);

  /* Setup reset control initial state */
  switch (resetControl)
  {
    case RESET_CONTROL_PREVENT_WHEN_RUNNING:
      /* Set the reset control pin LOW and change it to HIGH later when we get sync. */
      ResetControl.configure(LOW);
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      break;
    case RESET_CONTROL_PREVENT_ALWAYS:
      /* Set the reset control pin HIGH and never touch it again. */
      ResetControl.configure(HIGH);
      BIT_SET(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      break;
    case RESET_CONTROL_SERIAL_COMMAND:
      /*
       * Set the reset control pin HIGH. There currently isn't any practical difference
       * between this and PREVENT_ALWAYS but it doesn't hurt anything to have them separate.
       */
      ResetControl.configure(HIGH);
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      break;
    default:
      // Do nothing - keep MISRA happy
      break;
  }
}

/** Set board / microcontroller specific pin mappings / assignments.
 * The boardID is switch-case compared against raw boardID integers (not enum or defined label, and probably no need for that either)
 * which are originated from tuning SW (e.g. TS) set values and are available in reference/speeduino.ini (See pinLayout, note also that
 * numbering is not contiguous here).
 */
void setPinMapping(byte boardID)
{
  //Force set defaults. Will be overwritten below if needed.
  injectorControlMethodAssign(OUTPUT_CONTROL_DIRECT);
  ignitionControlMethodAssign(OUTPUT_CONTROL_DIRECT);

  switch (boardID)
  {
    //Note: Case 0 (Speeduino v0.1) was removed in Nov 2020 to handle default case for blank FRAM modules

    case 1:
    #ifndef SMALL_FLASH_MODE //No support for bluepill here anyway
      //Pin mappings as per the v0.2 shield
      inj1.pin = 8; //Output pin injector 1 is on
      inj2.pin = 9; //Output pin injector 2 is on
      inj3.pin = 10; //Output pin injector 3 is on
      inj4.pin = 11; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 12; //Output pin injector 5 is on
#endif
      ign1.pin = 28; //Pin for coil 1
      ign2.pin = 24; //Pin for coil 2
      ign3.pin = 40; //Pin for coil 3
      ign4.pin = 36; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(20); //The CAS pin
      Trigger2.setPin(21); //The Cam Sensor pin
      Trigger3.setPin(3); //The Cam sensor 2 pin
      pinTPS = A2; //TPS input pin
      pinMAP = A3; //MAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A8; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      TachOut.pin = 49; //Tacho output pin
      Idle1.pin = 30; //Single wire idle control
      Idle2.pin = 31; //2 wire idle control
      StepperDir.setPin(16); //Direction pin  for DRV8825 driver
      StepperStep.pin = 17; //Step pin for DRV8825 driver
      Fan.pin = 47; //Pin for the fan output
      FuelPump.pin = 4; //Fuel pump output
      Flex.pin = 2; // Flex sensor (Must be external interrupt enabled)
      ResetControl.pin = 43; //Reset control output
      break;
    #endif
    case 2:
    #ifndef SMALL_FLASH_MODE //No support for bluepill here anyway
      //Pin mappings as per the v0.3 shield
      inj1.pin = 8; //Output pin injector 1 is on
      inj2.pin = 9; //Output pin injector 2 is on
      inj3.pin = 10; //Output pin injector 3 is on
      inj4.pin = 11; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 12; //Output pin injector 5 is on
#endif

      ign1.pin = 28; //Pin for coil 1
      ign2.pin = 24; //Pin for coil 2
      ign3.pin = 40; //Pin for coil 3
      ign4.pin = 36; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(3); //The Cam sensor 2 pin
      pinTPS = A2;//TPS input pin
      pinMAP = A3; //MAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A8; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      TachOut.pin = 49; //Tacho output pin
      Idle1.pin = 5; //Single wire idle control
      Idle2.pin = 53; //2 wire idle control
      Boost.pin = 7; //Boost control
      VVT_1.pin = 6; //Default VVT output
      VVT_2.pin = 48; //Default VVT2 output
      FuelPump.pin = 4; //Fuel pump output
      StepperDir.setPin(16); //Direction pin  for DRV8825 driver
      StepperStep.pin = 17; //Step pin for DRV8825 driver
      StepperEnable.pin = 26; //Enable pin for DRV8825
      Fan.pin = A13; //Pin for the fan output
      Launch.pin = 51; //Can be overwritten below
      Flex.pin = 2; // Flex sensor (Must be external interrupt enabled)
      ResetControl.pin = 50; //Reset control output
      pinBaro = A5;
      VSS.pin = 20;

#if defined(CORE_TEENSY35)
        Trigger.setPin(23);
        StepperDir.setPin(33);
        StepperStep.pin = 34;
        ign1.pin = 31;
        TachOut.pin = 28;
        Fan.pin = 27;
        ign4.pin = 21;
        ign3.pin = 30;
        pinO2 = A22;
#endif
#endif
      break;

    case 3:
      //Pin mappings as per the v0.4 shield
      inj1.pin = 8; //Output pin injector 1 is on
      inj2.pin = 9; //Output pin injector 2 is on
      inj3.pin = 10; //Output pin injector 3 is on
      inj4.pin = 11; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 12; //Output pin injector 5 is on
#endif
#if INJ_CHANNELS >= 6
      inj6.pin = 50; //CAUTION: Uses the same as Coil 4 below.
#endif

      ign1.pin = 40; //Pin for coil 1
      ign2.pin = 38; //Pin for coil 2
      ign3.pin = 52; //Pin for coil 3
      ign4.pin = 50; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(3); //The Cam sensor 2 pin
      pinTPS = A2;//TPS input pin
      pinMAP = A3; //MAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A8; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      TachOut.pin = 49; //Tacho output pin  (Goes to ULN2803)
      Idle1.pin = 5; //Single wire idle control
      Idle2.pin = 6; //2 wire idle control
      Boost.pin = 7; //Boost control
      VVT_1.pin = 4; //Default VVT output
      VVT_2.pin = 48; //Default VVT2 output
      FuelPump.pin = 45; //Fuel pump output  (Goes to ULN2803)
      StepperDir.setPin(16); //Direction pin  for DRV8825 driver
      StepperStep.pin = 17; //Step pin for DRV8825 driver
      StepperEnable.pin = 24; //Enable pin for DRV8825
      Fan.pin = 47; //Pin for the fan output (Goes to ULN2803)
      Launch.pin = 51; //Can be overwritten below
      Flex.pin = 2; // Flex sensor (Must be external interrupt enabled)
      ResetControl.pin = 43; //Reset control output
      pinBaro = A5;
      VSS.pin = 20;
      WMIEmpty.pin = 46;
      WMIIndicator.pin = 44;
      WMIEnabled.pin = 42;

#if defined(CORE_TEENSY35)
#if INJ_CHANNELS >= 6
        inj6.pin = 51;
#endif

        Trigger.setPin(23);
        Trigger2.setPin(36);
        StepperDir.setPin(34);
        StepperStep.pin = 35;
        ign1.pin = 31;
        ign2.pin = 32;
        TachOut.pin = 28;
        Fan.pin = 27;
        ign4.pin = 29;
        ign3.pin = 30;
        pinO2 = A22;

        //Make sure the CAN pins aren't overwritten
        Trigger3.setPin(54);
        VVT_1.pin = 55;

#elif defined(CORE_TEENSY41)
        //These are only to prevent lockups or weird behaviour on T4.1 when this board is used as the default
        pinBaro = A4;
        pinMAP = A5;
        pinTPS = A3; //TPS input pin
        pinIAT = A0; //IAT sensor pin
        pinCLT = A1; //CLS sensor pin
        pinO2 = A2; //O2 Sensor pin
        pinBat = A15; //Battery reference voltage pin. Needs Alpha4+
        Launch.pin = 34; //Can be overwritten below
        VSS.pin = 35;

        Trigger.setPin(20); //The CAS pin
        Trigger2.setPin(21); //The Cam Sensor pin
        Trigger3.setPin(23);

        StepperDir.setPin(34);
        StepperStep.pin = 35;

        ign1.pin = 31;
        ign2.pin = 32;
        ign4.pin = 29;
        ign3.pin = 30;

        TachOut.pin = 28;
        Fan.pin = 27;
        FuelPump.pin = 33;
        WMIEmpty.pin = 34;
        WMIIndicator.pin = 35;
        WMIEnabled.pin = 36;
#elif defined(STM32F407xx)
     //Pin definitions for experimental board Tjeerd
        //Black F407VE wiki.stm32duino.com/index.php?title=STM32F407

        //******************************************
        //******** PORTA CONNECTIONS ***************
        //******************************************
        /* = PA0 */ //Wakeup ADC123
        // = PA1;
        // = PA2;
        // = PA3;
        // = PA4;
        /* = PA5; */ //ADC12
        /* = PA6; */ //ADC12 LED_BUILTIN_1
        FuelPump.pin = PA7; //ADC12 LED_BUILTIN_2
        ign3.pin = PA8;
        /* = PA9 */ //TXD1
        /* = PA10 */ //RXD1
        /* = PA11 */ //(DO NOT USE FOR SPEEDUINO) USB
        /* = PA12 */ //(DO NOT USE FOR SPEEDUINO) USB
        /* = PA13 */ //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        /* = PA14 */ //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        /* = PA15 */ //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK

        //******************************************
        //******** PORTB CONNECTIONS ***************
        //******************************************
        /* = PB0; */ //(DO NOT USE FOR SPEEDUINO) ADC123 - SPI FLASH CHIP CS pin
        pinBaro = PB1; //ADC12
        /* = PB2; */ //(DO NOT USE FOR SPEEDUINO) BOOT1
        /* = PB3; */ //(DO NOT USE FOR SPEEDUINO) SPI1_SCK FLASH CHIP
        /* = PB4; */ //(DO NOT USE FOR SPEEDUINO) SPI1_MISO FLASH CHIP
        /* = PB5; */ //(DO NOT USE FOR SPEEDUINO) SPI1_MOSI FLASH CHIP
        /* = PB6; */ //NRF_CE
        /* = PB7; */ //NRF_CS
        /* = PB8; */ //NRF_IRQ
        ign2.pin = PB9; //
        /* = PB9; */ //
        ign4.pin = PB10; //TXD3
        Idle1.pin = PB11; //RXD3
        Idle2.pin = PB12; //
        Boost.pin = PB12; //
        /* = PB13; */ //SPI2_SCK
        /* = PB14; */ //SPI2_MISO
        /* = PB15; */ //SPI2_MOSI

        //******************************************
        //******** PORTC CONNECTIONS ***************
        //******************************************
        pinMAP = PC0; //ADC123
        pinTPS = PC1; //ADC123
        pinIAT = PC2; //ADC123
        pinCLT = PC3; //ADC123
        pinO2 = PC4;  //ADC12
        pinBat = PC5; //ADC12
        VVT_1.pin = PC6; //
        /* = PC8; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D0
        /* = PC9; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D1
        /* = PC10; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D2
        /* = PC11; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D3
        /* = PC12; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_SCK
        TachOut.pin = PC13; //
        /* = PC14; */ //(DO NOT USE FOR SPEEDUINO) - OSC32_IN
        /* = PC15; */ //(DO NOT USE FOR SPEEDUINO) - OSC32_OUT

        //******************************************
        //******** PORTD CONNECTIONS ***************
        //******************************************
        /* = PD0; */ //CANRX
        /* = PD1; */ //CANTX
        /* = PD2; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_CMD
        VVT_2.pin = PD3; //
        Flex.pin = PD4;
        /* = PD5;*/ //TXD2
        /* = PD6; */ //RXD2
        ign1.pin = PD7; //
        /* = PD8; */ //
        ign5.pin = PD9;//
        /* = PD10; */ //
        /* = PD11; */ //
        inj1.pin = PD12; //
        inj2.pin = PD13; //
        inj3.pin = PD14; //
        inj4.pin = PD15; //

        //******************************************
        //******** PORTE CONNECTIONS ***************
        //******************************************
        Trigger.setPin(PE0); //
        Trigger2.setPin(PE1); //
        StepperEnable.pin = PE2; //
        /* = PE3; */ //ONBOARD KEY1
        /* = PE4; */ //ONBOARD KEY2
        StepperStep.pin = PE5; //
        Fan.pin = PE6; //
        StepperDir.setPin(PE7); //
        /* = PE8; */ //
        /* = PE9; */ //
        /* = PE10; */ //
#if INJ_CHANNELS >= 5
        inj5.pin = PE11; //
#endif
#if INJ_CHANNELS >= 6
        inj6.pin = PE12; //
#endif
        /* = PE13; */ //
        /* = PE14; */ //
        /* = PE15; */ //

#elif defined(CORE_STM32)
        //https://github.com/stm32duino/Arduino_Core_STM32/blob/master/variants/Generic_F411Cx/variant.h#L28
        //pins PA12, PA11 are used for USB or CAN couldn't be used for GPIO
        //pins PB12, PB13, PB14 and PB15 are used to SPI FLASH
        //PB2 can't be used as input because it's the BOOT pin
        inj1.pin = PB7; //Output pin injector 1 is on
        inj2.pin = PB6; //Output pin injector 2 is on
        inj3.pin = PB5; //Output pin injector 3 is on
        inj4.pin = PB4; //Output pin injector 4 is on
        ign1.pin = PB9; //Pin for coil 1
        ign2.pin = PB8; //Pin for coil 2
        ign3.pin = PB3; //Pin for coil 3
        ign4.pin = PA15; //Pin for coil 4
        pinTPS = A2;//TPS input pin
        pinMAP = A3; //MAP sensor pin
        pinIAT = A0; //IAT sensor pin
        pinCLT = A1; //CLS sensor pin
        pinO2 = A8; //O2 Sensor pin
        pinBat = A4; //Battery reference voltage pin
        pinBaro = pinMAP;
        TachOut.pin = PB1; //Tacho output pin  (Goes to ULN2803)
        Idle1.pin = PB2; //Single wire idle control
        Idle2.pin = PB10; //2 wire idle control
        Boost.pin = PA6; //Boost control
        StepperDir.setPin(PB10); //Direction pin  for DRV8825 driver
        StepperStep.pin = PB2; //Step pin for DRV8825 driver
        FuelPump.pin = PA8; //Fuel pump output
        Fan.pin = PA5; //Pin for the fan output (Goes to ULN2803)
        //external interrupt enabled pins
        Flex.pin = PC14; // Flex sensor (Must be external interrupt enabled)
        Trigger.setPin(PC13); //The CAS pin also led pin so bad idea
        Trigger2.setPin(PC15); //The Cam Sensor pin
#endif
      break;

    case 6:
      #ifndef SMALL_FLASH_MODE
      //Pin mappings as per the 2001-05 MX5 PNP shield
      inj1.pin = 44; //Output pin injector 1 is on
      inj2.pin = 46; //Output pin injector 2 is on
      inj3.pin = 47; //Output pin injector 3 is on
      inj4.pin = 45; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 14; //Output pin injector 5 is on
#endif

      ign1.pin = 42; //Pin for coil 1
      ign2.pin = 43; //Pin for coil 2
      ign3.pin = 32; //Pin for coil 3
      ign4.pin = 33; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(2); //The Cam sensor 2 pin
      pinTPS = A2;//TPS input pin
      pinMAP = A5; //MAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A3; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      TachOut.pin = 23; //Tacho output pin  (Goes to ULN2803)
      Idle1.pin = 5; //Single wire idle control
      Boost.pin = 4;
      VVT_1.pin = 11; //Default VVT output
      VVT_2.pin = 48; //Default VVT2 output
      Idle2.pin = 4; //2 wire idle control (Note this is shared with boost!!!)
      FuelPump.pin = 40; //Fuel pump output
      StepperDir.setPin(16); //Direction pin  for DRV8825 driver
      StepperStep.pin = 17; //Step pin for DRV8825 driver
      StepperEnable.pin = 24;
      Fan.pin = 41; //Pin for the fan output
      Launch.pin = 12; //Can be overwritten below
      Flex.pin = 3; // Flex sensor (Must be external interrupt enabled)
      ResetControl.pin = 39; //Reset control output
      #endif
      //This is NOT correct. It has not yet been tested with this board
      #if defined(CORE_TEENSY35)
        Trigger.setPin(23);
        Trigger2.setPin(36);
        StepperDir.setPin(34);
        StepperStep.pin = 35;
        ign1.pin = 33; //Done
        ign2.pin = 24; //Done
        ign3.pin = 51; //Won't work (No mapping for pin 32)
        ign4.pin = 52; //Won't work (No mapping for pin 33)
        FuelPump.pin = 26; //Requires PVT4 adapter or above
        Fan.pin = 50; //Won't work (No mapping for pin 35)
        TachOut.pin = 28; //Done
      #endif
      break;

    case 8:
      #ifndef SMALL_FLASH_MODE
      //Pin mappings as per the 1996-97 MX5 PNP shield
      inj1.pin = 11; //Output pin injector 1 is on
      inj2.pin = 10; //Output pin injector 2 is on
      inj3.pin = 9; //Output pin injector 3 is on
      inj4.pin = 8; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 14; //Output pin injector 5 is on
#endif

      ign1.pin = 39; //Pin for coil 1
      ign2.pin = 41; //Pin for coil 2
      ign3.pin = 32; //Pin for coil 3
      ign4.pin = 33; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      pinTPS = A2;//TPS input pin
      pinMAP = A5; //MAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A3; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      TachOut.pin = A9; //Tacho output pin  (Goes to ULN2803)
      Idle1.pin = 2; //Single wire idle control
      Boost.pin = 4;
      Idle2.pin = 4; //2 wire idle control (Note this is shared with boost!!!)
      FuelPump.pin = 49; //Fuel pump output
      StepperDir.setPin(16); //Direction pin  for DRV8825 driver
      StepperStep.pin = 17; //Step pin for DRV8825 driver
      StepperEnable.pin = 24;
      Fan.pin = 35; //Pin for the fan output
      Launch.pin = 37; //Can be overwritten below
      Flex.pin = 3; // Flex sensor (Must be external interrupt enabled)
      ResetControl.pin = 44; //Reset control output

      //This is NOT correct. It has not yet been tested with this board
      #if defined(CORE_TEENSY35)
        Trigger.setPin(23);
        Trigger2.setPin(36);
        StepperDir.setPin(34);
        StepperStep.pin = 35;
        ign1.pin = 33; //Done
        ign2.pin = 24; //Done
        ign3.pin = 51; //Won't work (No mapping for pin 32)
        ign4.pin = 52; //Won't work (No mapping for pin 33)
        FuelPump.pin = 26; //Requires PVT4 adapter or above
        Fan.pin = 50; //Won't work (No mapping for pin 35)
        TachOut.pin = 28; //Done
      #endif
      #endif
      break;

    case 9:
     #ifndef SMALL_FLASH_MODE
      //Pin mappings as per the 89-95 MX5 PNP shield
      inj1.pin = 11; //Output pin injector 1 is on
      inj2.pin = 10; //Output pin injector 2 is on
      inj3.pin = 9; //Output pin injector 3 is on
      inj4.pin = 8; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 14; //Output pin injector 5 is on
#endif

      ign1.pin = 39; //Pin for coil 1
      ign2.pin = 41; //Pin for coil 2
      ign3.pin = 32; //Pin for coil 3
      ign4.pin = 33; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      pinTPS = A2;//TPS input pin
      pinMAP = A5; //MAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A3; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      TachOut.pin = 49; //Tacho output pin  (Goes to ULN2803)
      Idle1.pin = 2; //Single wire idle control
      Boost.pin = 4;
      Idle2.pin = 4; //2 wire idle control (Note this is shared with boost!!!)
      FuelPump.pin = 37; //Fuel pump output
      //Note that there is no stepper driver output on the PNP boards. These pins are unconnected and remain here just to prevent issues with random pin numbers occurring
      StepperEnable.pin = 15; //Enable pin for the DRV8825
      StepperDir.setPin(16); //Direction pin  for DRV8825 driver
      StepperStep.pin = 17; //Step pin for DRV8825 driver
      Fan.pin = 35; //Pin for the fan output
      Launch.pin = 12; //Can be overwritten below
      Flex.pin = 3; // Flex sensor (Must be external interrupt enabled)
      ResetControl.pin = 44; //Reset control output
      VSS.pin = 20;
      IdleUp.pin = 48;
      CTPS.pin = 47;
#endif
#if defined(CORE_TEENSY35)
        Trigger.setPin(23);
        Trigger2.setPin(36);
        StepperDir.setPin(34);
        StepperStep.pin = 35;
        ign1.pin = 33; //Done
        ign2.pin = 24; //Done
        ign3.pin = 51; //Won't work (No mapping for pin 32)
        ign4.pin = 52; //Won't work (No mapping for pin 33)
        FuelPump.pin = 26; //Requires PVT4 adapter or above
        Fan.pin = 50; //Won't work (No mapping for pin 35)
        TachOut.pin = 28; //Done
      #endif
      break;

    case 10:
    #ifndef SMALL_FLASH_MODE //No support for bluepill here anyway
      //Pin mappings for user turtanas PCB
      inj1.pin = 4; //Output pin injector 1 is on
      inj2.pin = 5; //Output pin injector 2 is on
      inj3.pin = 6; //Output pin injector 3 is on
      inj4.pin = 7; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 8; //Placeholder only - NOT USED
#endif
#if INJ_CHANNELS >= 6
      inj6.pin = 9; //Placeholder only - NOT USED
#endif
#if INJ_CHANNELS >= 7
      inj7.pin = 10; //Placeholder only - NOT USED
#endif
#if INJ_CHANNELS >= 8
      inj8.pin = 11; //Placeholder only - NOT USED
#endif
      ign1.pin = 24; //Pin for coil 1
      ign2.pin = 28; //Pin for coil 2
      ign3.pin = 36; //Pin for coil 3
      ign4.pin = 40; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(18); //The CAS pin
      Trigger2.setPin(19); //The Cam Sensor pin
      pinTPS = A2;//TPS input pin
      pinMAP = A3; //MAP sensor pin
      pinMAP2 = A8; //MAP2 sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A4; //O2 Sensor pin
      pinBat = A7; //Battery reference voltage pin
      TachOut.pin = 41; //Tacho output pin transistor is missing 2n2222 for this and 1k for 12v
      FuelPump.pin = 42; //Fuel pump output 2n2222
      Fan.pin = 47; //Pin for the fan output
      TachOut.pin = 49; //Tacho output pin
      Flex.pin = 2; // Flex sensor (Must be external interrupt enabled)
      ResetControl.pin = 26; //Reset control output

    #endif
      break;

    case 20:
    #if defined(CORE_AVR) && !defined(SMALL_FLASH_MODE) //No support for bluepill here anyway
      //Pin mappings as per the Plazomat In/Out shields Rev 0.1
      inj1.pin = 8; //Output pin injector 1 is on
      inj2.pin = 9; //Output pin injector 2 is on
      inj3.pin = 10; //Output pin injector 3 is on
      inj4.pin = 11; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 12; //Output pin injector 5 is on
#endif

      ign1.pin = 28; //Pin for coil 1
      ign2.pin = 24; //Pin for coil 2
      ign3.pin = 40; //Pin for coil 3
      ign4.pin = 36; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(20); //The CAS pin
      Trigger2.setPin(21); //The Cam Sensor pin
      pinO2 = A8; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      pinMAP = A3; //MAP sensor pin
      pinTPS = A2;//TPS input pin
      pinCLT = A1; //CLS sensor pin
      pinIAT = A0; //IAT sensor pin
      Fan.pin = 47; //Pin for the fan output
      FuelPump.pin = 4; //Fuel pump output
      TachOut.pin = 49; //Tacho output pin
      ResetControl.pin = 26; //Reset control output
    #endif
      break;

    case 30:
    #ifndef SMALL_FLASH_MODE //No support for bluepill here anyway
      //Pin mappings as per the dazv6 shield
      inj1.pin = 8; //Output pin injector 1 is on
      inj2.pin = 9; //Output pin injector 2 is on
      inj3.pin = 10; //Output pin injector 3 is on
      inj4.pin = 11; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 12; //Output pin injector 5 is on
#endif

      ign1.pin = 40; //Pin for coil 1
      ign2.pin = 38; //Pin for coil 2
      ign3.pin = 50; //Pin for coil 3
      ign4.pin = 52; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(17); // cam sensor 2 pin, pin17 isn't external trigger enabled in arduino mega??
      pinTPS = A2;//TPS input pin
      pinMAP = A3; //MAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A8; //O2 Sensor pin
      pinO2_2 = A9; //O2 sensor pin (second sensor)
      pinBat = A4; //Battery reference voltage pin
      TachOut.pin = 49; //Tacho output pin
      Idle1.pin = 5; //Single wire idle control
      FuelPump.pin = 45; //Fuel pump output
      StepperDir.setPin(20); //Direction pin  for DRV8825 driver
      StepperStep.pin = 21; //Step pin for DRV8825 driver
      Boost.pin = 7;
      Fan.pin = 47; //Pin for the fan output
    #endif
      break;

   case 31:
      //Pin mappings for the BMW PnP PCBs by pazi88.
      #if defined(CORE_AVR)
      //This is the regular MEGA2560 pin mapping
      inj1.pin = 8; //Output pin injector 1
      inj2.pin = 9; //Output pin injector 2
      inj3.pin = 10; //Output pin injector 3
      inj4.pin = 11; //Output pin injector 4
#if INJ_CHANNELS >= 5
      inj5.pin = 12; //Output pin injector 5
#endif
#if INJ_CHANNELS >= 6
      inj6.pin = 50; //Output pin injector 6
#endif
#if INJ_CHANNELS >= 7
      inj7.pin = 39; //Output pin injector 7 (placeholder)
#endif
#if INJ_CHANNELS >= 8
      inj8.pin = 42; //Output pin injector 8 (placeholder)
#endif
      ign1.pin = 40; //Pin for coil 1
      ign2.pin = 38; //Pin for coil 2
      ign3.pin = 52; //Pin for coil 3
      ign4.pin = 48; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 36; //Pin for coil 5
#endif
#if IGN_CHANNELS >= 6
      ign6.pin = 34; //Pin for coil 6
#endif
#if IGN_CHANNELS >= 7
      ign7.pin = 46; //Pin for coil 7 (placeholder)
#endif
#if IGN_CHANNELS >= 8
      ign8.pin = 53; //Pin for coil 8 (placeholder)
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(20); //The Cam sensor 2 pin
      pinTPS = A2;//TPS input pin
      pinMAP = A3; //MAP sensor pin
      pinEMAP = A15; //EMAP sensor pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLT sensor pin
      pinO2 = A8; //O2 Sensor pin
      pinBat = A4; //Battery reference voltage pin
      pinBaro = A5; //Baro sensor pin
      TachOut.pin = 49; //Tacho output pin  (Goes to ULN2003)
      Idle1.pin = 5; //ICV pin1
      Idle2.pin = 6; //ICV pin3
      Boost.pin = 7; //Boost control
      VVT_1.pin = 4; //VVT1 output (intake vanos)
      VVT_2.pin = 26; //VVT2 output (exhaust vanos)
      FuelPump.pin = 45; //Fuel pump output  (Goes to ULN2003)
      StepperDir.setPin(16); //Stepper valve isn't used with these
      StepperStep.pin = 17; //Stepper valve isn't used with these
      StepperEnable.pin = 24; //Stepper valve isn't used with these
      Fan.pin = 47; //Pin for the fan output (Goes to ULN2003)
      Launch.pin = 51; //Launch control pin
      Flex.pin = 2; // Flex sensor
      ResetControl.pin = 43; //Reset control output
      VSS.pin = 3; //VSS input pin
      WMIEmpty.pin = 31; //(placeholder)
      WMIIndicator.pin = 33; //(placeholder)
      WMIEnabled.pin = 35; //(placeholder)
      IdleUp.pin = 37; //(placeholder)
      CTPS.pin = A6; //(placeholder)
#elif defined(STM32F407xx)
      inj1.pin = PB15; //Output pin injector 1
      inj2.pin = PB14; //Output pin injector 2
      inj3.pin = PB12; //Output pin injector 3
      inj4.pin = PB13; //Output pin injector 4
#if INJ_CHANNELS >= 5
      inj5.pin = PA8; //Output pin injector 5
#endif
#if INJ_CHANNELS >= 6
      inj6.pin = PE7; //Output pin injector 6
#endif
#if INJ_CHANNELS >= 7
      inj7.pin = PE13; //Output pin injector 7 (placeholder)
#endif
#if INJ_CHANNELS >= 8
      inj8.pin = PE10; //Output pin injector 8 (placeholder)
#endif
      ign1.pin = PE2; //Pin for coil 1
      ign2.pin = PE3; //Pin for coil 2
      ign3.pin = PC13; //Pin for coil 3
      ign4.pin = PE6; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = PE4; //Pin for coil 5
#endif
#if IGN_CHANNELS >= 6
      ign6.pin = PE5; //Pin for coil 6
#endif
#if IGN_CHANNELS >= 7
      ign7.pin = PE0; //Pin for coil 7 (placeholder)
#endif
#if IGN_CHANNELS >= 8
      ign8.pin = PB9; //Pin for coil 8 (placeholder)
#endif
      Trigger.setPin(PD3); //The CAS pin
      Trigger2.setPin(PD4); //The Cam Sensor pin
      pinTPS = PA2;//TPS input pin
      pinMAP = PA3; //MAP sensor pin
      pinEMAP = PC5; //EMAP sensor pin
      pinIAT = PA0; //IAT sensor pin
      pinCLT = PA1; //CLS sensor pin
      pinO2 = PB0; //O2 Sensor pin
      pinBat = PA4; //Battery reference voltage pin
      pinBaro = PA5; //Baro sensor pin
      TachOut.pin = PE8; //Tacho output pin  (Goes to ULN2003)
      Idle1.pin = PD10; //ICV pin1
      Idle2.pin = PD9; //ICV pin3
      Boost.pin = PD8; //Boost control
      VVT_1.pin = PD11; //VVT1 output (intake vanos)
      VVT_2.pin = PC7; //VVT2 output (exhaust vanos)
      FuelPump.pin = PE11; //Fuel pump output  (Goes to ULN2003)
      StepperDir.setPin(PB10); //Stepper valve isn't used with these
      StepperStep.pin = PB11; //Stepper valve isn't used with these
      StepperEnable.pin = PA15; //Stepper valve isn't used with these
      Fan.pin = PE9; //Pin for the fan output (Goes to ULN2003)
      Launch.pin = PB8; //Launch control pin
      Flex.pin = PD7; // Flex sensor
      ResetControl.pin = PB7; //Reset control output
      VSS.pin = PB6; //VSS input pin
      WMIEmpty.pin = PD15; //(placeholder)
      WMIIndicator.pin = PD13; //(placeholder)
      WMIEnabled.pin = PE15; //(placeholder)
      IdleUp.pin = PE14; //(placeholder)
      CTPS.pin = PA6; //(placeholder)
#endif
      break;

    case 40:
#ifndef SMALL_FLASH_MODE
      //Pin mappings as per the NO2C shield
      inj1.pin = 8; //Output pin injector 1 is on
      inj2.pin = 9; //Output pin injector 2 is on
      inj3.pin = 11; //Output pin injector 3 is on - NOT USED
      inj4.pin = 12; //Output pin injector 4 is on - NOT USED
#if INJ_CHANNELS >= 5
      inj5.pin = 13; //Placeholder only - NOT USED
#endif

      ign1.pin = 23; //Pin for coil 1
      ign2.pin = 22; //Pin for coil 2
      ign3.pin = 2; //Pin for coil 3 - ONLY WITH DB2
      ign4.pin = 3; //Pin for coil 4 - ONLY WITH DB2
#if IGN_CHANNELS >= 5
      ign5.pin = 46; //Placeholder only - NOT USED
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(21); //The Cam sensor 2 pin
      pinTPS = A3; //TPS input pin
      pinMAP = A0; //MAP sensor pin
      pinIAT = A5; //IAT sensor pin
      pinCLT = A4; //CLT sensor pin
      pinO2 = A2; //O2 sensor pin
      pinBat = A1; //Battery reference voltage pin
      pinBaro = A6; //Baro sensor pin - ONLY WITH DB
      TachOut.pin = 38; //Tacho output pin
      Idle1.pin = 5; //Single wire idle control
      Idle2.pin = 47; //2 wire idle control - NOT USED
      Boost.pin = 7; //Boost control
      VVT_1.pin = 6; //Default VVT output
      VVT_2.pin = 48; //Default VVT2 output
      FuelPump.pin = 4; //Fuel pump output
      StepperDir.setPin(25); //Direction pin for DRV8825 driver
      StepperStep.pin = 24; //Step pin for DRV8825 driver
      StepperEnable.pin = 27; //Enable pin for DRV8825 driver
      Launch.pin = 10; //Can be overwritten below
      Flex.pin = 20; // Flex sensor (Must be external interrupt enabled) - ONLY WITH DB
      Fan.pin = 30; //Pin for the fan output - ONLY WITH DB
      ResetControl.pin = 26; //Reset control output
      #endif
      break;

    case 41:
    #ifndef SMALL_FLASH_MODE //No support for bluepill here anyway
      //Pin mappings as per the UA4C shield
      inj1.pin = 8; //Output pin injector 1 is on
      inj2.pin = 7; //Output pin injector 2 is on
      inj3.pin = 6; //Output pin injector 3 is on
      inj4.pin = 5; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
      inj5.pin = 45; //Output pin injector 5 is on PLACEHOLDER value for now
#endif

      ign1.pin = 35; //Pin for coil 1
      ign2.pin = 36; //Pin for coil 2
      ign3.pin = 33; //Pin for coil 3
      ign4.pin = 34; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 44; //Pin for coil 5 PLACEHOLDER value for now
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(3); //The Cam sensor 2 pin
      Flex.pin = 20; // Flex sensor
      pinTPS = A3; //TPS input pin
      pinMAP = A0; //MAP sensor pin
      pinBaro = A7; //Baro sensor pin
      pinIAT = A5; //IAT sensor pin
      pinCLT = A4; //CLS sensor pin
      pinO2 = A1; //O2 Sensor pin
      pinO2_2 = A9; //O2 sensor pin (second sensor)
      pinBat = A2; //Battery reference voltage pin
      Launch.pin = 37; //Can be overwritten below
      TachOut.pin = 22; //Tacho output pin
      Idle1.pin = 9; //Single wire idle control
      Idle2.pin = 10; //2 wire idle control
      FuelPump.pin = 23; //Fuel pump output
      VVT_1.pin = 11; //Default VVT output
      VVT_2.pin = 48; //Default VVT2 output
      StepperDir.setPin(32); //Direction pin  for DRV8825 driver
      StepperStep.pin = 31; //Step pin for DRV8825 driver
      StepperEnable.pin = 30; //Enable pin for DRV8825 driver
      Boost.pin = 12; //Boost control
      Fan.pin = 24; //Pin for the fan output
      ResetControl.pin = 46; //Reset control output PLACEHOLDER value for now
    #endif
      break;

    case 42:
      //Pin mappings for all BlitzboxBL49sp variants
      inj1.pin = 6; //Output pin injector 1
      inj2.pin = 7; //Output pin injector 2
      inj3.pin = 8; //Output pin injector 3
      inj4.pin = 9; //Output pin injector 4
      ign1.pin = 24; //Pin for coil 1
      ign2.pin = 25; //Pin for coil 2
      ign3.pin = 23; //Pin for coil 3
      ign4.pin = 22; //Pin for coil 4
      Trigger.setPin(19); //The CRANK Sensor pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Flex.pin = 20; // Flex sensor PLACEHOLDER value for now
      pinTPS = A0; //TPS input pin
      pinO2 = A2; //O2 Sensor pin
      pinIAT = A3; //IAT sensor pin
      pinCLT = A4; //CLT sensor pin
      pinMAP = A7; //internal MAP sensor
      pinBat = A6; //Battery reference voltage pin
      pinBaro = A5; //external MAP/Baro sensor pin
      pinO2_2 = A9; //O2 sensor pin (second sensor) PLACEHOLDER value for now
      Launch.pin = 2; //Can be overwritten below
      TachOut.pin = 10; //Tacho output pin
      Idle1.pin = 11; //Single wire idle control
      Idle2.pin = 14; //2 wire idle control PLACEHOLDER value for now
      FuelPump.pin = 3; //Fuel pump output
      VVT_1.pin = 15; //Default VVT output PLACEHOLDER value for now
      Boost.pin = 13; //Boost control
      Fan.pin = 12; //Pin for the fan output
      ResetControl.pin = 46; //Reset control output PLACEHOLDER value for now
    break;

    case 45:
    #ifndef SMALL_FLASH_MODE //No support for bluepill here anyway
      //Pin mappings for the DIY-EFI CORE4 Module. This is an AVR only module
      #if defined(CORE_AVR)
      inj1.pin = 10; //Output pin injector 1 is on
      inj2.pin = 11; //Output pin injector 2 is on
      inj3.pin = 12; //Output pin injector 3 is on
      inj4.pin = 9; //Output pin injector 4 is on
      ign1.pin = 39; //Pin for coil 1
      ign2.pin = 29; //Pin for coil 2
      ign3.pin = 28; //Pin for coil 3
      ign4.pin = 27; //Pin for coil 4
#if IGN_CHANNELS >= 5
      ign5.pin = 26; //Placeholder  for coil 5
#endif
      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(21);// The Cam sensor 2 pin
      Flex.pin = 20; // Flex sensor
      pinTPS = A3; //TPS input pin
      pinMAP = A2; //MAP sensor pin
      pinBaro = A15; //Baro sensor pin
      pinIAT = A11; //IAT sensor pin
      pinCLT = A4; //CLS sensor pin
      pinO2 = A12; //O2 Sensor pin
      pinO2_2 = A5; //O2 sensor pin (second sensor)
      pinBat = A1; //Battery reference voltage pin
      Launch.pin = 24; //Can be overwritten below
      TachOut.pin = 38; //Tacho output pin
      Idle1.pin = 42; //Single wire idle control
      Idle2.pin = 43; //2 wire idle control
      FuelPump.pin = 41; //Fuel pump output
      VVT_1.pin = 44; //Default VVT output
      VVT_2.pin = 48; //Default VVT2 output
      StepperDir.setPin(32); //Direction pin  for DRV8825 driver
      StepperStep.pin = 31; //Step pin for DRV8825 driver
      StepperEnable.pin = 30; //Enable pin for DRV8825 driver
      Boost.pin = 45; //Boost control
#if INJ_CHANNELS >= 5
      inj5.pin = 33; //Output pin injector 5 is on
#endif
#if INJ_CHANNELS >= 6
      inj6.pin = 34; //Output pin injector 6 is on
#endif
      Fan.pin = 40; //Pin for the fan output
      ResetControl.pin = 46; //Reset control output PLACEHOLDER value for now
      #endif
    #endif
      break;

    #if defined(CORE_TEENSY35)
    case 50:
      //Pin mappings as per the teensy rev A shield
      inj1.pin = 2; //Output pin injector 1 is on
      inj2.pin = 10; //Output pin injector 2 is on
      inj3.pin = 6; //Output pin injector 3 is on
      inj4.pin = 9; //Output pin injector 4 is on

      ign1.pin = 29; //Pin for coil 1
      ign2.pin = 30; //Pin for coil 2
      ign3.pin = 31; //Pin for coil 3 - ONLY WITH DB2
      ign4.pin = 32; //Pin for coil 4 - ONLY WITH DB2
      Trigger.setPin(23); //The CAS pin
      Trigger2.setPin(36); //The Cam Sensor pin
      pinTPS = 16; //TPS input pin
      pinMAP = 17; //MAP sensor pin
      pinIAT = 14; //IAT sensor pin
      pinCLT = 15; //CLT sensor pin
      pinO2 = A22; //O2 sensor pin
      pinO2_2 = A21; //O2 sensor pin (second sensor)
      pinBat = 18; //Battery reference voltage pin
      TachOut.pin = 20; //Tacho output pin
      Idle1.pin = 5; //Single wire idle control
      Boost.pin = 11; //Boost control
      FuelPump.pin = 38; //Fuel pump output
      StepperDir.setPin(34); //Direction pin for DRV8825 driver
      StepperStep.pin = 35; //Step pin for DRV8825 driver
      StepperEnable.pin = 33; //Enable pin for DRV8825 driver
      Launch.pin = 26; //Can be overwritten below
      Fan.pin = 37; //Pin for the fan output - ONLY WITH DB
      break;

    case 51:
      //Pin mappings as per the teensy revB board shield
      inj1.pin = 2; //Output pin injector 1 is on
      inj2.pin = 10; //Output pin injector 2 is on
      inj3.pin = 6; //Output pin injector 3 is on - NOT USED
      inj4.pin = 9; //Output pin injector 4 is on - NOT USED
      ign1.pin = 29; //Pin for coil 1
      ign2.pin = 30; //Pin for coil 2
      ign3.pin = 31; //Pin for coil 3 - ONLY WITH DB2
      ign4.pin = 32; //Pin for coil 4 - ONLY WITH DB2
      Trigger.setPin(23); //The CAS pin
      Trigger2.setPin(36); //The Cam Sensor pin
      pinTPS = 16; //TPS input pin
      pinMAP = 17; //MAP sensor pin
      pinIAT = 14; //IAT sensor pin
      pinCLT = 15; //CLT sensor pin
      pinO2 = A22; //O2 sensor pin
      pinO2_2 = A21; //O2 sensor pin (second sensor)
      pinBat = 18; //Battery reference voltage pin
      TachOut.pin = 20; //Tacho output pin
      Idle1.pin = 5; //Single wire idle control
      Boost.pin = 11; //Boost control
      FuelPump.pin = 38; //Fuel pump output
      StepperDir.setPin(34); //Direction pin for DRV8825 driver
      StepperStep.pin = 35; //Step pin for DRV8825 driver
      StepperEnable.pin = 33; //Enable pin for DRV8825 driver
      Launch.pin = 26; //Can be overwritten below
      Fan.pin = 37; //Pin for the fan output - ONLY WITH DB
      break;
    #endif

    #if defined(CORE_TEENSY35)
    case 53:
      //Pin mappings for the Juice Box (ignition only board)
      inj1.pin = 2; //Output pin injector 1 is on - NOT USED
      inj2.pin = 56; //Output pin injector 2 is on - NOT USED
      inj3.pin = 6; //Output pin injector 3 is on - NOT USED
      inj4.pin = 50; //Output pin injector 4 is on - NOT USED
      ign1.pin = 29; //Pin for coil 1
      ign2.pin = 30; //Pin for coil 2
      ign3.pin = 31; //Pin for coil 3
      ign4.pin = 32; //Pin for coil 4
      Trigger.setPin(37); //The CAS pin
      Trigger2.setPin(38); //The Cam Sensor pin - NOT USED
      pinTPS = A2; //TPS input pin
      pinMAP = A7; //MAP sensor pin
      pinIAT = A1; //IAT sensor pin
      pinCLT = A5; //CLT sensor pin
      pinO2 = A0; //O2 sensor pin
      pinO2_2 = A21; //O2 sensor pin (second sensor) - NOT USED
      pinBat = A6; //Battery reference voltage pin
      TachOut.pin = 28; //Tacho output pin
      Idle1.pin = 5; //Single wire idle control - NOT USED
      Boost.pin = 11; //Boost control - NOT USED
      FuelPump.pin = 24; //Fuel pump output
      StepperDir.setPin(3); //Direction pin for DRV8825 driver - NOT USED
      StepperStep.pin = 4; //Step pin for DRV8825 driver - NOT USED
      StepperEnable.pin = 6; //Enable pin for DRV8825 driver - NOT USED
      Launch.pin = 26; //Can be overwritten below
      Fan.pin = 25; //Pin for the fan output
      break;
    #endif

    case 55:
      #if defined(CORE_TEENSY)
      //Pin mappings for the DropBear
      injectorControlMethodAssign(OUTPUT_CONTROL_MC33810);
      ignitionControlMethodAssign(OUTPUT_CONTROL_MC33810);

      //The injector pins below are not used directly as the control is via SPI through the MC33810s, however the pin numbers are set to be the SPI pins (SCLK, MOSI, MISO and CS) so that nothing else will set them as inputs
      inj1.pin = 13; //SCLK
      inj2.pin = 11; //MOSI
      inj3.pin = 12; //MISO
      inj4.pin = 10; //CS for MC33810 1
#if INJ_CHANNELS >= 5
      inj5.pin = 9; //CS for MC33810 2
#endif
#if INJ_CHANNELS >= 6
      inj6.pin = 9; //CS for MC33810 3
#endif

      //Dummy pins, without thes pin 0 (Serial1 RX) gets overwritten
      ign1.pin = 40;
      ign2.pin = 41;

      Trigger.setPin(19); //The CAS pin
      Trigger2.setPin(18); //The Cam Sensor pin
      Trigger3.setPin(22); //Uses one of the protected spare digitial inputs. This must be set or Serial1 (Pin 0) gets broken
      Flex.pin = A16; // Flex sensor
      pinMAP = A1; //MAP sensor pin
      pinBaro = A0; //Baro sensor pin
      pinBat = A14; //Battery reference voltage pin
      Launch.pin = A15; //Can be overwritten below
      TachOut.pin = 5; //Tacho output pin
      Idle1.pin = 27; //Single wire idle control
      Idle2.pin = 29; //2 wire idle control. Shared with Spare 1 output
      FuelPump.pin = 8; //Fuel pump output
      VVT_1.pin = 28; //Default VVT output
      StepperDir.setPin(32); //Direction pin  for DRV8825 driver
      StepperStep.pin = 31; //Step pin for DRV8825 driver
      StepperEnable.pin = 30; //Enable pin for DRV8825 driver
      Boost.pin = 24; //Boost control
      Fan.pin = 25; //Pin for the fan output
      ResetControl.pin = 46; //Reset control output PLACEHOLDER value for now

#if defined(CORE_TEENSY35)
        pinTPS = A22; //TPS input pin
        pinIAT = A19; //IAT sensor pin
        pinCLT = A20; //CLS sensor pin
        pinO2 = A21; //O2 Sensor pin
        pinO2_2 = A18; //Spare 2

        pSecondarySerial = &Serial1; //Header that is broken out on Dropbear boards is attached to Serial1
#endif

#if defined(CORE_TEENSY41)
        pinTPS = A17; //TPS input pin
        pinIAT = A14; //IAT sensor pin
        pinCLT = A15; //CLS sensor pin
        pinO2 = A16; //O2 Sensor pin
        pinBat = A3; //Battery reference voltage pin. Needs Alpha4+

        //New pins for the actual T4.1 version of the Dropbear
        pinBaro = A4;
        pinMAP = A5;
        pinTPS = A3; //TPS input pin
        pinIAT = A0; //IAT sensor pin
        pinCLT = A1; //CLS sensor pin
        pinO2 = A2; //O2 Sensor pin
        pinBat = A15; //Battery reference voltage pin. Needs Alpha4+
        Launch.pin = 36;
        Flex.pin = 37; // Flex sensor

        Trigger.setPin(20); //The CAS pin
        Trigger2.setPin(21); //The Cam Sensor pin

        FuelPump.pin = 5; //Fuel pump output
        TachOut.pin = 8; //Tacho output pin

        ResetControl.pin = 49; //PLaceholder only. Cannot use 42-47 as these are the SD card

#endif

      MC33810_1_CS.pin= 10;
      MC33810_2_CS.pin = 9;

      //Pin alignment to the MC33810 outputs
      MC33810_BIT_INJ1 = 3;
      MC33810_BIT_INJ2 = 1;
      MC33810_BIT_INJ3 = 0;
      MC33810_BIT_INJ4 = 2;
      MC33810_BIT_IGN1 = 4;
      MC33810_BIT_IGN2 = 5;
      MC33810_BIT_IGN3 = 6;
      MC33810_BIT_IGN4 = 7;

      MC33810_BIT_INJ5 = 3;
      MC33810_BIT_INJ6 = 1;
      MC33810_BIT_INJ7 = 0;
      MC33810_BIT_INJ8 = 2;
      MC33810_BIT_IGN5 = 4;
      MC33810_BIT_IGN6 = 5;
      MC33810_BIT_IGN7 = 6;
      MC33810_BIT_IGN8 = 7;



      #endif
      break;

    case 56:
      #if defined(CORE_TEENSY)
      //Pin mappings for the Bear Cub (Teensy 4.1)
      inj1.pin = 6;
      inj2.pin = 7;
      inj3.pin = 9;
      inj4.pin = 8;
#if INJ_CHANNELS >= 5
      inj5.pin = 0; //Not used
#endif

      ign1.pin = 2;
      ign2.pin = 3;
      ign3.pin = 4;
#if IGN_CHANNELS >= 5
      ign4.pin = 5;
#endif

      Trigger.setPin(20); //The CAS pin
      Trigger2.setPin(21); //The Cam Sensor pin
      Flex.pin = 37; // Flex sensor
      pinMAP = A5; //MAP sensor pin
      pinBaro = A4; //Baro sensor pin
      pinBat = A15; //Battery reference voltage pin
      pinTPS = A3; //TPS input pin
      pinIAT = A0; //IAT sensor pin
      pinCLT = A1; //CLS sensor pin
      pinO2 = A2; //O2 Sensor pin
      Launch.pin = 36;

      TachOut.pin = 38; //Tacho output pin
      Idle1.pin = 27; //Single wire idle control
      Idle2.pin = 26; //2 wire idle control. Shared with Spare 1 output
      FuelPump.pin = 10; //Fuel pump output
      VVT_1.pin = 28; //Default VVT output
      StepperDir.setPin(32); //Direction pin  for DRV8825 driver
      StepperStep.pin = 31; //Step pin for DRV8825 driver
      StepperEnable.pin = 30; //Enable pin for DRV8825 driver
      Boost.pin = 24; //Boost control
      Fan.pin = 25; //Pin for the fan output
      ResetControl.pin = 46; //Reset control output PLACEHOLDER value for now

      #endif
      break;


    case 60:
        #if defined(STM32F407xx)
        //Pin definitions for experimental board Tjeerd
        //Black F407VE wiki.stm32duino.com/index.php?title=STM32F407
        //https://github.com/Tjeerdie/SPECTRE/tree/master/SPECTRE_V0.5

        //******************************************
        //******** PORTA CONNECTIONS ***************
        //******************************************
        // = PA0; //Wakeup ADC123
        // = PA1; //ADC123
        // = PA2; //ADC123
        // = PA3; //ADC123
        // = PA4; //ADC12
        // = PA5; //ADC12
        // = PA6; //ADC12 LED_BUILTIN_1
        // = PA7; //ADC12 LED_BUILTIN_2
        ign3.pin = PA8;
        // = PA9;  //TXD1=Bluetooth module
        // = PA10; //RXD1=Bluetooth module
        // = PA11; //(DO NOT USE FOR SPEEDUINO) USB
        // = PA12; //(DO NOT USE FOR SPEEDUINO) USB
        // = PA13;  //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        // = PA14;  //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        // = PA15;  //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK

        //******************************************
        //******** PORTB CONNECTIONS ***************
        //******************************************
        // = PB0;  //(DO NOT USE FOR SPEEDUINO) ADC123 - SPI FLASH CHIP CS pin
        pinBaro = PB1; //ADC12
        // = PB2;  //(DO NOT USE FOR SPEEDUINO) BOOT1
        // = PB3;  //(DO NOT USE FOR SPEEDUINO) SPI1_SCK FLASH CHIP
        // = PB4;  //(DO NOT USE FOR SPEEDUINO) SPI1_MISO FLASH CHIP
        // = PB5;  //(DO NOT USE FOR SPEEDUINO) SPI1_MOSI FLASH CHIP
        // = PB6;  //NRF_CE
        // = PB7;  //NRF_CS
        // = PB8;  //NRF_IRQ
        ign2.pin = PB9; //
        // = PB9;  //
        // = PB10; //TXD3
        // = PB11; //RXD3
        // = PB12; //
        // = PB13;  //SPI2_SCK
        // = PB14;  //SPI2_MISO
        // = PB15;  //SPI2_MOSI

        //******************************************
        //******** PORTC CONNECTIONS ***************
        //******************************************
        pinIAT = PC0; //ADC123
        pinTPS = PC1; //ADC123
        pinMAP = PC2; //ADC123
        pinCLT = PC3; //ADC123
        pinO2 = PC4; //ADC12
        pinBat = PC5;  //ADC12
        Boost.pin = PC6; //
        Idle1.pin = PC7; //
        // = PC8;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D0
        // = PC9;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D1
        // = PC10;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D2
        // = PC11;  //(DO NOT USE FOR SPEEDUINO) - SDIO_D3
        // = PC12;  //(DO NOT USE FOR SPEEDUINO) - SDIO_SCK
        TachOut.pin = PC13; //
        // = PC14;  //(DO NOT USE FOR SPEEDUINO) - OSC32_IN
        // = PC15;  //(DO NOT USE FOR SPEEDUINO) - OSC32_OUT

        //******************************************
        //******** PORTD CONNECTIONS ***************
        //******************************************
        // = PD0;  //CANRX
        // = PD1;  //CANTX
        // = PD2;  //(DO NOT USE FOR SPEEDUINO) - SDIO_CMD
        Idle2.pin = PD3; //
        // = PD4;  //
        Flex.pin = PD4;
        // = PD5; //TXD2
        // = PD6;  //RXD2
        ign1.pin = PD7; //
        // = PD7;  //
        // = PD8;  //
#if IGN_CHANNELS >= 5
        ign5.pin = PD9;//
#endif
        ign4.pin = PD10;//
        // = PD11;  //
        inj1.pin = PD12; //
        inj2.pin = PD13; //
        inj3.pin = PD14; //
        inj4.pin = PD15; //

        //******************************************
        //******** PORTE CONNECTIONS ***************
        //******************************************
        Trigger.setPin(PE0); //
        Trigger2.setPin(PE1); //
        StepperEnable.pin = PE2; //
        FuelPump.pin = PE3; //ONBOARD KEY1
        // = PE4;  //ONBOARD KEY2
        StepperStep.pin = PE5; //
        Fan.pin = PE6; //
        StepperDir.setPin(PE7); //
        // = PE8;  //
#if INJ_CHANNELS >= 5
        inj5.pin = PE9; //
#endif
        // = PE10;  //
#if INJ_CHANNELS >= 6
        inj6.pin = PE11; //
#endif
        // = PE12; //
#if INJ_CHANNELS >= 8
        inj8.pin = PE13; //
#endif
#if INJ_CHANNELS >= 7
        inj7.pin = PE14; //
#endif
        // = PE15;  //
     #elif (defined(STM32F411xE) || defined(STM32F401xC))
        //pins PA12, PA11 are used for USB or CAN couldn't be used for GPIO
        //PB2 can't be used as input because is BOOT pin
        inj1.pin = PB7; //Output pin injector 1 is on
        inj2.pin = PB6; //Output pin injector 2 is on
        inj3.pin = PB5; //Output pin injector 3 is on
        inj4.pin = PB4; //Output pin injector 4 is on
        ign1.pin = PB9; //Pin for coil 1
        ign2.pin = PB8; //Pin for coil 2
        ign3.pin = PB3; //Pin for coil 3
        ign4.pin = PA15; //Pin for coil 4
        pinTPS = A2;//TPS input pin
        pinMAP = A3; //MAP sensor pin
        pinIAT = A0; //IAT sensor pin
        pinCLT = A1; //CLS sensor pin
        pinO2 = A8; //O2 Sensor pin
        pinBat = A4; //Battery reference voltage pin
        pinBaro = pinMAP;
        TachOut.pin = PB1; //Tacho output pin  (Goes to ULN2803)
        Idle1.pin = PB2; //Single wire idle control
        Idle2.pin = PB10; //2 wire idle control
        Boost.pin = PA6; //Boost control
        StepperDir.setPin(PB10); //Direction pin  for DRV8825 driver
        StepperStep.pin = PB2; //Step pin for DRV8825 driver
        FuelPump.pin = PA8; //Fuel pump output
        Fan.pin = PA5; //Pin for the fan output (Goes to ULN2803)

        //external interrupt enabled pins
        Flex.pin = PC14; // Flex sensor (Must be external interrupt enabled)
        Trigger.setPin(PC13); //The CAS pin also led pin so bad idea
        Trigger2.setPin(PC15); //The Cam Sensor pin

     #elif defined(CORE_STM32)
        //blue pill wiki.stm32duino.com/index.php?title=Blue_Pill
        //Maple mini wiki.stm32duino.com/index.php?title=Maple_Mini
        //pins PA12, PA11 are used for USB or CAN couldn't be used for GPIO
        //PB2 can't be used as input because is BOOT pin
        inj1.pin = PB7; //Output pin injector 1 is on
        inj2.pin = PB6; //Output pin injector 2 is on
        inj3.pin = PB5; //Output pin injector 3 is on
        inj4.pin = PB4; //Output pin injector 4 is on
        ign1.pin = PB3; //Pin for coil 1
        ign2.pin = PA15; //Pin for coil 2
        ign3.pin = PA14; //Pin for coil 3
        ign4.pin = PA9; //Pin for coil 4
#if IGN_CHANNELS >= 5
        ign5.pin = PA8; //Pin for coil 5
#endif
        pinTPS = A0; //TPS input pin
        pinMAP = A1; //MAP sensor pin
        pinIAT = A2; //IAT sensor pin
        pinCLT = A3; //CLS sensor pin
        pinO2 = A4; //O2 Sensor pin
        pinBat = A5; //Battery reference voltage pin
        pinBaro = pinMAP;
        Idle1.pin = PB2; //Single wire idle control
        Idle2.pin = PA2; //2 wire idle control
        Boost.pin = PA1; //Boost control
        VVT_1.pin = PA0; //Default VVT output
        VVT_2.pin = PA2; //Default VVT2 output
        StepperDir.setPin(PC15); //Direction pin  for DRV8825 driver
        StepperStep.pin = PC14; //Step pin for DRV8825 driver
        StepperEnable.pin = PC13; //Enable pin for DRV8825
        Fan.pin = PB1; //Pin for the fan output
        FuelPump.pin = PB11; //Fuel pump output
        TachOut.pin = PB10; //Tacho output pin
        //external interrupt enabled pins
        Flex.pin = PB8; // Flex sensor (Must be external interrupt enabled)
        Trigger.setPin(PA10); //The CAS pin
        Trigger2.setPin(PA13); //The Cam Sensor pin

    #endif
      break;

    case BOARD_ID_RUSEFI_FRANKENSO_STM32_F407_DISC:
#if defined(STM32F407xx)
      //******************************************
      //******** PORTA CONNECTIONS ***************
      //******************************************
      pinMAP = PA0;
      // = PA1;
      pinTPS = PA2;
      // = PA3;
      // = PA4;
      Trigger.setPin(PA5);
      // = PA6;
      // = PA7; // ALT crank
      // = PA8; // Thermocouple #3 /CS
      // = PA9;
      // = PA10
      // = PA11;
      // = PA12;
      // = PA13 // Thermocouple #4 /CS
      // = PA14 // Thermocouple #2 /CS
      // = PA15

      //******************************************
      //******** PORTB CONNECTIONS ***************
      //******************************************
      // PB0; Knock sensor /CS
      // PB1;
      // = PB2;
      // = PB3;
      // = PB4; USB SPI MISO
      // = PB5; USB SPI MOSI
      // = PB6; CAN TX
      // = PB7; Injector 12
      // = PB8; Injector 11
      // = PB9;
      // = PB10;
      // PB11; Knock sensor INT
      // = PB12; CAN RX
      /* = PB13; */ // Knock sensor SPI2_SCK
      /* = PB14; */ // Knock sensor SPI2_MISO
      /* = PB15; */ // Knock sensor SPI2_MOSI

      //******************************************
      //******** PORTC CONNECTIONS ***************
      //******************************************
      // = PC0; Knock sensor output.
      pinIAT = PC1;
      pinCLT = PC2;
      pinO2 = PC3;
      pinBat = PC4;
      // = PC5;
      Trigger2.setPin(PC6);
      ign1.pin = PC7;
      /* = PC8; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D0
      ign3.pin = PC9;
      /* = PC10; */ // USART TX
      /* = PC11; */ // USART RX
      /* = PC12; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_SCK
      inj4.pin = PC13;
      /* = PC14; */ //(DO NOT USE FOR SPEEDUINO) - OSC32_IN
      /* = PC15; */ //(DO NOT USE FOR SPEEDUINO) - OSC32_OUT

      //******************************************
      //******** PORTD CONNECTIONS ***************
      //******************************************
      /* = PD0; */ //CANRX
      /* = PD1; */ //CANTX
      /* = PD2; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_CMD
#if INJ_CHANNELS >= 7
      inj7.pin = PD3;
#endif
      /* = PD4; */ // USB SD /CS
      // = PD5; // Injector 10
      // = PD6;
      inj3.pin = PD7; //
#if IGN_CHANNELS >= 7
      ign7.pin = PD8;
#endif
#if IGN_CHANNELS >= 8
      ign8.pin = PD9;
#endif
      /* = PD10; */ //
      /* = PD11; */ //
      /* = PD12; */ // Thermocouple #1 /CS
      /* = PD13; */ //
      /* = PD14; */ //
      /* = PD15; */ //

      //******************************************
      //******** PORTE CONNECTIONS ***************
      //******************************************
      // = PE0; //
      // = PE1; //
#if INJ_CHANNELS >= 8
      inj8.pin = PE2;
#endif
#if INJ_CHANNELS >= 5
      inj5.pin = PE3;
#endif
#if INJ_CHANNELS >= 6
      inj6.pin = PE4;
#endif
      inj2.pin = PE5;
      inj1.pin = PE6;
      // = PE7; //
#if IGN_CHANNELS >= 5
      ign5.pin = PE8;
#endif
      /* = PE9; */ //
#if IGN_CHANNELS >= 6
      ign6.pin = PE10;
#endif
      // = PE11;
      ign4.pin = PE12;
      /* = PE13; */ //
      ign2.pin = PE14;
      /* = PE15; */ //
#endif
      break;

    default:
      #if defined(STM32F407xx)
      //Pin definitions for experimental board Tjeerd
        //Black F407VE wiki.stm32duino.com/index.php?title=STM32F407

        //******************************************
        //******** PORTA CONNECTIONS ***************
        //******************************************
        /* = PA0 */ //Wakeup ADC123
        // = PA1;
        // = PA2;
        // = PA3;
        // = PA4;
        /* = PA5; */ //ADC12
        FuelPump.pin = PA6; //ADC12 LED_BUILTIN_1
        /* = PA7; */ //ADC12 LED_BUILTIN_2
        ign3.pin = PA8;
        /* = PA9 */ //TXD1
        /* = PA10 */ //RXD1
        /* = PA11 */ //(DO NOT USE FOR SPEEDUINO) USB
        /* = PA12 */ //(DO NOT USE FOR SPEEDUINO) USB
        /* = PA13 */ //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        /* = PA14 */ //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK
        /* = PA15 */ //(DO NOT USE FOR SPEEDUINO) NOT ON GPIO - DEBUG ST-LINK

        //******************************************
        //******** PORTB CONNECTIONS ***************
        //******************************************
        /* = PB0; */ //(DO NOT USE FOR SPEEDUINO) ADC123 - SPI FLASH CHIP CS pin
        pinBaro = PB1; //ADC12
        /* = PB2; */ //(DO NOT USE FOR SPEEDUINO) BOOT1
        /* = PB3; */ //(DO NOT USE FOR SPEEDUINO) SPI1_SCK FLASH CHIP
        /* = PB4; */ //(DO NOT USE FOR SPEEDUINO) SPI1_MISO FLASH CHIP
        /* = PB5; */ //(DO NOT USE FOR SPEEDUINO) SPI1_MOSI FLASH CHIP
        /* = PB6; */ //NRF_CE
        /* = PB7; */ //NRF_CS
        /* = PB8; */ //NRF_IRQ
        ign2.pin = PB9; //
        /* = PB9; */ //
        ign4.pin = PB10; //TXD3
        Idle1.pin = PB11; //RXD3
        Idle2.pin = PB12; //
        /* Boost.pin = PB12; */ //
        /* = PB13; */ //SPI2_SCK
        /* = PB14; */ //SPI2_MISO
        /* = PB15; */ //SPI2_MOSI

        //******************************************
        //******** PORTC CONNECTIONS ***************
        //******************************************
        pinMAP = PC0; //ADC123
        pinTPS = PC1; //ADC123
        pinIAT = PC2; //ADC123
        pinCLT = PC3; //ADC123
        pinO2 = PC4; //ADC12
        pinBat = PC5; //ADC12
        /*VVT_1.pin = PC6; */ //
        /* = PC8; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D0
        /* = PC9; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D1
        /* = PC10; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D2
        /* = PC11; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_D3
        /* = PC12; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_SCK
        TachOut.pin = PC13; //
        /* = PC14; */ //(DO NOT USE FOR SPEEDUINO) - OSC32_IN
        /* = PC15; */ //(DO NOT USE FOR SPEEDUINO) - OSC32_OUT

        //******************************************
        //******** PORTD CONNECTIONS ***************
        //******************************************
        /* = PD0; */ //CANRX
        /* = PD1; */ //CANTX
        /* = PD2; */ //(DO NOT USE FOR SPEEDUINO) - SDIO_CMD
        /* = PD3; */ //
        /* = PD4; */ //
        Flex.pin = PD4;
        /* = PD5;*/ //TXD2
        /* = PD6; */ //RXD2
        ign1.pin = PD7; //
        /* = PD7; */ //
        /* = PD8; */ //
#if IGN_CHANNELS >= 5
        ign5.pin = PD9;//
#endif
        /* = PD10; */ //
        /* = PD11; */ //
        inj1.pin = PD12; //
        inj2.pin = PD13; //
        inj3.pin = PD14; //
        inj4.pin = PD15; //

        //******************************************
        //******** PORTE CONNECTIONS ***************
        //******************************************
        Trigger.setPin(PE0); //
        Trigger2.setPin(PE1); //
        StepperEnable.pin = PE2; //
        /* = PE3; */ //ONBOARD KEY1
        /* = PE4; */ //ONBOARD KEY2
        StepperStep.pin = PE5; //
        Fan.pin = PE6; //
        StepperDir.setPin(PE7); //
        /* = PE8; */ //
        /* = PE9; */ //
        /* = PE10; */ //
#if INJ_CHANNELS >= 5
        inj5.pin = PE11; //
#endif
#if INJ_CHANNELS >= 6
        inj6.pin = PE12; //
#endif
        /* = PE13; */ //
        /* = PE14; */ //
        /* = PE15; */ //
#else
#ifndef SMALL_FLASH_MODE //No support for bluepill here anyway
        //Pin mappings as per the v0.2 shield
        inj1.pin = 8; //Output pin injector 1 is on
        inj2.pin = 9; //Output pin injector 2 is on
        inj3.pin = 10; //Output pin injector 3 is on
        inj4.pin = 11; //Output pin injector 4 is on
#if INJ_CHANNELS >= 5
        inj5.pin = 12; //Output pin injector 5 is on
#endif

        ign1.pin = 28; //Pin for coil 1
        ign2.pin = 24; //Pin for coil 2
        ign3.pin = 40; //Pin for coil 3
        ign4.pin = 36; //Pin for coil 4
#if IGN_CHANNELS >= 5
        ign5.pin = 34; //Pin for coil 5 PLACEHOLDER value for now
#endif
        Trigger.setPin(20); //The CAS pin
        Trigger2.setPin(21); //The Cam Sensor pin
        pinTPS = A2; //TPS input pin
        pinMAP = A3; //MAP sensor pin
        pinIAT = A0; //IAT sensor pin
        pinCLT = A1; //CLS sensor pin
        #ifdef A8 //Bit hacky, but needed for the atmega2561
        pinO2 = A8; //O2 Sensor pin
        #endif
        pinBat = A4; //Battery reference voltage pin
        StepperDir.setPin(16); //Direction pin  for DRV8825 driver
        StepperStep.pin = 17; //Step pin for DRV8825 driver
        Fan.pin = 47; //Pin for the fan output
        FuelPump.pin = 4; //Fuel pump output
        TachOut.pin = 49; //Tacho output pin
        Flex.pin = 3; // Flex sensor (Must be external interrupt enabled)
        Boost.pin = 5;
        Idle1.pin = 6;
        ResetControl.pin = 43; //Reset control output
        #endif
      #endif
      break;
  }

  //Setup any devices that are using selectable pins
  setup_selectable_io();

  //Currently there's no default pin for Idle Up
  IdleUp.pin = pinTranslate(configPage2.idleUpPin);

  //Currently there's no default pin for Idle Up Output
  IdleUpOutput.pin = pinTranslate(configPage2.idleUpOutputPin);

  //Currently there's no default pin for closed throttle position sensor
  CTPS.pin = pinTranslate(configPage2.CTPSPin);

  // Air conditioning control initialisation
  if ((configPage15.airConCompPin != 0) && (configPage15.airConCompPin < BOARD_MAX_IO_PINS) ) { AirConComp.pin = pinTranslate(configPage15.airConCompPin); }
  if ((configPage15.airConFanPin != 0) && (configPage15.airConFanPin < BOARD_MAX_IO_PINS) ) { AirConFan.pin = pinTranslate(configPage15.airConFanPin); }
  if ((configPage15.airConReqPin != 0) && (configPage15.airConReqPin < BOARD_MAX_IO_PINS) ) { AirConRequest.pin = pinTranslate(configPage15.airConReqPin); }

  /* Reset control is a special case. If reset control is enabled, it needs its initial state set BEFORE its pinMode.
     If that doesn't happen and reset control is in "Serial Command" mode, the Arduino will end up in a reset loop
     because the control pin will go low as soon as the pinMode is set to OUTPUT. */
  if (configPage4.resetControlConfig != 0 && configPage4.resetControlPin < BOARD_MAX_IO_PINS)
  {
    if (configPage4.resetControlPin != 0U)
    {
      ResetControl.pin = pinTranslate(configPage4.resetControlPin);
    }
    resetControl = configPage4.resetControlConfig;
    setResetControlPinState();
  }

  //Finally, set the relevant pin modes for outputs
  Boost.configure();
  TachOut.configure(HIGH); //Set the tacho output default state

  Idle1.configure();
  Idle2.configure();
  IdleUpOutput.configure();

  FuelPump.configure();
  Fan.configure();

  StepperDir.configure();
  StepperStep.configure();
  StepperEnable.configure();

  VVT_1.configure();
  VVT_2.configure();

  if (configPage4.ignBypassEnabled > 0)
  {
    IgnBypass.configure();
  }

  //This is a legacy mode option to revert the MAP reading behaviour to match
  //what was in place prior to the 201905 firmware
  if(configPage2.legacyMAP > 0)
  {
    digitalWrite(pinMAP, HIGH);
  }

  ignition_pins_init();

  injector_pins_init();

  bool const using_spi = ignitionOutputControl == OUTPUT_CONTROL_MC33810
      || injectorOutputControl == OUTPUT_CONTROL_MC33810;

  if (using_spi)
  {
    bool const builtin_led_used_for_spi =
        LED_BUILTIN == SCK || LED_BUILTIN == MOSI || LED_BUILTIN != MISO;

    if (!builtin_led_used_for_spi)
    {
        //This is required on as the LED pin can otherwise be reset to an input.
        pinMode(LED_BUILTIN, OUTPUT);
    }
  }

//CS pin number is now set in a compile flag.
// #ifdef USE_SPI_EEPROM
//   //We need to send the flash CS (SS) pin if we're using SPI flash. It cannot read from globals.
//   EEPROM.begin(USE_SPI_EEPROM);
// #endif

  //And for inputs
#if defined(CORE_STM32)
#ifdef INPUT_ANALOG
      pinMode(pinMAP, INPUT_ANALOG);
      pinMode(pinO2, INPUT_ANALOG);
      pinMode(pinO2_2, INPUT_ANALOG);
      pinMode(pinTPS, INPUT_ANALOG);
      pinMode(pinIAT, INPUT_ANALOG);
      pinMode(pinCLT, INPUT_ANALOG);
      pinMode(pinBat, INPUT_ANALOG);
      pinMode(pinBaro, INPUT_ANALOG);
#else
      pinMode(pinMAP, INPUT);
      pinMode(pinO2, INPUT);
      pinMode(pinO2_2, INPUT);
      pinMode(pinTPS, INPUT);
      pinMode(pinIAT, INPUT);
      pinMode(pinCLT, INPUT);
      pinMode(pinBat, INPUT);
      pinMode(pinBaro, INPUT);
#endif
#endif

  //Each of the below are only set when their relevant function is enabled.
  //This can help prevent pin conflicts that users aren't aware of with unused functions
  if (configPage2.flexEnabled > 0 && !pinIsOutput(Flex.pin))
  {
    //Standard GM / Continental flex sensor requires pullup, but this should be onboard.
    //The internal pullup will not work (Requires ~3.3k)!
    Flex.configure();
  }

  if (configPage2.vssMode > 1 && !pinIsOutput(VSS.pin)) //Pin mode 1 for VSS is CAN
  {
    VSS.configure();
  }

  if (configPage6.launchEnabled > 0 && !pinIsOutput(Launch.pin))
  {
    byte const input_type = configPage6.lnchPullRes ? INPUT_PULLUP : INPUT;

    Launch.configure(input_type);
  }

  if (configPage2.idleUpEnabled > 0 && !pinIsOutput(IdleUp.pin))
  {
    byte const input_type = (configPage2.idleUpPolarity == 0) ? INPUT_PULLUP : INPUT;

    IdleUp.configure(input_type);
  }

  if (configPage2.CTPSEnabled > 0 && !pinIsOutput(CTPS.pin))
  {
    byte const input_type = (configPage2.CTPSPolarity == 0) ? INPUT_PULLUP : INPUT;

    CTPS.configure(input_type);
  }

  if (configPage10.fuel2Mode == FUEL2_MODE_INPUT_SWITCH && !pinIsOutput(Fuel2Input.pin))
  {
    byte const input_type = configPage10.fuel2InputPullup ? INPUT_PULLUP : INPUT;

    Fuel2Input.configure(input_type);
  }

  if (configPage10.spark2Mode == SPARK2_MODE_INPUT_SWITCH && !pinIsOutput(Spark2Input.pin))
  {
    byte const input_type = configPage10.spark2InputPullup ? INPUT_PULLUP : INPUT;

    Spark2Input.configure(input_type);
  }

  if (configPage10.fuelPressureEnable > 0  && !pinIsOutput(pinFuelPressure))
  {
    pinMode(pinFuelPressure, INPUT);
    FuelPressureEnabled = true;
  }

  if (configPage10.oilPressureEnable > 0 && !pinIsOutput(pinOilPressure))
  {
    pinMode(pinOilPressure, INPUT);
    OilPressureEnabled = true;
  }

  if (configPage13.onboard_log_trigger_Epin > 0 && !pinIsOutput(SDEnable.pin))
  {
    SDEnable.configure();
  }

  if (configPage10.wmiEnabled > 0)
  {
    WMIEnabled.configure();
    if (configPage10.wmiIndicatorEnabled > 0)
    {
      byte const initial_state = (configPage10.wmiIndicatorPolarity > 0) ? HIGH : LOW;

      WMIIndicator.configure(initial_state);
    }
    if (configPage10.wmiEmptyEnabled > 0 && !pinIsOutput(WMIEmpty.pin))
    {
      byte const input_type = (configPage10.wmiEmptyPolarity == 0) ? INPUT_PULLUP : INPUT;

      WMIEmpty.configure(input_type);
    }
  }

  if (configPage15.airConEnable == 1)
  {
    AirConComp.configure();

    if (!pinIsOutput(AirConRequest.pin))
    {
      byte const input_mode = (configPage15.airConReqPol == 1) ? INPUT : INPUT_PULLUP;

      AirConRequest.configure(input_mode);
    }

    if (configPage15.airConFanEnabled == 1)
    {
      AirConFan.configure();
    }
  }
}

/** Initialise the chosen trigger decoder.
 * - Set Interrupt numbers @ref triggerInterrupt, @ref triggerInterrupt2 and @ref triggerInterrupt3  by pin their numbers (based on board CORE_* define)
 * - Call decoder specific setup function triggerSetup_*() (by @ref config4.TrigPattern, set to one of the DECODER_* defines) and do any additional initialisations needed.
 *
 * @todo Explain why triggerSetup_*() alone cannot do all the setup, but there's ~10+ lines worth of extra init for each of decoders.
 */
void initialiseTriggers(void)
{
  byte triggerInterrupt = 0; // By default, use the first interrupt
  byte triggerInterrupt2 = 1;
  byte triggerInterrupt3 = 2;

#if defined(CORE_AVR)
    switch (Trigger.pin)
    {
      //Arduino Mega 2560 mapping
      case 2:
        triggerInterrupt = 0; break;
      case 3:
        triggerInterrupt = 1; break;
      case 18:
        triggerInterrupt = 5; break;
      case 19:
        triggerInterrupt = 4; break;
      case 20:
        triggerInterrupt = 3; break;
      case 21:
        triggerInterrupt = 2; break;
      default:
        triggerInterrupt = 0; break; //This should NEVER happen
    }
#else
    triggerInterrupt = Trigger.pin;
#endif

#if defined(CORE_AVR)
    switch (Trigger2.pin)
    {
      //Arduino Mega 2560 mapping
      case 2:
        triggerInterrupt2 = 0; break;
      case 3:
        triggerInterrupt2 = 1; break;
      case 18:
        triggerInterrupt2 = 5; break;
      case 19:
        triggerInterrupt2 = 4; break;
      case 20:
        triggerInterrupt2 = 3; break;
      case 21:
        triggerInterrupt2 = 2; break;
      default:
        triggerInterrupt2 = 0; break; //This should NEVER happen
    }
#else
    triggerInterrupt2 = Trigger2.pin;
#endif

#if defined(CORE_AVR)
    switch (Trigger3.pin)
    {
      //Arduino Mega 2560 mapping
      case 2:
        triggerInterrupt3 = 0; break;
      case 3:
        triggerInterrupt3 = 1; break;
      case 18:
        triggerInterrupt3 = 5; break;
      case 19:
        triggerInterrupt3 = 4; break;
      case 20:
        triggerInterrupt3 = 3; break;
      case 21:
        triggerInterrupt3 = 2; break;
      default:
        triggerInterrupt3 = 0; break; //This should NEVER happen
    }
#else
    triggerInterrupt3 = Trigger3.pin;
#endif
  Trigger.configure(INPUT);
  Trigger2.configure(INPUT);
  Trigger3.configure(INPUT);

  detachInterrupt(triggerInterrupt);
  detachInterrupt(triggerInterrupt2);
  detachInterrupt(triggerInterrupt3);
  //The default values for edges
  primaryTriggerEdge = 0; //This should ALWAYS be changed below
  secondaryTriggerEdge = 0; //This is optional and may not be changed below, depending on the decoder in use
  tertiaryTriggerEdge = 0; //This is even more optional and may not be changed below, depending on the decoder in use

  //Set the trigger function based on the decoder in the config
  switch (configPage4.TrigPattern)
  {
    case DECODER_MISSING_TOOTH:
      //Missing tooth decoder
      triggerSetup_missingTooth();
      triggerHandler = triggerPri_missingTooth;
      triggerSecondaryHandler = triggerSec_missingTooth;
      triggerTertiaryHandler = triggerThird_missingTooth;

      getRPM = getRPM_missingTooth;
      getCrankAngle = getCrankAngle_missingTooth;
      triggerSetEndTeeth = triggerSetEndTeeth_missingTooth;

      // Attach the crank trigger wheel interrupt
      // (Hall sensor drags to ground when triggering)
      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;
      tertiaryTriggerEdge = (configPage10.TrigEdgeThrd == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);

      if (BIT_CHECK(decoderState, BIT_DECODER_HAS_SECONDARY))
      {
        attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      }
      if (configPage10.vvt2Enabled > 0)
      {
        // we only need this for vvt2, so not really needed if it's not used
        attachInterrupt(triggerInterrupt3, triggerTertiaryHandler, tertiaryTriggerEdge);
      }

      break;

    case DECODER_BASIC_DISTRIBUTOR:
      // Basic distributor
      triggerSetup_BasicDistributor();
      triggerHandler = triggerPri_BasicDistributor;
      getRPM = getRPM_BasicDistributor;
      getCrankAngle = getCrankAngle_BasicDistributor;
      triggerSetEndTeeth = triggerSetEndTeeth_BasicDistributor;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case 2:
      triggerSetup_DualWheel();
      triggerHandler = triggerPri_DualWheel;
      triggerSecondaryHandler = triggerSec_DualWheel;
      getRPM = getRPM_DualWheel;
      getCrankAngle = getCrankAngle_DualWheel;
      triggerSetEndTeeth = triggerSetEndTeeth_DualWheel;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_GM7X:
      triggerSetup_GM7X();
      triggerHandler = triggerPri_GM7X;
      getRPM = getRPM_GM7X;
      getCrankAngle = getCrankAngle_GM7X;
      triggerSetEndTeeth = triggerSetEndTeeth_GM7X;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_4G63:
      triggerSetup_4G63(currentStatus.initialisationComplete);
      triggerHandler = triggerPri_4G63;
      triggerSecondaryHandler = triggerSec_4G63;
      getRPM = getRPM_4G63;
      getCrankAngle = getCrankAngle_4G63;
      triggerSetEndTeeth = triggerSetEndTeeth_4G63;

      primaryTriggerEdge = CHANGE;
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_24X:
      triggerSetup_24X(currentStatus.initialisationComplete);
      triggerHandler = triggerPri_24X;
      triggerSecondaryHandler = triggerSec_24X;
      getRPM = getRPM_24X;
      getCrankAngle = getCrankAngle_24X;
      triggerSetEndTeeth = triggerSetEndTeeth_24X;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = CHANGE; //Secondary is always on every change

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_JEEP2000:
      triggerSetup_Jeep2000(!currentStatus.initialisationComplete);
      triggerHandler = triggerPri_Jeep2000;
      triggerSecondaryHandler = triggerSec_Jeep2000;
      getRPM = getRPM_Jeep2000;
      getCrankAngle = getCrankAngle_Jeep2000;
      triggerSetEndTeeth = triggerSetEndTeeth_Jeep2000;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = CHANGE;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_AUDI135:
      triggerSetup_Audi135();
      triggerHandler = triggerPri_Audi135;
      triggerSecondaryHandler = triggerSec_Audi135;
      getRPM = getRPM_Audi135;
      getCrankAngle = getCrankAngle_Audi135;
      triggerSetEndTeeth = triggerSetEndTeeth_Audi135;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = RISING; //always rising for this trigger

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_HONDA_D17:
      triggerSetup_HondaD17();
      triggerHandler = triggerPri_HondaD17;
      triggerSecondaryHandler = triggerSec_HondaD17;
      getRPM = getRPM_HondaD17;
      getCrankAngle = getCrankAngle_HondaD17;
      triggerSetEndTeeth = triggerSetEndTeeth_HondaD17;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = CHANGE;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_MIATA_9905:
      triggerSetup_Miata9905(currentStatus.initialisationComplete);
      triggerHandler = triggerPri_Miata9905;
      triggerSecondaryHandler = triggerSec_Miata9905;
      getRPM = getRPM_Miata9905;
      getCrankAngle = getCrankAngle_Miata9905;
      triggerSetEndTeeth = triggerSetEndTeeth_Miata9905;

      //These may both need to change, not sure
      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_MAZDA_AU:
      triggerSetup_MazdaAU();
      triggerHandler = triggerPri_MazdaAU;
      triggerSecondaryHandler = triggerSec_MazdaAU;
      getRPM = getRPM_MazdaAU;
      getCrankAngle = getCrankAngle_MazdaAU;
      triggerSetEndTeeth = triggerSetEndTeeth_MazdaAU;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_NON360:
      triggerSetup_non360();
      //Is identical to the dual wheel decoder, so that is used.
      //Same goes for the secondary below
      //Note the use of the Dual Wheel trigger function here.
      //No point in having the same code in twice.
      triggerHandler = triggerPri_DualWheel;
      triggerSecondaryHandler = triggerSec_DualWheel;
      getRPM = getRPM_non360;
      getCrankAngle = getCrankAngle_non360;
      triggerSetEndTeeth = triggerSetEndTeeth_non360;
      // Attach the crank trigger wheel interrupt (Hall sensor drags to ground
      // when triggering)
      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_NISSAN_360:
      triggerSetup_Nissan360();
      triggerHandler = triggerPri_Nissan360;
      triggerSecondaryHandler = triggerSec_Nissan360;
      getRPM = getRPM_Nissan360;
      getCrankAngle = getCrankAngle_Nissan360;
      triggerSetEndTeeth = triggerSetEndTeeth_Nissan360;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = CHANGE;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_SUBARU_67:
      triggerSetup_Subaru67();
      triggerHandler = triggerPri_Subaru67;
      triggerSecondaryHandler = triggerSec_Subaru67;
      getRPM = getRPM_Subaru67;
      getCrankAngle = getCrankAngle_Subaru67;
      triggerSetEndTeeth = triggerSetEndTeeth_Subaru67;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_DAIHATSU_PLUS1:
      triggerSetup_Daihatsu();
      triggerHandler = triggerPri_Daihatsu;
      getRPM = getRPM_Daihatsu;
      getCrankAngle = getCrankAngle_Daihatsu;
      triggerSetEndTeeth = triggerSetEndTeeth_Daihatsu;

      //No secondary input required for this pattern
      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_HARLEY:
      triggerSetup_Harley(currentStatus.initialisationComplete);
      triggerHandler = triggerPri_Harley;
      //triggerSecondaryHandler = triggerSec_Harley;
      getRPM = getRPM_Harley;
      getCrankAngle = getCrankAngle_Harley;
      triggerSetEndTeeth = triggerSetEndTeeth_Harley;

      primaryTriggerEdge = RISING; //Always rising
      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_36_2_2_2:
      //36-2-2-2
      triggerSetup_ThirtySixMinus222();
      triggerHandler = triggerPri_ThirtySixMinus222;
      triggerSecondaryHandler = triggerSec_ThirtySixMinus222;
      getRPM = getRPM_ThirtySixMinus222;
      getCrankAngle = getCrankAngle_missingTooth; //This uses the same function as the missing tooth decoder, so no need to duplicate code
      triggerSetEndTeeth = triggerSetEndTeeth_ThirtySixMinus222;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_36_2_1:
      //36-2-1
      triggerSetup_ThirtySixMinus21();
      triggerHandler = triggerPri_ThirtySixMinus21;
      triggerSecondaryHandler = triggerSec_missingTooth;
      getRPM = getRPM_ThirtySixMinus21;
      getCrankAngle = getCrankAngle_missingTooth; //This uses the same function as the missing tooth decoder, so no need to duplicate code
      triggerSetEndTeeth = triggerSetEndTeeth_ThirtySixMinus21;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_420A:
      //DSM 420a
      triggerSetup_420a();
      triggerHandler = triggerPri_420a;
      triggerSecondaryHandler = triggerSec_420a;
      getRPM = getRPM_420a;
      getCrankAngle = getCrankAngle_420a;
      triggerSetEndTeeth = triggerSetEndTeeth_420a;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = FALLING; //Always falling edge

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_WEBER:
      //Weber-Marelli
      triggerSetup_DualWheel();
      triggerHandler = triggerPri_Webber;
      triggerSecondaryHandler = triggerSec_Webber;
      getRPM = getRPM_DualWheel;
      getCrankAngle = getCrankAngle_DualWheel;
      triggerSetEndTeeth = triggerSetEndTeeth_DualWheel;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_ST170:
      //Ford ST170
      triggerSetup_FordST170();
      triggerHandler = triggerPri_missingTooth;
      triggerSecondaryHandler = triggerSec_FordST170;
      getRPM = getRPM_FordST170;
      getCrankAngle = getCrankAngle_FordST170;
      triggerSetEndTeeth = triggerSetEndTeeth_FordST170;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);

      break;

    case DECODER_DRZ400:
      triggerSetup_DRZ400();
      triggerHandler = triggerPri_DualWheel;
      triggerSecondaryHandler = triggerSec_DRZ400;
      getRPM = getRPM_DualWheel;
      getCrankAngle = getCrankAngle_DualWheel;
      triggerSetEndTeeth = triggerSetEndTeeth_DualWheel;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_NGC:
      //Chrysler NGC - 4, 6 and 8 cylinder
      triggerSetup_NGC();
      triggerHandler = triggerPri_NGC;
      getRPM = getRPM_NGC;
      getCrankAngle = getCrankAngle_missingTooth;
      triggerSetEndTeeth = triggerSetEndTeeth_NGC;

      primaryTriggerEdge = CHANGE;
      if (configPage2.nCylinders == 4) {
        triggerSecondaryHandler = triggerSec_NGC4;
        secondaryTriggerEdge = CHANGE;
      }
      else {
        triggerSecondaryHandler = triggerSec_NGC68;
        secondaryTriggerEdge = FALLING;
      }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_VMAX:
      triggerSetup_Vmax(currentStatus.initialisationComplete);
      triggerHandler = triggerPri_Vmax;
      getRPM = getRPM_Vmax;
      getCrankAngle = getCrankAngle_Vmax;
      triggerSetEndTeeth = triggerSetEndTeeth_Vmax;

      // set as boolean so we can directly use it in decoder.
      primaryTriggerEdge = (configPage4.TrigEdge == 0);

      //Hardcoded change, the primaryTriggerEdge will be used in the decoder to
      //select if it`s an inverted or non-inverted signal.
      attachInterrupt(triggerInterrupt, triggerHandler, CHANGE);
      break;

    case DECODER_RENIX:
      //Renault 44 tooth decoder
      triggerSetup_Renix();
      triggerHandler = triggerPri_Renix;
      getRPM = getRPM_missingTooth;
      getCrankAngle = getCrankAngle_missingTooth;
      triggerSetEndTeeth = triggerSetEndTeeth_Renix;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      /* FIXME: Is secondary edge trigger needed for something? */
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_ROVERMEMS:
      //Rover MEMs - covers multiple flywheel trigger combinations.
      triggerSetup_RoverMEMS();
      triggerHandler = triggerPri_RoverMEMS;
      getRPM = getRPM_RoverMEMS;
      triggerSetEndTeeth = triggerSetEndTeeth_RoverMEMS;

      triggerSecondaryHandler = triggerSec_RoverMEMS;
      getCrankAngle = getCrankAngle_missingTooth;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;
      secondaryTriggerEdge = (configPage4.TrigEdgeSec == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_SUZUKI_K6A:
      triggerSetup_SuzukiK6A();
      // only primary, no secondary, trigger pattern is over 720 degrees
      triggerHandler = triggerPri_SuzukiK6A;
      getRPM = getRPM_SuzukiK6A;
      getCrankAngle = getCrankAngle_SuzukiK6A;
      triggerSetEndTeeth = triggerSetEndTeeth_SuzukiK6A;

      // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    default:
      triggerHandler = triggerPri_missingTooth;
      getRPM = getRPM_missingTooth;
      getCrankAngle = getCrankAngle_missingTooth;

      primaryTriggerEdge = (configPage4.TrigEdge == 0) ? RISING : FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;
  }

#if defined(CORE_TEENSY41)
    //Teensy 4 requires a HYSTERESIS flag to be set on the trigger pins to
    //prevent false interrupts
    setTriggerHysteresis();
#endif
}

static inline bool isAnyFuelScheduleRunning(void) {
  bool anyRunning = false;

  for (size_t i = injChannel1; i < injChannelCount; i++)
  {
    if (injectors.injector((injectorChannelID_t)i).fuelSchedule->Status == RUNNING)
    {
      anyRunning = true;
      break;
    }
  }

  return anyRunning;
}

static inline bool isAnyIgnScheduleRunning(void)
{
  bool anyRunning = false;

  for (size_t i = ignChannel1; i < ignChannelCount; i++)
  {
    if (ignitions.ignition((ignitionChannelID_t)i).ignitionSchedule->Status == RUNNING)
    {
      anyRunning = true;
      break;
    }
  }

  return anyRunning;
}

/** Change injectors or/and ignition angles to 720deg.
 * Roll back req_fuel size and set number of outputs equal to cylinder count.
* */
void changeHalfToFullSync(void)
{
  //Need to do another check for injLayout as this function can be called from ignition
  noInterrupts();
  if (configPage2.injLayout == INJ_SEQUENTIAL
      && CRANK_ANGLE_MAX_INJ != 720
      && !isAnyFuelScheduleRunning())
  {
    CRANK_ANGLE_MAX_INJ = 720;
    req_fuel_uS = req_fuel_init_uS * 2;

    injectors.configure_sequential_injector_schedules(injChannelCount);

    switch (configPage2.nCylinders)
    {
      case 4:
      case 6:
      case 8:
        injectors.setMaxInjectors(configPage2.nCylinders);
        break;

      default:
        break; //No actions required for other cylinder counts

    }
  }
  interrupts();

  //Need to do another check for sparkMode as this function can be called from injection
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
      && CRANK_ANGLE_MAX_IGN != 720
      && !isAnyIgnScheduleRunning())
  {
    CRANK_ANGLE_MAX_IGN = 720;
    ignitions.setMaxIgnitions(configPage2.nCylinders);

    switch (configPage2.nCylinders)
    {
    case 4:
      ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
      break;

    case 6:
      ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
      ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
      break;

    case 8:
      ignitions.configure_coil_schedule(ignChannel1, ignition_id_1);
      ignitions.configure_coil_schedule(ignChannel2, ignition_id_2);
      ignitions.configure_coil_schedule(ignChannel3, ignition_id_3);
      ignitions.configure_coil_schedule(ignChannel4, ignition_id_4);
      break;

    default:
      break; //No actions required for other cylinder counts

    }
  }
}

/** Change injectors or/and ignition angles to 360deg.
 * In semi sequentiol mode req_fuel size is half.
 * Set number of outputs equal to half cylinder count.
* */
void changeFullToHalfSync(void)
{
  if(configPage2.injLayout == INJ_SEQUENTIAL && CRANK_ANGLE_MAX_INJ != 360)
  {
    CRANK_ANGLE_MAX_INJ = 360;
    req_fuel_uS = req_fuel_init_uS;
    switch (configPage2.nCylinders)
    {
      case 4:
        if(configPage4.inj4cylPairing == INJ_PAIR_13_24)
        {
          injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_3);
          injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_4);
        }
        else
        {
          injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_4);
          injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_3);
        }
        injectors.setMaxInjectors(2);
        break;

      case 6:
#if INJ_CHANNELS >= 6
        injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_4);
        injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_5);
        injectors.configure_injector_schedule(injChannel3, injector_id_3, injector_id_6);
        injectors.setMaxInjectors(3);
#endif
        break;

      case 8:
#if INJ_CHANNELS >= 8
        injectors.configure_injector_schedule(injChannel1, injector_id_1, injector_id_5);
        injectors.configure_injector_schedule(injChannel2, injector_id_2, injector_id_6);
        injectors.configure_injector_schedule(injChannel3, injector_id_3, injector_id_7);
        injectors.configure_injector_schedule(injChannel4, injector_id_4, injector_id_8);
        injectors.setMaxInjectors(4);
#endif
        break;
    }
  }

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && CRANK_ANGLE_MAX_IGN != 360)
  {
    CRANK_ANGLE_MAX_IGN = 360;
    ignitions.setMaxIgnitions(configPage2.nCylinders / 2);

    switch (configPage2.nCylinders)
    {
      case 4:
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1, ignition_id_3);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2, ignition_id_4);
        break;

      case 6:
#if IGN_CHANNELS >= 6
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1, ignition_id_4);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2, ignition_id_5);
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_3, ignition_id_6);
#endif
        break;

      case 8:
#if IGN_CHANNELS >= 8
        ignitions.configure_coil_schedule(ignChannel1, ignition_id_1, ignition_id_5);
        ignitions.configure_coil_schedule(ignChannel2, ignition_id_2, ignition_id_6);
        ignitions.configure_coil_schedule(ignChannel3, ignition_id_3, ignition_id_7);
        ignitions.configure_coil_schedule(ignChannel4, ignition_id_4, ignition_id_8);
#endif
        break;
    }
  }
}
