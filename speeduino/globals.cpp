/** @file
 * Instantiation of various (table2D, table3D) tables, volatile (interrupt modified) variables, Injector (1...8) enablement flags, etc.
 */
#include "globals.h"
#include "scheduler.h"
#include "ignition_contexts.h"
#include "auxiliary_pins.h"
#include "ignition_pins.h"
#include "injector_pins.h"

const char TSfirmwareVersion[] PROGMEM = "Speeduino";

const byte data_structure_version = 2; //This identifies the data structure when reading / writing. (outdated ?)

struct table3d16RpmLoad fuelTable; ///< 16x16 fuel map
struct table3d16RpmLoad fuelTable2; ///< 16x16 fuel map
struct table3d16RpmLoad ignitionTable; ///< 16x16 ignition map
struct table3d16RpmLoad ignitionTable2; ///< 16x16 ignition map
struct table3d16RpmLoad afrTable; ///< 16x16 afr target map
struct table3d8RpmLoad stagingTable; ///< 8x8 fuel staging table
struct table3d8RpmLoad boostTable; ///< 8x8 boost map
struct table3d8RpmLoad boostTableLookupDuty; ///< 8x8 boost map lookup table
struct table3d8RpmLoad vvtTable; ///< 8x8 vvt map
struct table3d8RpmLoad vvt2Table; ///< 8x8 vvt2 map
struct table3d8RpmLoad wmiTable; ///< 8x8 wmi map
trimTable3d trim1Table; ///< 6x6 Fuel trim 1 map
trimTable3d trim2Table; ///< 6x6 Fuel trim 2 map
trimTable3d trim3Table; ///< 6x6 Fuel trim 3 map
trimTable3d trim4Table; ///< 6x6 Fuel trim 4 map
trimTable3d trim5Table; ///< 6x6 Fuel trim 5 map
trimTable3d trim6Table; ///< 6x6 Fuel trim 6 map
trimTable3d trim7Table; ///< 6x6 Fuel trim 7 map
trimTable3d trim8Table; ///< 6x6 Fuel trim 8 map
struct table3d4RpmLoad dwellTable; ///< 4x4 Dwell map
struct table2D taeTable; ///< 4 bin TPS Acceleration Enrichment map (2D)
struct table2D maeTable;
struct table2D WUETable; ///< 10 bin Warm Up Enrichment map (2D)
struct table2D ASETable; ///< 4 bin After Start Enrichment map (2D)
struct table2D ASECountTable; ///< 4 bin After Start duration map (2D)
struct table2D PrimingPulseTable; ///< 4 bin Priming pulsewidth map (2D)
struct table2D crankingEnrichTable; ///< 4 bin cranking Enrichment map (2D)
struct table2D dwellVCorrectionTable; ///< 6 bin dwell voltage correction (2D)
struct table2D injectorVCorrectionTable; ///< 6 bin injector voltage correction (2D)
struct table2D injectorAngleTable; ///< 4 bin injector angle curve (2D)
struct table2D IATDensityCorrectionTable; ///< 9 bin inlet air temperature density correction (2D)
struct table2D baroFuelTable; ///< 8 bin baro correction curve (2D)
struct table2D IATRetardTable; ///< 6 bin ignition adjustment based on inlet air temperature  (2D)
struct table2D idleTargetTable; ///< 10 bin idle target table for idle timing (2D)
struct table2D idleAdvanceTable; ///< 6 bin idle advance adjustment table based on RPM difference  (2D)
struct table2D CLTAdvanceTable; ///< 6 bin ignition adjustment based on coolant temperature  (2D)
struct table2D rotarySplitTable; ///< 8 bin ignition split curve for rotary leading/trailing  (2D)
struct table2D flexFuelTable;  ///< 6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D flexAdvTable;   ///< 6 bin flex fuel correction table for timing advance (2D)
struct table2D flexBoostTable; ///< 6 bin flex fuel correction table for boost adjustments (2D)
struct table2D fuelTempTable;  ///< 6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D knockWindowStartTable;
struct table2D knockWindowDurationTable;
struct table2D oilPressureProtectTable;
struct table2D wmiAdvTable; //6 bin wmi correction table for timing advance (2D)
struct table2D coolantProtectTable;
struct table2D fanPWMTable;
struct table2D rollingCutTable;

//These are variables used across multiple files
bool initialisationComplete = false; ///< Tracks whether the setup() function has run completely (true = has run)
uint8_t softLimitTime = 0; //The time (in 0.1 seconds, based on seclx10) that the soft limiter started
volatile uint16_t mainLoopCount; //Main loop counter (incremented at each main loop rev., used for maintaining currentStatus.loopsPerSecond)
uint32_t revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
volatile unsigned long timer5_overflow_count = 0; //Increments every time counter 5 overflows. Used for the fast version of micros()
volatile unsigned long ms_counter = 0; //A counter that increments once per ms
uint16_t fixedCrankingOverride = 0;
bool clutchTrigger;
bool previousClutchTrigger;
volatile uint32_t toothHistory[TOOTH_LOG_SIZE]; ///< Tooth trigger history - delta time (in uS) from last tooth (Indexed by @ref toothHistoryIndex)
volatile uint8_t compositeLogHistory[TOOTH_LOG_SIZE];
volatile bool fpPrimed = false; ///< Tracks whether or not the fuel pump priming has been completed yet
volatile bool injPrimed = false; ///< Tracks whether or not the injectors priming has been completed yet
volatile unsigned int toothHistoryIndex = 0; ///< Current index to @ref toothHistory array
unsigned long currentLoopTime; /**< The time (in uS) that the current mainloop started */
volatile uint16_t ignitionCount; /**< The count of ignition events that have taken place since the engine started */
#if defined(CORE_SAMD21)
  PinStatus primaryTriggerEdge;
  PinStatus secondaryTriggerEdge;
  PinStatus tertiaryTriggerEdge;
#else
  byte primaryTriggerEdge;
  byte secondaryTriggerEdge;
  byte tertiaryTriggerEdge;
#endif
int CRANK_ANGLE_MAX_IGN = 360;
///< The number of crank degrees that the system tracks over.
///Typically 720 divided by the number of squirts per cycle
///(Eg 360 for wasted 2 squirt and 720 for sequential single squirt)
int CRANK_ANGLE_MAX_INJ = 360;
volatile uint32_t runSecsX10;
volatile uint32_t seclx10;
volatile byte HWTest_INJ = 0; /**< Each bit in this variable represents one of the injector channels and it's HW test status */
volatile byte HWTest_INJ_Pulsed = 0; /**< Each bit in this variable represents one of the injector channels and it's pulsed HW test status */
volatile byte HWTest_IGN = 0; /**< Each bit in this variable represents one of the ignition channels and it's HW test status */
volatile byte HWTest_IGN_Pulsed = 0;


//This needs to be here because using the config page directly can prevent burning the setting
byte resetControl = RESET_CONTROL_DISABLED;

volatile byte TIMER_mask;
volatile byte LOOP_TIMER;

/**< Specifies whether the injectors are controlled directly (Via an IO pin)
 *   or using something like the MC33810.
 */
OUTPUT_CONTROL_TYPE injectorOutputControl = OUTPUT_CONTROL_DIRECT;

/**< Specifies whether the coils are controlled directly (via an IO pin)
 *   or using something like the MC33810.
 *   0 = Direct (OUTPUT_CONTROL_DIRECT),
 *   10 = MC33810 (OUTPUT_CONTROL_MC33810)
 */
OUTPUT_CONTROL_TYPE ignitionOutputControl = OUTPUT_CONTROL_DIRECT;

byte pinTPS;      //TPS input pin
byte pinMAP;      //MAP sensor pin
byte pinEMAP;     //EMAP sensor pin
byte pinMAP2;     //2nd MAP sensor (Currently unused)
byte pinIAT;      //IAT sensor pin
byte pinCLT;      //CLS sensor pin
byte pinO2;       //O2 Sensor pin
byte pinO2_2;     //second O2 pin
byte pinBat;      //Battery voltage pin

byte pinIdleUp;   //Input for triggering Idle Up
bool IdleUpEnabled;
byte pinCTPS;     //Input for triggering closed throttle state
bool CTPSEnabled;
byte pinFuel2Input;  //Input for switching to the 2nd fuel table
bool Fuel2InputEnabled;
byte pinSpark2Input; //Input for switching to the 2nd ignition table
bool spark2InputSwitchModeEnabled;
byte pinLaunch;
bool LaunchEnabled;
byte pinVSS;  // VSS (Vehicle speed sensor) Pin
bool VSSEnabled;
byte pinBaro; //Pin that an external barometric pressure sensor is attached to (If used)
byte pinFuelPressure;
bool FuelPressureEnabled;
byte pinOilPressure;
bool OilPressureEnabled;
byte pinWMIEmpty; // Water tank empty sensor
bool WMIEmptyEnabled;

byte pinMC33810_1_CS;
byte pinMC33810_2_CS;
byte pinSDEnable;
bool SDEnableEnabled;

struct statuses currentStatus; /**< The master global "live" status struct. Contains all values that are updated frequently and used across modules */
struct config2 configPage2;
struct config4 configPage4;
struct config6 configPage6;
struct config9 configPage9;
struct config10 configPage10;
struct config13 configPage13;
struct config15 configPage15;

//byte cltCalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the coolant sensor calibration values */
//byte iatCalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the inlet air temperature sensor calibration values */
//byte o2CalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the O2 sensor calibration values */

uint16_t cltCalibration_bins[32];
uint16_t cltCalibration_values[32];
struct table2D cltCalibrationTable;
uint16_t iatCalibration_bins[32];
uint16_t iatCalibration_values[32];
struct table2D iatCalibrationTable;
uint16_t o2Calibration_bins[32];
uint8_t o2Calibration_values[32];
struct table2D o2CalibrationTable;

//These function do checks on a pin to determine if it is already in use by another
//(higher importance) active function
bool pinIsOutput(byte pin)
{
  bool used = false;
  bool isIdlePWM = configPage6.iacAlgorithm > 0
    && (configPage6.iacAlgorithm <= 3 || configPage6.iacAlgorithm == 6);
  bool isIdleSteper = configPage6.iacAlgorithm > 3 && configPage6.iacAlgorithm != 6;
  //Injector?
  if (pin == inj1.pin
  || (pin == inj2.pin && configPage2.nInjectors > 1)
  || (pin == inj3.pin && configPage2.nInjectors > 2)
  || (pin == inj4.pin && configPage2.nInjectors > 3)
#if INJ_CHANNELS >= 5
  || (pin == inj5.pin && configPage2.nInjectors > 4)
#endif
#if INJ_CHANNELS >= 6
  || (pin == inj6.pin && configPage2.nInjectors > 5)
#endif
#if INJ_CHANNELS >= 7
  || (pin == inj7.pin && configPage2.nInjectors > 6)
#endif
#if INJ_CHANNELS >= 8
  || (pin == inj8.pin && configPage2.nInjectors > 7)
#endif
  )
  {
    used = true;
  }

  //Ignition?
  if (pin == ign1.pin
  || (pin == ign2.pin && ignitions.maxOutputs > 1)
  || (pin == ign3.pin && ignitions.maxOutputs > 2)
  || (pin == ign4.pin && ignitions.maxOutputs > 3)
#if IGN_CHANNELS >= 5
  || (pin == ign5.pin && ignitions.maxOutputs > 4)
#endif
#if IGN_CHANNELS >= 6
  || (pin == ign6.pin && ignitions.maxOutputs > 5)
#endif
#if IGN_CHANNELS >= 7
  || (pin == ign7.pin && ignitions.maxOutputs > 6)
#endif
#if IGN_CHANNELS >= 8
  || (pin == ign8.pin && ignitions.maxOutputs > 7)
#endif
  )
  {
    used = true;
  }
  //Functions?
  if (pin == FuelPump.pin
  || (pin == Fan.pin && configPage2.fanEnable == 1)
  || (pin == VVT_1.pin && configPage6.vvtEnabled > 0)
  || (pin == VVT_1.pin && configPage10.wmiEnabled > 0)
  || (pin == VVT_2.pin && configPage10.vvt2Enabled > 0)
  || (pin == Boost.pin && configPage6.boostEnabled == 1)
  || (pin == Idle1.pin && isIdlePWM)
  || (pin == Idle2.pin && isIdlePWM && (configPage6.iacChannels == 1))
  || (pin == StepperEnable.pin && isIdleSteper)
  || (pin == StepperStep.pin && isIdleSteper)
  || (pin == StepperDir.pin && isIdleSteper)
  || (pin == TachOut.pin)
  || (pin == AirConComp.pin && configPage15.airConEnable > 0)
  || (pin == AirConFan.pin && configPage15.airConEnable > 0 && configPage15.airConFanEnabled > 0))
  {
    used = true;
  }
  //Forbidden or hardware reserved? (Defined at board_xyz.h file)
  if (pinIsReserved(pin))
  {
    used = true;
  }

  return used;
}

bool pinIsUsed(byte pin)
{
  bool used = false;

  //Analog input?
  if (pinIsSensor(pin))
  {
    used = true;
  }

  //Functions?
  if (pinIsOutput(pin))
  {
    used = true;
  }

  return used;
}
