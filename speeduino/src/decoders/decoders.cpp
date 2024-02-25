/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
 *
 * Crank and Cam decoders
 *
 * This file contains the various crank and cam wheel decoder functions.
 * Each decoder must have the following 4 functions (Where xxxx is the decoder name):
 *
 * - **triggerSetup_xxxx** - Called once from within setup() and configures any required variables
 * - **triggerPri_xxxx** - Called each time the primary (No. 1) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **triggerSec_xxxx** - Called each time the secondary (No. 2) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **getRPM_xxxx** - Returns the current RPM, as calculated by the decoder
 * - **getCrankAngle_xxxx** - Returns the current crank angle, as calculated by the decoder
 * - **getCamAngle_xxxx** - Returns the current CAM angle, as calculated by the decoder
 *
 * Each decoder must utilise at least the following variables:
 *
 * - toothLastToothTime - The time (In uS) that the last primary tooth was 'seen'
 */

/* Notes on Doxygen Groups/Modules documentation style:
 * - Installing doxygen (e.g. Ubuntu) via pkg mgr: sudo apt-get install doxygen graphviz
 * - @defgroup tag name/description becomes the short name on (Doxygen) "Modules" page
 * - Relying on JAVADOC_AUTOBRIEF (in Doxyfile, essentially automatic @brief), the first sentence (ending with period) becomes
 *   the longer description (second column following name) on (Doxygen) "Modules" page (old Desc: ... could be this sentence)
 * - All the content after first sentence (like old Note:...) is visible on the page linked from the name (1st col) on "Modules" page
 * - To group all decoders together add 1) @defgroup dec Decoders (on top) and 2) "@ingroup dec" to each decoder (under @defgroup)
 * - To compare Speeduino Doxyfile to default config, do: `doxygen -g Doxyfile.default ; diff Doxyfile.default Doxyfile`
 */
#include <limits.h>
#include "globals.h"
#include "decoders.h"
#include "ignition_control.h"
#include "scheduler.h"
#include "crankMaths.h"
#include "timers.h"
#include "schedule_calcs.h"
#include "ignition_contexts.h"
#include "bit_macros.h"
#include "triggers.h"
#include "utilities.h"
#include "null_trigger.h"
#include "missing_tooth.h"

decoder_context_st decoder;

static void primary_isr_handler(void)
{
  decoder.primaryToothHandler();
}

static void secondary_isr_handler(void)
{
  decoder.secondaryToothHandler();
}

static void tertiary_isr_handler(void)
{
  decoder.tertiaryToothHandler();
}

void decoder_context_st::
attach_primary_interrupt(byte pin, trigger_handler_fn handler, interrupt_mode_t edge)
{
  primaryToothHandler = handler;
  attachInterrupt(digitalPinToInterrupt(pin), primary_isr_handler, edge);
}

void decoder_context_st::
attach_secondary_interrupt(byte pin, trigger_handler_fn handler, interrupt_mode_t edge)
{
  secondaryToothHandler = handler;
  attachInterrupt(digitalPinToInterrupt(pin), secondary_isr_handler, edge);
}

void decoder_context_st::
attach_tertiary_interrupt(byte pin, trigger_handler_fn handler, interrupt_mode_t edge)
{
  tertiaryToothHandler = handler;
  attachInterrupt(digitalPinToInterrupt(pin), tertiary_isr_handler, edge);
}

static inline bool HasAnySync(const statuses &status)
{
  return status.hasSync || BIT_CHECK(status.status3, BIT_STATUS3_HALFSYNC);
}

void (*triggerHandler)(void) = nullTriggerHandler; ///Pointer for the trigger function (Gets pointed to the relevant decoder)
void (*triggerSecondaryHandler)(void) = nullTriggerHandler; ///Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
void (*triggerTertiaryHandler)(void) = nullTriggerHandler; ///Pointer for the tertiary trigger function (Gets pointed to the relevant decoder)
uint16_t (*getRPM)(void) = nullGetRPM; ///Pointer to the getRPM function (Gets pointed to the relevant decoder)
int (*getCrankAngle)(void) = nullGetCrankAngle; ///Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
void (*triggerSetEndTeeth)(void) = triggerSetEndTeeth_missingTooth; ///Pointer to the triggerSetEndTeeth function of each decoder

static void triggerRoverMEMSCommon(void);

volatile unsigned long curTime;
volatile unsigned long curGap;
volatile unsigned long curTime2;
volatile unsigned long curGap2;
volatile unsigned long curTime3;
volatile unsigned long curGap3;
volatile unsigned long lastGap;
volatile unsigned long targetGap;

//The maximum time (in uS) that the system will continue to function before the
//engine is considered stalled/stopped. This is unique to each decoder,
//depending on the number of teeth etc. 500000 (half a second) is used as the
//default value, most decoders will be much less.
unsigned long MAX_STALL_TIME = MICROS_PER_SEC / 2U;
//The current number of teeth (Once sync has been achieved, this can never actually be 0)
volatile uint16_t toothCurrentCount = 0;
//Used for decoders such as Audi 135 where not every tooth is used for
//calculating crank angle. This variable stores the actual number of teeth,
//not the number being used to calculate crank angle
volatile byte toothSystemCount = 0;
//As below, but used for decoders where not every tooth count is used for calculation
volatile unsigned long toothSystemLastToothTime = 0;
//The time (micros()) that the last tooth was registered
volatile unsigned long toothLastToothTime = 0;
//The time (micros()) that the last tooth was registered on the secondary input
volatile unsigned long toothLastSecToothTime = 0;
//The time (micros()) that the last tooth was registered on the second cam input
volatile unsigned long toothLastThirdToothTime = 0;
//The time (micros()) that the tooth before the last tooth was registered
volatile unsigned long toothLastMinusOneToothTime = 0;
//The time (micros()) that the tooth before the last tooth was registered on secondary input
volatile unsigned long toothLastMinusOneSecToothTime = 0;
//The time (micros()) that the last tooth rose (used by special decoders to
//determine missing teeth polarity)
volatile unsigned long toothLastToothRisingTime = 0;
//The time (micros()) that the last tooth rose on the secondary input (used by
//special decoders to determine missing teeth polarity)
volatile unsigned long toothLastSecToothRisingTime = 0;
volatile unsigned long targetGap2;
volatile unsigned long targetGap3;
//The time (micros()) that tooth 1 last triggered
volatile unsigned long toothOneTime = 0;
//The 2nd to last time (micros()) that tooth 1 last triggered
volatile unsigned long toothOneMinusOneTime = 0;
// For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)
volatile bool revolutionOne = false;
// used to identify in the rover pattern which has a non unique primary trigger
// something unique - has the secondary tooth changed.
volatile bool revolutionLastOne = false;

//Used for identifying the current secondary (Usually cam) tooth for patterns
//with multiple secondary teeth
volatile unsigned int secondaryToothCount;
// used to identify in the rover pattern which has a non unique primary trigger
// something unique - has the secondary tooth changed.
volatile unsigned int secondaryLastToothCount = 0;
//The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long secondaryLastToothTime = 0;
//The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long secondaryLastToothTime1 = 0;

//Used for identifying the current third (Usually exhaust cam - used for VVT2) tooth for patterns with multiple secondary teeth
volatile unsigned int thirdToothCount;
//The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long thirdLastToothTime = 0;
//The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long thirdLastToothTime1 = 0;

uint16_t triggerActualTeeth;
// The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
volatile unsigned long triggerFilterTime;
// The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
// for the secondary input
volatile unsigned long triggerSecFilterTime;
// The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
// for the Third input
volatile unsigned long triggerThirdFilterTime;

volatile uint8_t decoderState = 0;

UQ24X8_t microsPerDegree;
UQ1X15_t degreesPerMicro;

// The shortest valid time (in uS) pulse DURATION
unsigned int triggerSecFilterTime_duration;
//The number of crank degrees that elapse per tooth
volatile uint16_t triggerToothAngle;
//How many teeth must've been seen on this revolution before we try to confirm sync
//(Useful for missing tooth type decoders)
byte checkSyncToothCount;
unsigned long elapsedTime;
unsigned long lastCrankAngleCalc;
//The time between the vvt reference pulse and the last crank pulse
unsigned long lastVVTtime;

//An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder,
//but may grow later if there are other decoders that use this style
int16_t toothAngles[24];

#ifdef USE_LIBDIVIDE
libdivide::libdivide_s16_t divTriggerToothAngle;
#endif

/** Universal (shared between decoders) decoder routines.
*
* @defgroup dec_uni Universal Decoder Routines
*
* @{
*/
// whichTooth - 0 for Primary (Crank), 1 for Secondary (Cam)

/** Add tooth log entry to toothHistory (array).
 * Enabled by (either) currentStatus.toothLogEnabled and currentStatus.compositeTriggerUsed.
 * @param toothTime - Tooth Time
 * @param whichTooth - 0 for Primary (Crank), 2 for Secondary (Cam) 3 for Tertiary (Cam)
 */
static inline void
addToothLogEntry(unsigned long toothTime, tooth_source_t whichTooth)
{
  if (BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY))
  {
    return;
  }
  //High speed tooth logging history
  if (currentStatus.toothLogEnabled || currentStatus.compositeTriggerUsed > 0)
  {
    bool valueLogged = false;
    if (currentStatus.toothLogEnabled)
    {
      //Tooth log only works on the Crank tooth
      if (whichTooth == TOOTH_CRANK)
      {
        toothHistory[toothHistoryIndex] = toothTime; //Set the value in the log.
        valueLogged = true;
      }
    }
    else if (currentStatus.compositeTriggerUsed > 0)
    {
      compositeLogHistory[toothHistoryIndex] = 0;
      if (currentStatus.compositeTriggerUsed == 4)
      {
        // we want to display both cams so swap the values round to display primary
        // as cam1 and secondary as cam2, include the crank in the data as the third output
        if (Trigger2.read())
        {
          BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI);
        }
        if (Trigger3.read())
        {
          BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC);
        }
        if (Trigger.read())
        {
          BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD);
        }
        if (whichTooth > TOOTH_CAM_SECONDARY)
        {
          BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG);
        }
      }
      else
      {
        // we want to display crank and one of the cams
        if (Trigger.read())
        {
          BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI);
        }
        if (currentStatus.compositeTriggerUsed == 3)
        {
          // display cam2 and also log data for cam 1
          if (Trigger3.read())
          {
            // only the COMPOSITE_LOG_SEC value is visualised hence the swapping of the data
            BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC);
          }
          if (Trigger2.read())
          {
            BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD);
          }
        }
        else
        {
          // display cam1 and also log data for cam 2 - this is the historic composite view
          if (Trigger2.read())
          {
            BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC);
          }
          if (Trigger3.read())
          {
            BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_THIRD);
          }
        }
        if (whichTooth > TOOTH_CRANK)
        {
          BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG);
        }
      }
      if (currentStatus.hasSync)
      {
        BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SYNC);
      }

      if (revolutionOne == 1)
      {
        BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_ENGINE_CYCLE);
      }
      else
      {
        BIT_CLEAR(compositeLogHistory[toothHistoryIndex], COMPOSITE_ENGINE_CYCLE);
      }

      toothHistory[toothHistoryIndex] = micros();
      valueLogged = true;
    }

    //If there has been a value logged above, update the indexes
    if (valueLogged)
    {
      if (toothHistoryIndex < (TOOTH_LOG_SIZE - 1))
      {
        toothHistoryIndex++; BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      }
      else
      {
        BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      }
    }
  } //Tooth/Composite log enabled
}

/** Interrupt handler for primary trigger.
* This function is called on both the rising and falling edges of the primary trigger, when either the
* composite or tooth loggers are turned on.
*/
void loggerPrimaryISR(void)
{
  //This value will be set to the return value of the decoder function,
  //indicating whether or not this pulse passed the filters
  BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER);

  /*
  Need to still call the standard decoder trigger.
  Two checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  If either of these are true, the primary decoder function is called
  */
  bool const trigger_state = Trigger.read();
  bool const valid_edge =
    (primaryTriggerEdge == RISING && trigger_state)
    || (primaryTriggerEdge == FALLING && !trigger_state)
    || primaryTriggerEdge == CHANGE;

  if (valid_edge)
  {
    triggerHandler();
  }

  if (currentStatus.toothLogEnabled
      && BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER))
  {
    //Tooth logger only logs when the edge was correct
    if (valid_edge)
    {
      addToothLogEntry(curGap, TOOTH_CRANK);
    }
  }
  else if (currentStatus.compositeTriggerUsed > 0)
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap, TOOTH_CRANK);
  }
}

/** Interrupt handler for secondary trigger.
* As loggerPrimaryISR, but for the secondary trigger.
*/
void loggerSecondaryISR(void)
{
  //This value will be set to the return value of the decoder function,
  //indicating whether or not this pulse passed the filters
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder function is called
  */
  bool const trigger_state = Trigger2.read();
  bool const valid_edge =
    (secondaryTriggerEdge == RISING && trigger_state)
    || (secondaryTriggerEdge == FALLING && !trigger_state)
    || secondaryTriggerEdge == CHANGE;

  if (valid_edge)
  {
    triggerSecondaryHandler();
  }

  //No tooth logger for the secondary input
  if (currentStatus.compositeTriggerUsed > 0
      && BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER))
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap2, TOOTH_CAM_SECONDARY);
  }
}

/** Interrupt handler for third trigger.
* As loggerPrimaryISR, but for the third trigger.
*/
void loggerTertiaryISR(void)
{
  //This value will be set to the return value of the decoder function,
  //indicating whether or not this pulse passed the filters
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder function is called
  */

  bool const trigger_state = Trigger3.read();
  bool const valid_edge =
    (tertiaryTriggerEdge == RISING && trigger_state)
    || (tertiaryTriggerEdge == FALLING && !trigger_state)
    || tertiaryTriggerEdge == CHANGE;

  if (valid_edge)
  {
    triggerTertiaryHandler();
  }

  //No tooth logger for the secondary input
  if (currentStatus.compositeTriggerUsed > 0
      && BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER))
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap3, TOOTH_CAM_TERTIARY);
  }
}

static inline bool IsCranking(const statuses &status)
{
  return (status.RPM < status.crankRPM) && (status.startRevolutions == 0U);
}

__attribute__((noinline))
bool SetRevolutionTime(uint32_t revTime)
{
  bool const revolution_time_changed = revTime != revolutionTime;

  if (revolution_time_changed)
  {
    revolutionTime = revTime;
    microsPerDegree = div360(revolutionTime << microsPerDegree_Shift);
    degreesPerMicro = (uint16_t)UDIV_ROUND_CLOSEST((UINT32_C(360) << degreesPerMicro_Shift), revolutionTime, uint32_t);
  }

  return revolution_time_changed;
}

bool UpdateRevolutionTimeFromTeeth(bool isCamTeeth)
{
  noInterrupts();

  bool haveUpdatedRevTime = HasAnySync(currentStatus)
    && !IsCranking(currentStatus)
    && (toothOneMinusOneTime != UINT32_C(0))
    && (toothOneTime > toothOneMinusOneTime)
    //The time in uS that one revolution would take at current speed
    //(The time tooth 1 was last seen, minus the time it was seen prior to that)
    && SetRevolutionTime((toothOneTime - toothOneMinusOneTime) >> (isCamTeeth ? 1U : 0U));

  interrupts();

  return haveUpdatedRevTime;
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
__attribute__((noinline))int crankingGetRPM(byte totalTeeth, bool isCamTeeth)
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

/** Compute RPM.
* As nearly all the decoders use a common method of determining RPM
* (The time the last full revolution took), a common function is simpler.
* @param isCamTeeth - Indicates that this is a cam wheel tooth.
* Some patterns have a tooth #1 every crank rev, others are every cam rev.
* @return RPM
*/
__attribute__((noinline)) uint16_t
stdGetRPM(bool isCamTeeth)
{
  if (UpdateRevolutionTimeFromTeeth(isCamTeeth))
  {
    return RpmFromRevolutionTimeUs(revolutionTime);
  }

  return currentStatus.RPM;
}


/** @} */

/*****************************************************************
 * Rover MEMS decoder
 * Covers multiple trigger wheels used interchangeably over the range of MEMS units
 * Specifically covers teeth patterns on the primary trigger (crank)
 * 3 gap 14 gap 2 gap 13 gap
 * 11 gap 5 gap 12 gap 4 gap
 * 2 gap 14 gap 3 gap 13 gap
 * 17 gap 17 gap
 *
 * Support no cam, single tooth Cam (or half moon cam), and multi tooth (5-3-2 teeth)
 *
 * @defgroup dec_rover_mems Rover MEMS all versions including T Series, O Series, Mini and K Series
 * @{
 */
// used for flywheel gap pattern matching
volatile uint32_t roverMEMSTeethSeen = 0;

void triggerSetup_RoverMEMS(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  for (unsigned i = 0; i < 10; i++)
  {
    // Repurpose ToothAngles to store data needed for this implementation.
    toothAngles[i] = 0;
  }

  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be disgarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 36U);
  // only 1 tooth on the wheel not 36
  triggerSecFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U);

  configPage4.triggerTeeth = 36;
  //The number of degrees that passes from tooth to tooth 360 / 36 theortical teeth
  triggerToothAngle = 360 / configPage4.triggerTeeth;
  //The number of physical teeth on the wheel. Need to fix now so we can identify
  //the wheel on the first rotation and not risk a type 1 wheel not being spotted
  triggerActualTeeth = 36;
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0; // current tooth
  secondaryToothCount = 0;
  secondaryLastToothCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  revolutionOne = 0;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_RoverMEMS(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    // have we seen more than 1 tooth so we start processing
    if (toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
    {
      uint32_t const deltaLastToothTime = toothLastToothTime - toothLastMinusOneToothTime;

      //Begin the missing tooth detection
      //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one)
      targetGap = deltaLastToothTime + (deltaLastToothTime >> 1);
      currentStatus.hasSync = true;
      if (curGap > targetGap) // we've found a gap
      {
        // add the space for the gap and the tooth we've just seen so mulitply by 4
        roverMEMSTeethSeen = roverMEMSTeethSeen << 2;
        roverMEMSTeethSeen++; // add the tooth seen to the variable
        // Increment the tooth counter on the wheel
        // (used to spot a revolution and trigger igition timing)
        toothCurrentCount++;

        // the missing tooth gap messing up timing as it appears in different
        // parts of the cycle. Don't update setFilter as it would be wrong with the gap
        toothCurrentCount++;
      }
      else
      {
        //Regular (non-missing) tooth so update things
        // make a space, multiply by 2
        roverMEMSTeethSeen = roverMEMSTeethSeen << 1;
        roverMEMSTeethSeen++; // add the tooth seen
        //Increment the tooth counter on the wheel (used to spot a revolution)
        toothCurrentCount++;
        setFilter(curGap);
      }

      // reduce checks to minimise cpu load when looking for key point to
      // identify where we are on the wheel
      if (toothCurrentCount >= triggerActualTeeth)
      {
                                  //12345678901234567890123456789012
        if (roverMEMSTeethSeen == 0b11111101111111011111111110111111)
        {
          // Binary pattern for trigger pattern 9-7-10-6- (#5)
          if (toothAngles[ID_TOOTH_PATTERN] != 5)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 1;
            toothAngles[SKIP_TOOTH2] = 11;
            toothAngles[SKIP_TOOTH3] = 19;
            toothAngles[SKIP_TOOTH4] = 30;
            toothAngles[ID_TOOTH_PATTERN] = 5;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //123456789012345678901234567890123456
        else if (roverMEMSTeethSeen == 0b11011101111111111111101101111111)
        {
          // Binary pattern for trigger pattern 3-14-2-13- (#4)
          if (toothAngles[ID_TOOTH_PATTERN] != 4)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 8;
            toothAngles[SKIP_TOOTH2] = 11;
            toothAngles[SKIP_TOOTH3] = 25;
            toothAngles[SKIP_TOOTH4] = 27;
            toothAngles[ID_TOOTH_PATTERN] = 4;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //123456789012345678901234567890123456
        else if (roverMEMSTeethSeen == 0b11011011111111111111011101111111)
        {
          // Binary pattern for trigger pattern 2-14-3-13- (#3)
          if (toothAngles[ID_TOOTH_PATTERN] != 3)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 8;
            toothAngles[SKIP_TOOTH2] = 10;
            toothAngles[SKIP_TOOTH3] = 24;
            toothAngles[SKIP_TOOTH4] = 27;
            toothAngles[ID_TOOTH_PATTERN] = 3;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //12345678901234567890123456789012
        else if (roverMEMSTeethSeen == 0b11111101111101111111111110111101)
        {
          // Binary pattern for trigger pattern 11-5-12-4- (#2)
          if (toothAngles[ID_TOOTH_PATTERN] != 2)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 1;
            toothAngles[SKIP_TOOTH2] = 12;
            toothAngles[SKIP_TOOTH3] = 17;
            toothAngles[SKIP_TOOTH4] = 29;
            toothAngles[ID_TOOTH_PATTERN] = 2;
            // this could be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 4;
            triggerActualTeeth = 36; // should be 32 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }                             //12345678901234567890123456789012
        else if (roverMEMSTeethSeen == 0b11111111111101111111111111111101)
        {
          // Binary pattern for trigger pattern 17-17- (#1)
          if (toothAngles[ID_TOOTH_PATTERN] != 1)
          {
            //teeth to skip when calculating RPM as they've just had a gap
            toothAngles[SKIP_TOOTH1] = 1;
            toothAngles[SKIP_TOOTH2] = 18;
            toothAngles[ID_TOOTH_PATTERN] = 1;
            // this should be read in from the config file, but people could adjust it.
            configPage4.triggerMissingTeeth = 2;
            triggerActualTeeth = 36; // should be 34 if not hacking toothcounter
          }
          triggerRoverMEMSCommon();
        }
        else if (toothCurrentCount > triggerActualTeeth + 1)
        {
          // no patterns match after a rotation when we only need 32 teeth to match,
          // we've lost sync
          currentStatus.hasSync = false;
          if (secondaryToothCount > 0)
            BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          else
            BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          currentStatus.syncLossCounter++;
        }
      }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

      crankAngle = ignitionLimits(crankAngle);
      if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && revolutionOne)
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount));
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}


static void triggerRoverMEMSCommon(void)
{
  // pattern 1 isn't unique & if we don't have a cam we need special code to
  // identify if we're tooth 18 or 36 - this allows batch injection but not spark to run
  // as we have to be greater than 18 teeth when using the cam this code also works for that.
  if (toothCurrentCount > 18)
  {
    toothCurrentCount = 1;
    toothOneMinusOneTime = toothOneTime;
    toothOneTime = curTime;
    revolutionOne = !revolutionOne; //Flip sequential revolution tracker
  }

  //if Sequential fuel or ignition is in use, further checks are needed before determining sync
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL || configPage2.injLayout == INJ_SEQUENTIAL)
  {
    //If either fuel or ignition is sequential, only declare sync if the cam tooth
    //has been seen OR if the missing wheel is on the cam
    if (secondaryToothCount > 0 || configPage4.TrigSpeed == CAM_SPEED)
    {
      currentStatus.hasSync = true;
      //the engine is fully synced so clear the Half Sync bit
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
      //Reset the secondary tooth counter to prevent it overflowing
      if (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE)
      {
        secondaryToothCount = 0;
      }
    }
    else if (currentStatus.hasSync != true)
    {
      BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC);
    } //If there is primary trigger but no secondary we only have half sync.
  }
  else
  {
    //If nothing is using sequential, we have sync and also clear half sync bit
    currentStatus.hasSync = true;
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
  }

  if (currentStatus.hasSync)
  {
    currentStatus.startRevolutions++;
  }
  else
  {
    currentStatus.startRevolutions = 0;
  }
}

int getCrankAngle_RoverMEMS(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  bool tempRevolutionOne;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempRevolutionOne = revolutionOne;
  tempToothLastToothTime = toothLastToothTime;

  interrupts();

  //Number of teeth that have passed since tooth 1,
  //multiplied by the angle each tooth represents,
  //plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

  //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
  if (tempRevolutionOne && configPage4.TrigSpeed == CRANK_SPEED)
  {
    crankAngle += 360;
  }

  lastCrankAngleCalc = micros();
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

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

void triggerSec_RoverMEMS(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if ((toothLastSecToothTime == 0))
  {
    targetGap2 = curGap * 2;
    curGap2 = 0;
    toothLastSecToothTime = curTime2;
  }

  if (curGap2 >= triggerSecFilterTime)
  {
    secondaryToothCount++;
    toothLastSecToothTime = curTime2;

    //Record the VVT Angle
    if (configPage6.vvtEnabled > 0
        && (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE ||
            (configPage4.trigPatternSec == SEC_TRIGGER_5_3_2 && secondaryToothCount == 6)))
    {
      int16_t curAngle;

      curAngle = getCrankAngle();
      while (curAngle > 360)
      {
        curAngle -= 360;
      }
      curAngle -= configPage4.triggerAngle; //Value at TDC
      if (configPage6.vvtMode == VVT_MODE_CLOSED_LOOP)
      {
        curAngle -= configPage10.vvtCLMinAng;
      }

      currentStatus.vvt1Angle = curAngle;
    }

    if (configPage4.trigPatternSec == SEC_TRIGGER_SINGLE)
    {
      //Standard single tooth cam trigger
      revolutionOne = true;
      //Next secondary filter is half the current gap
      triggerSecFilterTime = curGap2 >> 1;
    }
    else if (configPage4.trigPatternSec == SEC_TRIGGER_5_3_2) // multi tooth cam
    {
      if (curGap2 < targetGap2) // ie normal tooth sized gap, not a single or double gap
      {
        //Next secondary filter is half the current gap
        triggerSecFilterTime = curGap2 >> 1;
        //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one)
        targetGap2 = curGap2 + (curGap2 >> 1);
      }
      else
      {
        // gap either single or double - nb remember we've got the tooth after the gap,
        // so on the 5 tooth pattern we'll see here tooth 6
        if (secondaryToothCount == 6)
        {
          // if we've got the tooth after the gap from reading 5 teeth we're on
          // cycle 360-720 & tooth 18-36
          revolutionOne = false;
          if (toothCurrentCount < 19)
          {
            toothCurrentCount += 18;
          }
        }
        else if (secondaryToothCount == 4)
        {
          // we've got the tooth after the gap from reading 3 teeth we're on
          // cycle 0-360 & tooth 1-18
          revolutionOne = true;
          if (toothCurrentCount > 17)
          {
            toothCurrentCount -= 18;
          }
        }
        else if (secondaryToothCount == 3)
        {
          // if we've got the tooth after the gap from reading 2 teeth we're on
          // cycle 0-360 & tooth 18-36
          revolutionOne = true;
          if (toothCurrentCount < 19)
          {
            toothCurrentCount += 18;
          }
        }
        // as we've had a gap we need to reset to this being the first tooth after the gap
        secondaryToothCount = 1;
      }
    }
  } //Trigger filter
}

uint16_t getRPM_RoverMEMS(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH1]
        && toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH2]
        && toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH3]
        && toothCurrentCount != (uint16_t)toothAngles[SKIP_TOOTH4])
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM as the missing tooth messes the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

void triggerSetEndTeeth_RoverMEMS(void)
{
  int16_t tempIgnitionEndTooth[5];
  int16_t toothAdder = 0;

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = 36;
  }
  int16_t const toothAdderLimit = 36 + toothAdder;
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
  ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

  tempIgnitionEndTooth[1] = ((ignition1.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[1] > toothAdderLimit)
  {
    tempIgnitionEndTooth[1] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[1] <= 0)
  {
    tempIgnitionEndTooth[1] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[1] > toothAdderLimit)
  {
    tempIgnitionEndTooth[1] = toothAdderLimit;
  }

  tempIgnitionEndTooth[2] = ((ignition2.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[2] > toothAdderLimit)
  {
    tempIgnitionEndTooth[2] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[2] <= 0)
  {
    tempIgnitionEndTooth[2] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[2] > toothAdderLimit)
  {
    tempIgnitionEndTooth[2] = toothAdderLimit;
  }

  tempIgnitionEndTooth[3] = ((ignition3.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[3] > toothAdderLimit)
  {
    tempIgnitionEndTooth[3] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[3] <= 0)
  {
    tempIgnitionEndTooth[3] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[3] > toothAdderLimit)
  {
    tempIgnitionEndTooth[3] = toothAdderLimit;
  }

  tempIgnitionEndTooth[4] = ((ignition4.endAngle - configPage4.triggerAngle) / (int16_t)10) - 1;
  if (tempIgnitionEndTooth[4] > toothAdderLimit)
  {
    tempIgnitionEndTooth[4] -= toothAdderLimit;
  }
  if (tempIgnitionEndTooth[4] <= 0)
  {
    tempIgnitionEndTooth[4] += toothAdderLimit;
  }
  if (tempIgnitionEndTooth[4] > toothAdderLimit)
  {
    tempIgnitionEndTooth[4] = toothAdderLimit;
  }

  // take into account the missing teeth on the Rover flywheels
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    // check the calculated trigger tooth exists, if it doesn't use the previous tooth
    // nb the toothAngles[x] holds the tooth after the gap, hence the '-1' to see if it matches a gap

    for (unsigned tempCount = 1; tempCount < 5; tempCount++)
    {
      if (tempIgnitionEndTooth[tempCount] == toothAngles[1]
          || tempIgnitionEndTooth[tempCount] == toothAngles[2]
          || tempIgnitionEndTooth[tempCount] == toothAngles[3]
          || tempIgnitionEndTooth[tempCount] == toothAngles[4]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[1]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[2]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[3]
          || tempIgnitionEndTooth[tempCount] == 36 + toothAngles[4])
      {
        tempIgnitionEndTooth[tempCount]--;
      }
    }
  }
  else
  {
    for (unsigned tempCount = 1; tempCount < 5; tempCount++)
    {
      if (tempIgnitionEndTooth[tempCount] == toothAngles[1]
          || tempIgnitionEndTooth[tempCount] == toothAngles[2])
      {
        tempIgnitionEndTooth[tempCount]--;
      }
    }
  }

  ignition1.endTooth = tempIgnitionEndTooth[1];
  ignition2.endTooth = tempIgnitionEndTooth[2];
  ignition3.endTooth = tempIgnitionEndTooth[3];
  ignition4.endTooth = tempIgnitionEndTooth[4];
}

/** @} */
/** @} */

/** Suzuki K6A 3 cylinder engine

* (See: https://www.msextra.com/forums/viewtopic.php?t=74614)
* @defgroup Suzuki_K6A Suzuki K6A
* @{
*/
void triggerSetup_SuzukiK6A(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth (primary)
  //- set to a value, needs to be set per tooth
  triggerToothAngle = 90;
  toothCurrentCount = 99; //Fake tooth count represents no sync

  configPage4.TrigSpeed = CAM_SPEED;
  triggerActualTeeth = 7;
  toothCurrentCount = 1;
  curGap = curGap2 = curGap3 = 0;

  //Set a startup value here to avoid filter errors when starting.
  //This MUST have the initial check to prevent the fuel pump just staying on all the time
  if (!currentStatus.initialisationComplete)
  {
    toothLastToothTime = micros();
  }
  else
  {
    toothLastToothTime = 0;
  }
  toothLastMinusOneToothTime = 0;

  // based on data in msextra page linked to above we can deduce,
  // gap between rising and falling edge of a normal 70 degree tooth is 48 degrees,
  // this means the gap is 70 degrees - 48 degrees = 22 degrees.
  // assume this is constant for all similar sized gaps and teeth
  // sync tooth is 35 degrees - eyeball looks like the tooth is 50% tooth and 50% gap
  // so guess its 17 degrees and 18 degrees.

  // coded every tooth here in case you want to try "change" setting on the trigger setup
  // (this is defined in init.ino and what i've set it to, otherwise you need code
  // to select rising or falling in init.ino (steal it from another trigger)).
  // If you don't want change then drop the 'falling' edges listed below and half
  // the number of edges + reduce the triggerActualTeeth
  // nb as you can edit the trigger offset using rising or falling edge setup
  // below is irrelevant as you can adjust via the trigger ofset to cover the difference.

  // not using toothAngles[0] as i'm hoping it makes logic easier

  toothAngles[1] = 0;   // 0 TDC cylinder 1,
  toothAngles[2] = 170; // 170 - end of cylinder 1, start of cylinder 3, trigger ignition for cylinder 3 on this tooth
  toothAngles[3] = 240; // 70 TDC cylinder 3
  toothAngles[4] = 410; // 170  - end of cylinder 3, start of cylinder2, trigger ignition for cylinder 2 on this tooth
  toothAngles[5] = 480; // 70 TDC cylinder 2
  toothAngles[6] = 515; // 35 Additional sync tooth
  toothAngles[7] = 650; // 135 end of cylinder 2, start of cylinder 1, trigger ignition for cylinder 1 on this tooth
  // 70 - gap to rotation to TDC1. array item 1 and 8 are the same,
  // code never gets here its for reference only
  toothAngles[8] = 720;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  MAX_STALL_TIME = 3333UL * triggerToothAngle;
  triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerSecFilterTime = 0; //Need to figure out something better for this
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  // never sure if we need to set this in this type of trigger
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  // we can never have half sync - its either full or none.
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
}

void triggerPri_SuzukiK6A(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  if (curGap >= triggerFilterTime || currentStatus.startRevolutions == 0)
  {
    toothCurrentCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    // now to figure out if its a normal tooth or the extra sync tooth
    // pattern is normally small tooth, big tooth, small tooth, big tooth.
    // The extra tooth breaks the pattern so it goes, big tooth (curGap3),
    // small tooth(curGap2), small tooth(curGap)
    // reuse curGap2 and curGap3 (from secondary and tertiary decoders) to store
    // previous tooth sizes as not needed in this decoder.

    if (curGap <= curGap2 && curGap2 <= curGap3)
    {
      // cur Gap is smaller than last gap & last gap is smaller than gap before that
      // - means we must be on sync tooth
      toothCurrentCount = 6; // set tooth counter to correct tooth
      currentStatus.hasSync = true;
    }

    curGap3 = curGap2; // update values for next time we're in the loop
    curGap2 = curGap;

    if (toothCurrentCount == triggerActualTeeth + 1 && currentStatus.hasSync)
    {
      // seen enough teeth to have a revolution of the crank
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      // increment for 2 revs as we do 720 degrees on the the crank
      currentStatus.startRevolutions = currentStatus.startRevolutions + 2;
    }
    else if (toothCurrentCount > triggerActualTeeth + 1)
    {
      // Lost sync
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      triggerFilterTime = 0;
      toothCurrentCount = 0;
    }

    // check gaps match with tooth to check we have sync
    // so if we *think* we've seen tooth 3 whos gap should be smaller than the
    // previous tooth & it isn't, then we've lost sync
    switch (toothCurrentCount)
    {
    case 1:
    case 3:
    case 5:
    case 6:
      // current tooth gap is bigger than previous tooth gap = syncloss
      // eg tooth 3 should be smaller than tooth 2 gap, if its not then we've
      // lost sync and the tooth 3 we've just seen isn't really tooth 3
      if (curGap > curGap2)
      {
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerFilterTime = 0;
        toothCurrentCount = 2;
      }
      break;

    case 2:
    case 4:
    case 7:
      // current tooth gap is smaller than the previous tooth gap = syncloss
      // eg tooth 2 should be bigger than tooth 1, if its not then we've got syncloss
      if (curGap < curGap2)
      {
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        triggerFilterTime = 0;
        toothCurrentCount = 1;
      }
      break;
    }

    // Setup data to allow other areas of the system to work due to odd sized teeth
    // - this could be merged with sync checking above, left separate to keep
    // code clearer as its doing only one function at once
    // % of filter are not based on previous tooth size but expected next tooth size
    // triggerToothAngle is the size of the prevous tooth not the future tooth
    if (currentStatus.hasSync)
    {
      // Set tooth angle based on previous gap and triggerFilterTime based on
      // previous gap and next gap
      switch (toothCurrentCount)
      {
      case 2:
      case 4:
        // equivalent of tooth 1 except we've not done rotation code yet so its 8
        // 170 degree tooth, next tooth is 70
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 17 degrees
          triggerFilterTime = curGap >> 3;
          break;

        case 2: // 50 % 35 degrees
          triggerFilterTime = (curGap >> 3) + (curGap >> 4);
          break;

        case 3: // 75 % 52 degrees
          triggerFilterTime = (curGap >> 2) + (curGap >> 4);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 5:
        // 70 degrees, next tooth is 35
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 8 degrees
          triggerFilterTime = curGap >> 3;
          break;

        case 2: // 50 % 17 degrees
          triggerFilterTime = curGap >> 2;
          break;

        case 3: // 75 % 25 degrees
          triggerFilterTime = (curGap >> 2) + (curGap >> 3);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 6:
        // sync tooth, next tooth is 135
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 33 degrees
          triggerFilterTime = curGap;
          break;

        case 2: // 50 % 67 degrees
          triggerFilterTime = curGap * 2;
          break;

        case 3: // 75 % 100 degrees
          triggerFilterTime = curGap * 3;
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 7:
        // 135 degree tooth, next tooth is 70
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 17 degrees
          triggerFilterTime = curGap >> 3;
          break;

        case 2: // 50 % 35 degrees
          triggerFilterTime = curGap >> 2;
          break;

        case 3: // 75 % 52 degrees
          triggerFilterTime = (curGap >> 2) + (curGap >> 3);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      case 1:
      case 3:
        // 70 degree tooth, next tooth is 170
        switch (configPage4.triggerFilter)
        {
        case 1: // 25 % 42 degrees
          triggerFilterTime = (curGap >> 1) + (curGap >> 3);
          break;

        case 2: // 50 % 85 degrees
          triggerFilterTime = curGap + (curGap >> 2);
          break;

        case 3: // 75 % 127 degrees
          triggerFilterTime = curGap + (curGap >> 1) + (curGap >> 2);
          break;

        default:
          triggerFilterTime = 0;
          break;
        }
        break;

      }

      //NEW IGNITION MODE
      if (configPage2.perToothIgn)
      {
        int16_t crankAngle = toothAngles[toothCurrentCount] + configPage4.triggerAngle;

        crankAngle = ignitionLimits(crankAngle);
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    } // has sync
  } //Trigger filter
}

void triggerSec_SuzukiK6A(void)
{
  return;
}

uint16_t getRPM_SuzukiK6A(void)
{
  //Cranking code needs working out.

  uint16_t tempRPM;

  tempRPM = stdGetRPM(720);
  //Set the stall time to be twice the current RPM.
  //This is a safe figure as there should be no single revolution where this
  //changes more than this
  MAX_STALL_TIME = revolutionTime << 1;
  if (MAX_STALL_TIME < 366667UL) //Check for 50rpm minimum
  {
    MAX_STALL_TIME = 366667UL;
  }

  return tempRPM;
}

int getCrankAngle_SuzukiK6A(void)
{
  int crankAngle = 0;

  //This is the current angle ATDC the engine is at.
  //This is the last known position based on what tooth was last 'seen'.
  //It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.
  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Perform a lookup of the fixed toothAngles array to find what the angle of
  //the last tooth passed was.
  crankAngle = toothAngles[tempToothCurrentCount] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;

  switch (toothCurrentCount)
  {
  case 2:
  case 4:
    // equivalent of tooth 1 except we've not done rotation code yet so its 8
    // 170 degree tooth, next tooth is 70
    triggerToothAngle = 170;
    break;

  case 5:
    // 70 degrees, next tooth is 35
    triggerToothAngle = 70;
    break;

  case 6:
    // sync tooth, next tooth is 135
    triggerToothAngle = 35;
    break;

  case 7:
    // 135 degree tooth, next tooth is 70
    triggerToothAngle = 135;
    break;

  case 1:
  case 3:
    // 70 degree tooth, next tooth is 170
    triggerToothAngle = 70;
    break;
  }
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);
  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle < 0)
  {
    crankAngle += 720;
  }

  return crankAngle;
}

// Assumes no advance greater than 48 degrees.
// Triggers on the tooth before the ignition event
void triggerSetEndTeeth_SuzukiK6A(void)
{
  byte nCount, bExit;

  //Temp variables are used here to avoid potential issues if a trigger interrupt
  //occurs part way through this function
  int16_t tempIgnitionEndTooth;

  tempIgnitionEndTooth = ignitions.ignition(ignChannel1).endAngle - configPage4.triggerAngle;
  tempIgnitionEndTooth = ignitionLimits(tempIgnitionEndTooth);

  for (nCount = 1, bExit = false; nCount < 8 && !bExit; nCount++)
  {
    if (tempIgnitionEndTooth <= toothAngles[nCount])
    {
      // The tooth we want is the tooth prior to this one.
      tempIgnitionEndTooth = nCount - 1;
      if (tempIgnitionEndTooth <= 0)
      {
        tempIgnitionEndTooth = 7;
      }
      bExit = true;
    }
  }
  if (nCount == 8)
  {
    // didn't find a match, use tooth 7 as it must be greater than 7 but less than 1.
    tempIgnitionEndTooth = 7;
  }
  ignitions.ignition(ignChannel1).endTooth = tempIgnitionEndTooth;

  tempIgnitionEndTooth = ignitions.ignition(ignChannel2).endAngle - configPage4.triggerAngle;
  tempIgnitionEndTooth = ignitionLimits(tempIgnitionEndTooth);

  for (nCount = 1, bExit = false; nCount < 8 && !bExit; nCount++)
  {
    if (tempIgnitionEndTooth <= toothAngles[nCount])
    {
      // The tooth we want is the tooth prior to this one.
      tempIgnitionEndTooth = nCount - 1;
      if (tempIgnitionEndTooth <= 0)
      {
        tempIgnitionEndTooth = 7;
      }
      bExit = true; // force exit from loop
    }
  }
  if (nCount == 8)
  {
    // didn't find a match, use tooth 7 as it must be greater than 7 but less than 1.
    tempIgnitionEndTooth = 7;
  }

  ignitions.ignition(ignChannel2).endTooth = tempIgnitionEndTooth;

  tempIgnitionEndTooth = ignitions.ignition(ignChannel3).endAngle - configPage4.triggerAngle;
  tempIgnitionEndTooth = ignitionLimits(tempIgnitionEndTooth);

  for (nCount = 1, bExit = false; nCount < 8 && !bExit; nCount++)
  {
    if (tempIgnitionEndTooth <= toothAngles[nCount])
    {
      // The tooth we want is the tooth prior to this one.
      tempIgnitionEndTooth = nCount - 1;
      if (tempIgnitionEndTooth <= 0)
      {
        tempIgnitionEndTooth = 7;
      }
      bExit = true; // force exit from loop
    }
  }

  if (nCount == 8)
  {
    // didn't find a match, use tooth 7 as it must be greater than 7 but less than 1.
    tempIgnitionEndTooth = 7;
  }
  ignitions.ignition(ignChannel1).endTooth = tempIgnitionEndTooth;
}
/** @} */

