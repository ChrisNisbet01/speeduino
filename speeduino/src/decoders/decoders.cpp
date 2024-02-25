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

/** @} */

/** Miata '99 to '05 with 4x 70 degree duration teeth running at cam speed.
Teeth believed to be at the same angles as the 4g63 decoder.
Tooth #1 is defined as the next crank tooth after the crank signal is HIGH when*
*the cam signal is falling.
Tooth number one is at 355* ATDC.
* (See: www.forum.diyefi.org/viewtopic.php?f=56&t=1077)
* @defgroup miata_99_05 Miata '99 to '05
* @{
*/
void triggerSetup_Miata9905(bool const initialisationComplete)
{
  triggerToothAngle = 90; //The number of degrees that passes from tooth to tooth (primary)
  toothCurrentCount = 99; //Fake tooth count represents no sync
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  triggerActualTeeth = 8;

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on all the time
    secondaryToothCount = 0;
    toothLastToothTime = micros();
  }
  else
  {
    toothLastToothTime = 0;
  }
  toothLastMinusOneToothTime = 0;

  //Note that these angles are for every rising and falling edge
  toothAngles[0] = 710; //
  toothAngles[1] = 100; //First crank pulse after the SINGLE cam pulse
  toothAngles[2] = 170; //
  toothAngles[3] = 280; //
  toothAngles[4] = 350; //
  toothAngles[5] = 460; //First crank pulse AFTER the DOUBLE cam pulse
  toothAngles[6] = 530; //
  toothAngles[7] = 640; //

  unsigned const minimum_rpm = 50;

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
  //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerFilterTime = 1500;
  triggerSecFilterTime = 0; //Need to figure out something better for this
  BIT_SET(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_Miata9905(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  if (curGap >= triggerFilterTime || currentStatus.startRevolutions == 0)
  {
    toothCurrentCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    if (toothCurrentCount == triggerActualTeeth + 1)
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if (!currentStatus.hasSync || configPage4.useResync)
      {
        if (secondaryToothCount == 2)
        {
          toothCurrentCount = 6;
          currentStatus.hasSync = true;
        }
      }
    }

    if (currentStatus.hasSync)
    {
      //Whilst this is an uneven tooth pattern, if the specific angle between
      //the last 2 teeth is specified, 1st deriv prediction can be used
      if (configPage4.triggerFilter == 1 || currentStatus.RPM < 1400)
      {
        //Lite filter
        //Trigger filter is set to whatever time it took to do 70 degrees
        //(Next trigger is 110 degrees away)
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7)
        {
          triggerToothAngle = 70;
          triggerFilterTime = curGap;
        }
        else
        {
          //Trigger filter is set to (110*3)/8=41.25=41 degrees
          //(Next trigger is 70 degrees away).
          triggerToothAngle = 110;
          triggerFilterTime = (curGap * 3) >> 3;
        }
      }
      else if (configPage4.triggerFilter == 2)
      {
        //Medium filter level
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7) //87.5 degrees with a target of 110
        {
          triggerToothAngle = 70;
          triggerFilterTime = (curGap * 5) >> 2;
        }
        else //55 degrees with a target of 70
        {
          triggerToothAngle = 110;
          triggerFilterTime = (curGap >> 1);
        }
      }
      else if (configPage4.triggerFilter == 3)
      {
        //Aggressive filter level
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7) //96.26 degrees with a target of 110
        {
          triggerToothAngle = 70;
          triggerFilterTime = (curGap * 11) >> 3;
        }
        else //61.87 degrees with a target of 70
        {
          triggerToothAngle = 110;
          triggerFilterTime = (curGap * 9) >> 5;
        }
      }
      else if (configPage4.triggerFilter == 0)
      {
        //trigger filter is turned off.
        triggerFilterTime = 0;
        triggerSecFilterTime = 0;
        if (toothCurrentCount == 1
            || toothCurrentCount == 3
            || toothCurrentCount == 5
            || toothCurrentCount == 7) //96.26 degrees with a target of 110
        {
          triggerToothAngle = 70;
        }
        else
        {
          triggerToothAngle = 110;
        }
      }

      //EXPERIMENTAL!
      //New ignition mode is ONLY available on 9905 when the trigger angle is
      //set to the stock value of 0.
      if (configPage2.perToothIgn && configPage4.triggerAngle == 0 && currentStatus.advance > 0)
      {
        int16_t crankAngle = ignitionLimits(toothAngles[toothCurrentCount - 1]);

        //Handle non-sequential tooth counts
        if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && toothCurrentCount > configPage2.nCylinders)
        {
          checkPerToothTiming(crankAngle, toothCurrentCount - configPage2.nCylinders);
        }
        else
        {
          checkPerToothTiming(crankAngle, toothCurrentCount);
        }
      }
    } //Has sync

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //The +30 here is a safety margin. When switching from fixed timing to normal,
    //there can be a situation where a pulse started when fixed and ending when
    //in normal mode causes problems. This prevents that.
    unsigned const rpm_safety_margin = 30;

    if (currentStatus.RPM < currentStatus.crankRPM + rpm_safety_margin && configPage4.ignCranklock)
    {
      if (toothCurrentCount == 1 || toothCurrentCount == 5)
      {
        twoCoilsEndCharge(ignition_id_1, ignition_id_3);
      }
      else if (toothCurrentCount == 3 || toothCurrentCount == 7)
      {
        twoCoilsEndCharge(ignition_id_2, ignition_id_4);
      }
    }
    secondaryToothCount = 0;
  } //Trigger filter

}

void triggerSec_Miata9905(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) || !currentStatus.hasSync)
  {
    //If this is removed, can have trouble getting sync again after the
    //engine is turned off (but ECU not reset).
    triggerFilterTime = 1500;
  }

  if (curGap2 >= triggerSecFilterTime)
  {
    toothLastSecToothTime = curTime2;
    lastGap = curGap2;
    secondaryToothCount++;

    //TODO Add some secondary filtering here

    //Record the VVT tooth time
    if (toothCurrentCount == 1 && curTime2 > toothLastToothTime)
    {
      lastVVTtime = curTime2 - toothLastToothTime;
    }
  }
}

uint16_t getRPM_Miata9905(void)
{
  //During cranking, RPM is calculated 4 times per revolution, once for each
  //tooth on the crank signal.
  //Because these signals aren't even (Alternating 110 and 70 degrees), this
  //needs a special function
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM && currentStatus.hasSync)
  {
    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      tempRPM = 0;
    }
    else
    {
      int tempToothAngle;
      unsigned long toothTime;

      noInterrupts();

      tempToothAngle = triggerToothAngle;
      //Note that trigger tooth angle changes between 70 and 110 depending on
      //the last tooth that was seen
      toothTime = toothLastToothTime - toothLastMinusOneToothTime;

      interrupts();

      toothTime = toothTime * 36;
      tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN / 10U)) / toothTime;
      SetRevolutionTime((10UL * toothTime) / tempToothAngle);
      MAX_STALL_TIME = 366667UL; // 50RPM
    }
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
    //Set the stall time to be twice the current RPM. This is a safe figure as
    //there should be no single revolution where this changes more than this
    MAX_STALL_TIME = revolutionTime << 1;
    if (MAX_STALL_TIME < 366667UL) //Check for 50rpm minimum
    {
      MAX_STALL_TIME = 366667UL;
    }
  }

  return tempRPM;
}

int getCrankAngle_Miata9905(void)
{
  int crankAngle = 0;

  {
    //This is the current angle ATDC the engine is at. This is the last known
    //position based on what tooth was last 'seen'. It is only accurate to the
    //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
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
    crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
    crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

    if (crankAngle >= 720)
    {
      crankAngle -= 720;
    }
    if (crankAngle > CRANK_ANGLE_MAX)
    {
      crankAngle -= CRANK_ANGLE_MAX;
    }
    if (crankAngle < 0)
    {
      crankAngle += 360;
    }
  }

  return crankAngle;
}

int getCamAngle_Miata9905(void)
{
  int16_t curAngle;//lastVVTtime is the time between tooth #1 (10* BTDC) and the single cam tooth.
  //All cam angles in in BTDC, so the actual advance angle is
  //370 - timeToAngleDegPerMicroSec(lastVVTtime) - <the angle of the cam at 0 advance>
  curAngle = 370 - timeToAngleDegPerMicroSec(lastVVTtime, degreesPerMicro) - configPage10.vvtCL0DutyAng;
  currentStatus.vvt1Angle =
    ANGLE_FILTER((curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);

  return currentStatus.vvt1Angle;
}

void triggerSetEndTeeth_Miata9905(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
  ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 8;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4;
      ignition4.endTooth = 6;
    }
    else if (currentStatus.advance > 0)
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 3;
      ignition3.endTooth = 5;
      ignition4.endTooth = 7;
    }
  }
  else
  {
    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 4;
      ignition2.endTooth = 2;
      ignition3.endTooth = 4; //Not used
      ignition4.endTooth = 2; //Not used
    }
    else if (currentStatus.advance > 0)
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 3;
      ignition3.endTooth = 1; //Not used
      ignition4.endTooth = 3; //Not used
    }
  }
}

/** @} */

/** Mazda AU version.
Tooth #2 is defined as the next crank tooth after the single cam tooth.
Tooth number one is at 348* ATDC.
* @defgroup mazda_au Mazda AU
* @{
*/
void triggerSetup_MazdaAU(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth (primary).
  //This is the maximum gap
  triggerToothAngle = 108;
  toothCurrentCount = 99; //Fake tooth count represents no sync
  secondaryToothCount = 0; //Needed for the cam tooth tracking
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);

  toothAngles[0] = 348; //tooth #1
  toothAngles[1] = 96; //tooth #2
  toothAngles[2] = 168; //tooth #3
  toothAngles[3] = 276; //tooth #4

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle);
  //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerFilterTime = 1500;
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2
  //(for cam speed)
  triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  BIT_SET(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
}

void triggerPri_MazdaAU(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap >= triggerFilterTime)
  {
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothCurrentCount++;
    //Trigger is on CHANGE, hence 4 pulses = 1 crank rev
    if (toothCurrentCount == 1 || toothCurrentCount == 5)
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.hasSync = true;
      currentStatus.startRevolutions++; //Counter
    }

    if (currentStatus.hasSync)
    {
      // Locked cranking timing is available, fixed at 12* BTDC
      if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
      {
        if (toothCurrentCount == 1)
        {
          singleCoilEndCharge(ignition_id_1);
        }
        else if (toothCurrentCount == 3)
        {
          singleCoilEndCharge(ignition_id_2);
        }
      }

      //Whilst this is an uneven tooth pattern, if the specific angle between
      //the last 2 teeth is specified, 1st deriv prediction can be used
      //Trigger filter is set to whatever time it took to do 72 degrees
      //(Next trigger is 108 degrees away)
      if (toothCurrentCount == 1 || toothCurrentCount == 3)
      {
        triggerToothAngle = 72;
        triggerFilterTime = curGap;
      }
      else
      {
        //Trigger filter is set to (108*3)/8=40 degrees
        //(Next trigger is 70 degrees away).
        triggerToothAngle = 108;
        triggerFilterTime = (curGap * 3) >> 3;
      }

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
    } //Has sync
  } //Filter time
}

void triggerSec_MazdaAU(void)
{
  curTime2 = micros();
  lastGap = curGap2;
  curGap2 = curTime2 - toothLastSecToothTime;
  toothLastSecToothTime = curTime2;

  if (!currentStatus.hasSync)
  {
    //we find sync by looking for the 2 teeth that are close together.
    //The next crank tooth after that is the one we're looking for.
    //For the sake of this decoder, the lone cam tooth will be designated #1
    if (secondaryToothCount == 2)
    {
      toothCurrentCount = 1;
      currentStatus.hasSync = true;
    }
    else
    {
      triggerFilterTime = 1500; //In case the engine has been running and then lost sync.
      targetGap = (lastGap) >> 1; //The target gap is set at half the last tooth gap
      //If the gap between this tooth and the last one is less than half of the
      //previous gap, then we are very likely at the extra (3rd) tooth on the cam).
      //This tooth is located at 421 crank degrees (aka 61 degrees) and therefore
      //the last crank tooth seen was number 1 (At 350 degrees)
      if (curGap2 < targetGap)
      {
        secondaryToothCount = 2;
      }
    }
    secondaryToothCount++;
  }
}


uint16_t getRPM_MazdaAU(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync)
  {
    //During cranking, RPM is calculated 4 times per revolution, once for each tooth on the crank signal.
    //Because these signals aren't even (Alternating 108 and 72 degrees), this needs a special function
    if (currentStatus.RPM < currentStatus.crankRPM)
    {
      int tempToothAngle;

      noInterrupts();

      tempToothAngle = triggerToothAngle;
      //Note that trigger tooth angle changes between 72 and 108 depending on
      //the last tooth that was seen
      SetRevolutionTime(36 * (toothLastToothTime - toothLastMinusOneToothTime));

      interrupts();

      tempRPM = (tempToothAngle * MICROS_PER_MIN) / revolutionTime;
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}

int getCrankAngle_MazdaAU(void)
{
  int crankAngle = 0;
  if (currentStatus.hasSync)
  {
    //This is the current angle ATDC the engine is at. This is the last known
    //position based on what tooth was last 'seen'. It is only accurate to the
    //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
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
    crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
    crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

    if (crankAngle >= 720)
    {
      crankAngle -= 720;
    }
    if (crankAngle > CRANK_ANGLE_MAX)
    {
      crankAngle -= CRANK_ANGLE_MAX;
    }
    if (crankAngle < 0)
    {
      crankAngle += 360;
    }
  }

  return crankAngle;
}

void triggerSetEndTeeth_MazdaAU(void)
{
}

/** @} */

/** Non-360 Dual wheel with 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
There can be no missing teeth on the primary wheel.
* @defgroup dec_non360 Non-360 Dual wheel
* @{
*/
void triggerSetup_non360(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //The number of degrees that passes from tooth to tooth multiplied by the additional multiplier
  triggerToothAngle = (360U * configPage4.TrigAngMul) / configPage4.triggerTeeth;
  toothCurrentCount = 255; //Default value
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth);
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2
  //(for cam speed)
  triggerSecFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle;
}


void triggerPri_non360(void)
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

void triggerSec_non360(void)
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

uint16_t getRPM_non360(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync && toothCurrentCount != 0)
  {
    if (currentStatus.RPM < currentStatus.crankRPM)
    {
      tempRPM = crankingGetRPM(configPage4.triggerTeeth, CRANK_SPEED);
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}

int getCrankAngle_non360(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Handle case where the secondary tooth was the last one seen
  if (tempToothCurrentCount == 0)
  {
    tempToothCurrentCount = configPage4.triggerTeeth;
  }

  //Number of teeth that have passed since tooth 1, multiplied by the angle each
  //tooth represents, plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = (tempToothCurrentCount - 1) * triggerToothAngle;
  //Have to divide by the multiplier to get back to actual crank angle.
  crankAngle = (crankAngle / configPage4.TrigAngMul) + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += 360;
  }

  return crankAngle;
}

void triggerSetEndTeeth_non360(void)
{
}

/** @} */

/** Nissan 360 tooth on cam (Optical trigger disc inside distributor housing).
See http://wiki.r31skylineclub.com/index.php/Crank_Angle_Sensor .
* @defgroup dec_nissan360 Nissan 360 tooth on cam
* @{
*/
void triggerSetup_Nissan360(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 360UL);
  //Same as above, but fixed at 2 teeth on the secondary input and divided by 2
  //(for cam speed)
  triggerSecFilterTime = (int)(MICROS_PER_SEC / (MAX_RPM / 60U * 2U)) / 2U;
  secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  toothCurrentCount = 1;
  triggerToothAngle = 2;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle;
}

void triggerPri_Nissan360(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  toothCurrentCount++; //Increment the tooth counter
  //Flag this pulse as being a valid trigger (ie that it passed filters)
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  toothLastMinusOneToothTime = toothLastToothTime;
  toothLastToothTime = curTime;

  if (currentStatus.hasSync)
  {
    if (toothCurrentCount == 361) //2 complete crank revolutions
    {
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //EXPERIMENTAL!
    if (configPage2.perToothIgn)
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * 2) + configPage4.triggerAngle;
      if (crankAngle > CRANK_ANGLE_MAX_IGN)
      {
        crankAngle -= CRANK_ANGLE_MAX_IGN;
        checkPerToothTiming(crankAngle, toothCurrentCount / 2);
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

void triggerSec_Nissan360(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  toothLastSecToothTime = curTime2;

  //Calculate number of primary teeth that this window has been active for
  byte trigEdge;

  if (configPage4.TrigEdgeSec == 0)
  {
    trigEdge = LOW;
  }
  else
  {
    trigEdge = HIGH;
  }

  //This occurs on the first rotation upon powerup OR the start of a secondary window
  if (secondaryToothCount == 0 || Trigger2.read() == trigEdge)
  {
    secondaryToothCount = toothCurrentCount;
  }
  else
  {
    //If we reach here, we are at the end of a secondary window
    //How many primary teeth have passed during the duration of this secondary window
    byte secondaryDuration = toothCurrentCount - secondaryToothCount;

    if (!currentStatus.hasSync)
    {
      if (configPage2.nCylinders == 4)
      {
        //Supported pattern is where all the inner windows as a different size (Most SR engines)
        //These equate to 4,8,12,16 teeth spacings
        if (secondaryDuration >= 15 && secondaryDuration <= 17)
        {
          //Duration of window = 16 primary teeth
          //End of first window (The longest) occurs 16 teeth after TDC
          toothCurrentCount = 16;
          currentStatus.hasSync = true;
        }
        else if (secondaryDuration >= 11 && secondaryDuration <= 13)
        {
          //Duration of window = 12 primary teeth
          toothCurrentCount = 102; //End of second window is after 90+12 primary teeth
          currentStatus.hasSync = true;
        }
        else if (secondaryDuration >= 7 && secondaryDuration <= 9)
        {
          //Duration of window = 8 primary teeth
          toothCurrentCount = 188; //End of third window is after 90+90+8 primary teeth
          currentStatus.hasSync = true;
        }
        else if (secondaryDuration >= 3 && secondaryDuration <= 5)
        {
          //Duration of window = 4 primary teeth
          toothCurrentCount = 274; //End of fourth window is after 90+90+90+4 primary teeth
          currentStatus.hasSync = true;
        }
        else //This should really never happen
        {
          currentStatus.hasSync = false;
          currentStatus.syncLossCounter++;
        }
      }
      else if (configPage2.nCylinders == 6)
      {
        //Pattern on the 6 cylinders is 4-8-12-16-20-24
        if (secondaryDuration >= 3 && secondaryDuration <= 5)
        {
          //Duration of window = 4 primary teeth
          toothCurrentCount = 124; //End of smallest window is after 60+60+4 primary teeth
          currentStatus.hasSync = true;
        }
      }
      else if (configPage2.nCylinders == 8)
      {
        //V8 Optispark
        //Pattern on the 8 cylinders is the same as the 6 cylinder 4-8-12-16-20-24
        if (secondaryDuration >= 6 && secondaryDuration <= 8)
        {
          //Duration of window = 16 primary teeth
          toothCurrentCount = 56; //End of the shortest of the individual windows. Occurs at 102 crank degrees.
          currentStatus.hasSync = true;
        }
      }
      else //This should really never happen (Only 4, 6 and 8 cylinder engines for this pattern)
      {
        currentStatus.hasSync = false;
      }
    }
    else
    {
      if (configPage4.useResync)
      {
        //Already have sync, but do a verify every 720 degrees.
        if (configPage2.nCylinders == 4)
        {
          if (secondaryDuration >= 15 && secondaryDuration <= 17)
          {
            //Duration of window = 16 primary teeth
            toothCurrentCount = 16; //End of first window (The longest) occurs 16 teeth after TDC
          }
        }
        else if (configPage2.nCylinders == 6)
        {
          if (secondaryDuration == 4)
          {
            /* Do nothing. */
          }
        } //Cylinder count
      } //use resync
    } //Has sync
  } //First getting sync or not
}

uint16_t getRPM_Nissan360(void)
{
  //Can't use stdGetRPM as there is no separate cranking RPM calc
  //(stdGetRPM returns 0 if cranking)
  uint16_t tempRPM;

  if (currentStatus.hasSync && toothLastToothTime != 0 && toothLastMinusOneToothTime != 0)
  {
    if (currentStatus.startRevolutions < 2)
    {
      noInterrupts();

      //Each tooth covers 2 crank degrees, so multiply by 180 to get a full revolution time.
      SetRevolutionTime((toothLastToothTime - toothLastMinusOneToothTime) * 180);

      interrupts();
    }
    else
    {
      noInterrupts();

      //The time in uS that one revolution would take at current speed
      //(The time tooth 1 was last seen, minus the time it was seen prior to that)
      SetRevolutionTime((toothOneTime - toothOneMinusOneTime) >> 1);

      interrupts();
    }
    //Calc RPM based on last full revolution time (Faster as /)
    tempRPM = RpmFromRevolutionTimeUs(revolutionTime);
    //Set the stall time to be twice the current RPM. This is a safe figure as
    //there should be no single revolution where this changes more than this
    MAX_STALL_TIME = revolutionTime << 1;
  }
  else
  {
    tempRPM = 0;
  }

  return tempRPM;
}

int getCrankAngle_Nissan360(void)
{
  //As each tooth represents 2 crank degrees, we only need to determine whether
  //we're more or less than halfway between teeth to know whether to add another 1 degrees
  int crankAngle = 0;
  int tempToothLastToothTime;
  int tempToothLastMinusOneToothTime;
  int tempToothCurrentCount;

  noInterrupts();

  tempToothLastToothTime = toothLastToothTime;
  tempToothLastMinusOneToothTime = toothLastMinusOneToothTime;
  tempToothCurrentCount = toothCurrentCount;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  crankAngle = ((tempToothCurrentCount - 1) * 2) + configPage4.triggerAngle;
  unsigned long halfTooth = (tempToothLastToothTime - tempToothLastMinusOneToothTime) / 2;
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
  if (elapsedTime > halfTooth)
  {
    //Means we're over halfway to the next tooth, so add on 1 degree
    crankAngle += 1;
  }

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += 360;
  }

  return crankAngle;
}

static void triggerSetEndTeeth_Nissan360_ignition(ignition_context_st &ignition)
{
  //This uses 4 prior teeth, just to ensure there is sufficient time to set the schedule etc
  byte const offset_teeth = 4;

  if (ignition.endAngle - offset_teeth > configPage4.triggerAngle)
  {
    ignition.endTooth = ((ignition.endAngle - configPage4.triggerAngle) / 2) - offset_teeth;
  }
  else
  {
    ignition.endTooth = ((ignition.endAngle + 720 - configPage4.triggerAngle) / 2) - offset_teeth;
  }
}

void triggerSetEndTeeth_Nissan360(void)
{
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel1));
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel2));
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel3));
  triggerSetEndTeeth_Nissan360_ignition(ignitions.ignition(ignChannel4));
}

/** @} */

/** Subaru 6/7 Trigger pattern decoder for 6 tooth (irregularly spaced) crank
*   and 7 tooth (also fairly irregular) cam wheels (eg late 90's Impreza 2.2).
This seems to be present in late 90's Subaru. In 2001 Subaru moved to 36-2-2-2*
*(See: http://www.vems.hu/wiki/index.php?page=InputTrigger%2FSubaruTrigger ).
* @defgroup dec_subaru_6_7 Subaru 6/7
* @{
*/
void triggerSetup_Subaru67(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 360UL);
  triggerSecFilterTime = 0;
  //Initially set to 0 prior to calculating the secondary window duration
  secondaryToothCount = 0;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  toothCurrentCount = 1;
  triggerToothAngle = 2;
  BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  toothSystemCount = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * 93U;

  toothAngles[0] = 710; //tooth #1
  toothAngles[1] = 83; //tooth #2
  toothAngles[2] = 115; //tooth #3
  toothAngles[3] = 170; //tooth #4
  toothAngles[4] = toothAngles[1] + 180;
  toothAngles[5] = toothAngles[2] + 180;
  toothAngles[6] = toothAngles[3] + 180;
  toothAngles[7] = toothAngles[1] + 360;
  toothAngles[8] = toothAngles[2] + 360;
  toothAngles[9] = toothAngles[3] + 360;
  toothAngles[10] = toothAngles[1] + 540;
  toothAngles[11] = toothAngles[2] + 540;
}


void triggerPri_Subaru67(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap < triggerFilterTime)
  {
    return;
  }

  toothCurrentCount++; //Increment the tooth counter
  //Used to count the number of primary pulses that have occurred since the last
  //secondary. Is part of the noise filtering system.
  toothSystemCount++;
  //Flag this pulse as being a valid trigger (ie that it passed filters)
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

  toothLastMinusOneToothTime = toothLastToothTime;
  toothLastToothTime = curTime;

  if (toothCurrentCount > 13) //can't have more than 12 teeth so have lost sync
  {
    toothCurrentCount = 0;
    currentStatus.hasSync = false;
    currentStatus.syncLossCounter++;
  }

  //Sync is determined by counting the number of cam teeth that have passed between the crank teeth
  switch (secondaryToothCount)
  {
  case 0:
    //If no teeth have passed, we can't do anything
    break;

  case 1:
    //Can't do anything with a single pulse from the cam either (We need either 2 or 3 pulses)
    if (toothCurrentCount == 5 || toothCurrentCount == 11)
    {
      currentStatus.hasSync = true;
    }
    else
    {
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      // we don't know if its 5 or 11, but we'll be right 50% of the time and
      // speed up getting sync 50%
      toothCurrentCount = 5;
    }
    secondaryToothCount = 0;
    break;

  case 2:
    if (toothCurrentCount == 8)
    {
      currentStatus.hasSync = true;
    }
    else
    {
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      toothCurrentCount = 8;
    }
    secondaryToothCount = 0;
    break;

  case 3:
    if (toothCurrentCount == 2)
    {
      currentStatus.hasSync = true;
    }
    else
    {
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
      toothCurrentCount = 2;
    }
    secondaryToothCount = 0;
    break;

  default:
    //Almost certainly due to noise or cranking stop/start
    currentStatus.hasSync = false;
    BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
    currentStatus.syncLossCounter++;
    secondaryToothCount = 0;
    break;
  }

  //Check sync again
  if (currentStatus.hasSync)
  {
    //Locked timing during cranking. This is fixed at 10* BTDC.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
    {
      if (toothCurrentCount == 1 || toothCurrentCount == 7)
      {
        twoCoilsEndCharge(ignition_id_1, ignition_id_3);
      }
      else if (toothCurrentCount == 4 || toothCurrentCount == 10)
      {
        twoCoilsEndCharge(ignition_id_2, ignition_id_4);
      }
    }

    if (toothCurrentCount > 12) // done 720 degrees so increment rotation
    {
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //Set the last angle between teeth for better calc accuracy
    if (toothCurrentCount == 1) //Special case for tooth 1
    {
      triggerToothAngle = 55;
    }
    else if (toothCurrentCount == 2) //Special case for tooth 2
    {
      triggerToothAngle = 93;
    }
    else
    {
      triggerToothAngle = toothAngles[toothCurrentCount - 1] - toothAngles[toothCurrentCount - 2];
    }
    BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);


    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = toothAngles[toothCurrentCount - 1] + configPage4.triggerAngle;

      if ((configPage4.sparkMode != IGN_MODE_SEQUENTIAL))
      {
        crankAngle = ignitionLimits(toothAngles[toothCurrentCount - 1]);

        //Handle non-sequential tooth counts
        if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && toothCurrentCount > 6)
        {
          checkPerToothTiming(crankAngle, (toothCurrentCount - 6));
        }
        else
        {
          checkPerToothTiming(crankAngle, toothCurrentCount);
        }
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

void triggerSec_Subaru67(void)
{
  if (toothSystemCount == 0 || toothSystemCount == 3)
  {
    curTime2 = micros();
    curGap2 = curTime2 - toothLastSecToothTime;

    if (curGap2 > triggerSecFilterTime)
    {
      toothLastSecToothTime = curTime2;
      secondaryToothCount++;
      toothSystemCount = 0;

      if (secondaryToothCount > 1)
      {
        //Set filter at 25% of the current speed
        //Note that this can only be set on the 2nd or 3rd cam tooth in each set.
        triggerSecFilterTime = curGap2 >> 2;
      }
      else //Filter disabled
      {
        triggerSecFilterTime = 0;
      }
    }
  }
  else
  {
    //Sanity check
    if (toothSystemCount > 3)
    {
      toothSystemCount = 0;
      secondaryToothCount = 1;
      // impossible to have more than 3 crank teeth between cam teeth
      // - must have noise but can't have sync
      currentStatus.hasSync = false;
      currentStatus.syncLossCounter++;
    }
    secondaryToothCount = 0;
  }

}

uint16_t getRPM_Subaru67(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.startRevolutions > 0)
  {
    //As the tooth count is over 720 degrees
    tempRPM = stdGetRPM(CAM_SPEED);
  }

  return tempRPM;
}

int getCrankAngle_Subaru67(void)
{
  int crankAngle = 0;
  if (currentStatus.hasSync)
  {
    //This is the current angle ATDC the engine is at. This is the last known
    //position based on what tooth was last 'seen'. It is only accurate to the
    //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
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
    crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
    crankAngle += timeToAngleIntervalTooth(elapsedTime);

    if (crankAngle >= 720)
    {
      crankAngle -= 720;
    }
    if (crankAngle > CRANK_ANGLE_MAX)
    {
      crankAngle -= CRANK_ANGLE_MAX;
    }
    if (crankAngle < 0)
    {
      crankAngle += 360;
    }
  }

  return crankAngle;
}

void triggerSetEndTeeth_Subaru67(void)
{
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
    ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
    ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
    ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 12;
      ignition2.endTooth = 3;
      ignition3.endTooth = 6;
      ignition4.endTooth = 9;
    }
    else
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 4;
      ignition3.endTooth = 7;
      ignition4.endTooth = 10;
    }
  }
  else
  {
    ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
    ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);

    if (currentStatus.advance >= 10)
    {
      ignition1.endTooth = 6;
      ignition2.endTooth = 3;
    }
    else
    {
      ignition1.endTooth = 1;
      ignition2.endTooth = 4;
    }
  }
}
/** @} */

/** Daihatsu +1 trigger for 3 and 4 cylinder engines.
* Tooth equal to the number of cylinders are evenly spaced on the cam. No position sensing (Distributor is retained),
* so crank angle is a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_daihatsu Daihatsu (3  and 4 cyl.)
* @{
*/
void triggerSetup_Daihatsu(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerActualTeeth = configPage2.nCylinders + 1;
  triggerToothAngle = 720 / triggerActualTeeth; //The number of degrees that passes from tooth to tooth
  triggerFilterTime = MICROS_PER_MIN / MAX_RPM / configPage2.nCylinders; // Minimum time required between teeth
  triggerFilterTime = triggerFilterTime / 2; //Safety margin
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);

  //Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm
  //rather than 50rpm due to the potentially very high stall time on a 4
  //cylinder if we wait that long.
  unsigned const minimum_rpm = 90;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle) * 4U;

  if (configPage2.nCylinders == 3)
  {
    toothAngles[0] = 0; //tooth #1
    toothAngles[1] = 30; //tooth #2 (Extra tooth)
    toothAngles[2] = 240; //tooth #3
    toothAngles[3] = 480; //tooth #4
  }
  else
  {
    //Should be 4 cylinders here
    toothAngles[0] = 0; //tooth #1
    toothAngles[1] = 30; //tooth #2 (Extra tooth)
    toothAngles[2] = 180; //tooth #3
    toothAngles[3] = 360; //tooth #4
    toothAngles[4] = 540; //tooth #5
  }
}

void triggerPri_Daihatsu(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  {
    toothSystemCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    if (currentStatus.hasSync)
    {
      //Check if we're back to the beginning of a revolution
      if (toothCurrentCount == triggerActualTeeth)
      {
        toothCurrentCount = 1; //Reset the counter
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.hasSync = true;
        currentStatus.startRevolutions++; //Counter

        //Need to set a special filter time for the next tooth
        triggerFilterTime = 20; //Fix this later
      }
      else
      {
        toothCurrentCount++; //Increment the tooth counter
        setFilter(curGap); //Recalc the new filter value
      }

      if (configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
      {
        //This locks the cranking timing to 0 degrees BTDC (All the triggers allow for)
        if (toothCurrentCount == 1)
        {
          singleCoilEndCharge(ignition_id_1);
        }
        else if (toothCurrentCount == 2)
        {
          singleCoilEndCharge(ignition_id_2);
        }
        else if (toothCurrentCount == 3)
        {
          singleCoilEndCharge(ignition_id_3);
        }
        else if (toothCurrentCount == 4)
        {
          singleCoilEndCharge(ignition_id_4);
        }
      }
    }
    else //NO SYNC
    {
      if (toothSystemCount >= 3) //Need to have seen at least 3 teeth to determine SYNC
      {
        unsigned long targetTime;
        //We need to try and find the extra tooth (#2) which is located 30 degrees
        //after tooth #1
        //Aim for tooth times less than about 60 degrees
        if (configPage2.nCylinders == 3)
        {
          //Teeth are 240 degrees apart for 3 cylinder. 240/4 = 60
          targetTime = (toothLastToothTime -  toothLastMinusOneToothTime) / 4;
        }
        else
        {
          //Teeth are 180 degrees apart for 4 cylinder. (180*3)/8 = 67
          targetTime = ((toothLastToothTime -  toothLastMinusOneToothTime) * 3) / 8;
        }
        if (curGap < targetTime)
        {
          //Means we're on the extra tooth here
          toothCurrentCount = 2; //Reset the counter
          currentStatus.hasSync = true;
          triggerFilterTime = targetTime; //Lazy, but it works
        }
      }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  }
}

void triggerSec_Daihatsu(void) //Not required (Should never be called in the first place)
{
  return;
}

uint16_t getRPM_Daihatsu(void)
{
  uint16_t tempRPM = 0;

  //Disable special cranking processing for now
  if (0 && (currentStatus.RPM < currentStatus.crankRPM))
  {
    //Can't use standard cranking RPM function due to extra tooth
    if (currentStatus.hasSync)
    {
      if (toothCurrentCount == 2)
      {
        tempRPM = currentStatus.RPM;
      }
      else if (toothCurrentCount == 3)
      {
        tempRPM = currentStatus.RPM;
      }
      else
      {
        noInterrupts();

        SetRevolutionTime((toothLastToothTime - toothLastMinusOneToothTime) * (triggerActualTeeth - 1));

        interrupts();

        tempRPM = RpmFromRevolutionTimeUs(revolutionTime);
      } //is tooth #2
    }
    else //No sync
    {
      tempRPM = 0;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
  } //Tracking over 2 crank revolutions

  return tempRPM;
}

int getCrankAngle_Daihatsu(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  int crankAngle;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Crank angle of the last tooth seen
  crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += CRANK_ANGLE_MAX;
  }

  return crankAngle;
}

void triggerSetEndTeeth_Daihatsu(void)
{
}

/** @} */

/** Harley Davidson (V2) with 2 unevenly Spaced Teeth.
Within the decoder code, the sync tooth is referred to as tooth #1. Derived from GMX7 and adapted for Harley.
Only rising Edge is used for simplicity.The second input is ignored, as it does not help to resolve cam position.
* @defgroup dec_harley Harley Davidson
* @{
*/
void triggerSetup_Harley(bool const initialisationComplete)
{
  triggerToothAngle = 0; // The number of degrees that passes from tooth to tooth, ev. 0. It alternates uneven
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * 60U);

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on all the time.
    toothLastToothTime = micros();
  }
  triggerFilterTime = 1500;
}

void triggerPri_Harley(void)
{
  lastGap = curGap;
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  setFilter(curGap); // Filtering adjusted according to setting
  if (curGap > triggerFilterTime)
  {
    if (Trigger.read()) // Has to be the same as in main() trigger-attach, for readability we do it this way.
    {
      //Flag this pulse as being a valid trigger (ie that it passed filters)
      BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
      targetGap = lastGap; //Gap is the Time to next toothtrigger, so we know where we are
      toothCurrentCount++;
      if (curGap > targetGap)
      {
        toothCurrentCount = 1;
        triggerToothAngle = 0; // Has to be equal to Angle Routine
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.hasSync = true;
      }
      else
      {
        toothCurrentCount = 2;
        triggerToothAngle = 157;
      }
      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if (currentStatus.hasSync)
      {
        currentStatus.syncLossCounter++;
      }
      currentStatus.hasSync = false;
      toothCurrentCount = 0;
    } //Primary trigger high
  } //Trigger filter
}


// Needs to be enabled in main()
void triggerSec_Harley(void)
{
  return; // No need for now. The only thing it could help to sync more quickly or confirm position.
} // End Sec Trigger

uint16_t getRPM_Harley(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync)
  {
    if (currentStatus.RPM < (unsigned int)(configPage4.crankRPM * 100))
    {
      // No difference with this option?
      int tempToothAngle;
      unsigned long toothTime;
      if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
      {
        tempRPM = 0;
      }
      else
      {
        noInterrupts();

        tempToothAngle = triggerToothAngle;
        //The time in uS that one revolution would take at current speed
        //(The time tooth 1 was last seen, minus the time it was seen prior to that)
        SetRevolutionTime(toothOneTime - toothOneMinusOneTime);
        //Note that trigger tooth angle changes between 129 and 332 depending on
        //the last tooth that was seen
        toothTime = (toothLastToothTime - toothLastMinusOneToothTime);

        interrupts();

        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN / 10U)) / toothTime;
      }
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }
  return tempRPM;
}

int getCrankAngle_Harley(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Check if the last tooth seen was the reference tooth (Number 3). All others can be calculated, but tooth 3 has a unique angle
  int crankAngle;

  if (tempToothCurrentCount == 1 || tempToothCurrentCount == 3)
  {
    //Number of teeth that have passed since tooth 1, multiplied by the angle
    //each tooth represents, plus the angle that tooth 1 is ATDC.
    //This gives accuracy only to the nearest tooth.
    crankAngle = 0 + configPage4.triggerAngle;
  }
  else
  {
    crankAngle = 157 + configPage4.triggerAngle;
  }

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += 360;
  }

  return crankAngle;
}

void triggerSetEndTeeth_Harley(void)
{
}

/** @} */

//************************************************************************************************************************

/** 36-2-2-2 crank based trigger wheel.
* A crank based trigger with a nominal 36 teeth, but 6 of these removed in 3 groups of 2.
* 2 of these groups are located concurrently.
* Note: This decoder supports both the H4 version (13-missing-16-missing-1-missing)
* and the H6 version of 36-2-2-2 (19-missing-10-missing-1-missing).
* The decoder checks which pattern is selected in order to determine the tooth number
* Note: www.thefactoryfiveforum.com/attachment.php?attachmentid=34279&d=1412431418
*
* @defgroup dec_36_2_2_2 36-2-2-2 Trigger wheel
* @{
*/
void triggerSetup_ThirtySixMinus222(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  //The number of physical teeth on the wheel.
  //Doing this here saves us a calculation each time in the interrupt
  triggerActualTeeth = 30;
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 36);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  checkSyncToothCount = configPage4.triggerTeeth >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;
}

void triggerPri_ThirtySixMinus222(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger.
  //(A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    //Begin the missing tooth detection
    //If the time between the current tooth and the last is greater than 2x the
    //time between the last tooth and the tooth before that, we make the
    //assertion that we must be at the first tooth after a gap
    //toothSystemCount is used to keep track of which missed tooth we're on.
    //It will be set to 1 if that last tooth seen was the middle one in the
    //-2-2 area. At all other times it will be 0
    if (toothSystemCount == 0)
    {
      //Multiply by 2 (Checks for a gap 2x greater than the last one)
      targetGap = (toothLastToothTime - toothLastMinusOneToothTime) * 2;
    }

    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      curGap = 0;
    }

    if (curGap > targetGap)
    {
      {
        if (toothSystemCount == 1)
        {
          //This occurs when we're at the first tooth after the 2 lots of 2x missing tooth.
          if (configPage2.nCylinders == 4) //H4
          {
            toothCurrentCount = 19;
          }
          else if (configPage2.nCylinders == 6) //H6 - NOT TESTED!
          {
            toothCurrentCount = 12;
          }

          toothSystemCount = 0;
          currentStatus.hasSync = true;
        }
        else
        {
          //We've seen a missing tooth set, but do not yet know whether it is
          //the single one or the double one.
          toothSystemCount = 1;
          toothCurrentCount++;
          //Accurately reflect the actual tooth count, including the skipped ones
          toothCurrentCount++;
        }
        //The tooth angle is double at this point
        BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
        //This is used to prevent a condition where serious intermittent signals
        //(e.g. someone furiously plugging the sensor wire in and out)
        //can leave the filter in an unrecoverable state
        triggerFilterTime = 0;
      }
    }
    else
    {
      if (toothCurrentCount > 36)
      {
        //Means a complete rotation has occurred.
        toothCurrentCount = 1;
        revolutionOne = !revolutionOne; //Flip sequential revolution tracker
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.startRevolutions++; //Counter

      }
      else if (toothSystemCount == 1)
      {
        //This occurs when a set of missing teeth had been seen, but the next
        //one was NOT missing.
        if (configPage2.nCylinders == 4)
        {
          //H4
          toothCurrentCount = 35;
          currentStatus.hasSync = true;
        }
        else if (configPage2.nCylinders == 6)
        {
          //H6 - THIS NEEDS TESTING
          toothCurrentCount = 34;
          currentStatus.hasSync = true;
        }
      }

      //Filter can only be recalculated for the regular teeth, not the missing one.
      setFilter(curGap);

      BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      toothSystemCount = 0;
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //EXPERIMENTAL!
    if (configPage2.perToothIgn)
    {
      int16_t crankAngle =
        ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      checkPerToothTiming(crankAngle, toothCurrentCount);
    }
  }
}

void triggerSec_ThirtySixMinus222(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus222(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (configPage2.nCylinders == 4
        && toothCurrentCount != 19
        && toothCurrentCount != 16
        && toothCurrentCount != 34
        && BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else if (configPage2.nCylinders == 6
             && toothCurrentCount != 9
             && toothCurrentCount != 12
             && toothCurrentCount != 33
             && BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at and of the missing teeth as it
      //messes the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_ThirtySixMinus222(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
  return 0;
}

void triggerSetEndTeeth_ThirtySixMinus222(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);

  if (configPage2.nCylinders == 4)
  {
    if (currentStatus.advance < 10)
    {
      ignition1.endTooth = 36;
    }
    else if (currentStatus.advance < 20)
    {
      ignition1.endTooth = 35;
    }
    else if (currentStatus.advance < 30)
    {
      ignition1.endTooth = 34;
    }
    else
    {
      ignition1.endTooth = 31;
    }

    if (currentStatus.advance < 30)
    {
      ignition2.endTooth = 16;
    }
    else
    {
      ignition2.endTooth = 13;
    }
  }
  else if (configPage2.nCylinders == 6)
  {
    //H6
    if (currentStatus.advance < 10)
    {
      ignition1.endTooth = 36;
    }
    else if (currentStatus.advance < 20)
    {
      ignition1.endTooth = 35;
    }
    else if (currentStatus.advance < 30)
    {
      ignition1.endTooth = 34;
    }
    else if (currentStatus.advance < 40)
    {
      ignition1.endTooth = 33;
    }
    else
    {
      ignition1.endTooth = 31;
    }

    if (currentStatus.advance < 20)
    {
      ignition2.endTooth = 9;
    }
    else
    {
      ignition2.endTooth = 6;
    }

    if (currentStatus.advance < 10)
    {
      ignition3.endTooth = 23;
    }
    else if (currentStatus.advance < 20)
    {
      ignition3.endTooth = 22;
    }
    else if (currentStatus.advance < 30)
    {
      ignition3.endTooth = 21;
    }
    else if (currentStatus.advance < 40)
    {
      ignition3.endTooth = 20;
    }
    else
    {
      ignition3.endTooth = 19;
    }
  }
}
/** @} */

//************************************************************************************************************************

/** 36-2-1 / Mistsubishi 4B11 - A crank based trigger with a nominal 36 teeth,
*   but with 1 single and 1 double missing tooth.
* @defgroup dec_36_2_1 36-2-1 For Mistsubishi 4B11
* @{
*/
void triggerSetup_ThirtySixMinus21(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  //The number of physical teeth on the wheel. Doing this here saves us a
  //calculation each time in the interrupt. Not Used
  triggerActualTeeth = 33;
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 36);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;
}

void triggerPri_ThirtySixMinus21(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    //Begin the missing tooth detection
    //If the time between the current tooth and the last is greater than 2x the
    //time between the last tooth and the tooth before that, we make the
    //assertion that we must be at the first tooth after a gap

    //Multiply by 3 (Checks for a gap 3x greater than the last one)
    targetGap2 = 3 * (toothLastToothTime - toothLastMinusOneToothTime);
    //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one)
    //(Uses bitshift to divide by 2 as in the missing tooth decoder)
    targetGap = targetGap2 >> 1;

    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      curGap = 0;
    }

    if (curGap > targetGap)
    {
      if (curGap < targetGap2)
      {
        //we are at the tooth after the single gap
        toothCurrentCount = 20; //it's either 19 or 20, need to clarify engine direction!
        currentStatus.hasSync = true;
      }
      else
      {
        //we are at the tooth after the double gap
        toothCurrentCount = 1;
        currentStatus.hasSync = true;
      }

      //The tooth angle is double at this point
      BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      //This is used to prevent a condition where serious intermittent signals
      //(e.g. someone furiously plugging the sensor wire in and out) can leave
      //the filter in an unrecoverable state
      triggerFilterTime = 0;
    }
  }
  else
  {
    if (toothCurrentCount > 36 || toothCurrentCount == 1)
    {
      //Means a complete rotation has occurred.
      toothCurrentCount = 1;
      revolutionOne = !revolutionOne; //Flip sequential revolution tracker
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //Filter can only be recalculated for the regular teeth, not the missing one.
    setFilter(curGap);

    BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  }

  toothLastMinusOneToothTime = toothLastToothTime;
  toothLastToothTime = curTime;

  //EXPERIMENTAL!
  if (configPage2.perToothIgn)
  {
    int16_t crankAngle =
      ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
    crankAngle = ignitionLimits(crankAngle);
    checkPerToothTiming(crankAngle, toothCurrentCount);
  }
}

void triggerSec_ThirtySixMinus21(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus21(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (toothCurrentCount != 20 && BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes
      //the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }

  return tempRPM;
}

int getCrankAngle_ThirtySixMinus21(void)
{
  //NOT USED - This pattern uses the missing tooth version of this function
  return 0;
}

void triggerSetEndTeeth_ThirtySixMinus21(void)
{
  ignitions.ignition(ignChannel1).endTooth = 10;
  ignitions.ignition(ignChannel2).endTooth = 28; // Arbitrarily picked  at 180°.
}

/** @} */

//************************************************************************************************************************

/** DSM 420a, For the DSM Eclipse with 16 teeth total on the crank.
* Tracks the falling side of the signal.
* Sync is determined by watching for a falling edge on the secondary signal and
* checking if the primary signal is high then.
* https://github.com/noisymime/speeduino/issues/133
* @defgroup dec_dsm_420a DSM 420a, For the DSM Eclipse
* @{
*/
void triggerSetup_420a(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM). Any pulses that occur faster than this
  //time will be discarded as noise
  triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * 360UL));
  triggerSecFilterTime = 0;
  //Initially set to 0 prior to calculating the secondary window duration
  secondaryToothCount = 0;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  toothCurrentCount = 1;
  triggerToothAngle = 20; //Is only correct for the 4 short pulses before each TDC
  BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  toothSystemCount = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * 93U);

  toothAngles[0] = 711; //tooth #1, just before #1 TDC
  toothAngles[1] = 111;
  toothAngles[2] = 131;
  toothAngles[3] = 151;
  toothAngles[4] = 171; //Just before #3 TDC
  toothAngles[5] = toothAngles[1] + 180;
  toothAngles[6] = toothAngles[2] + 180;
  toothAngles[7] = toothAngles[3] + 180;
  toothAngles[8] = toothAngles[4] + 180; //Just before #4 TDC
  toothAngles[9] = toothAngles[1] + 360;
  toothAngles[10] = toothAngles[2] + 360;
  toothAngles[11] = toothAngles[3] + 360;
  toothAngles[12] = toothAngles[4] + 360; //Just before #2 TDC
  toothAngles[13] = toothAngles[1] + 540;
  toothAngles[14] = toothAngles[2] + 540;
  toothAngles[15] = toothAngles[3] + 540;
}

void triggerPri_420a(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
    {
      curGap = 0;
    }

    if (toothCurrentCount > 16 && currentStatus.hasSync)
    {
      //Means a complete rotation has occurred.
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    triggerFilterTime = 0;

    BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //EXPERIMENTAL!
    if (configPage2.perToothIgn)
    {
      int16_t crankAngle = (toothAngles[toothCurrentCount - 1]) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      checkPerToothTiming(crankAngle, toothCurrentCount);
    }
  }
}

void triggerSec_420a(void)
{
  //Secondary trigger is only on falling edge

  if (Trigger.read())
  {
    //Secondary signal is falling and primary signal is HIGH
    if (!currentStatus.hasSync)
    {
      //If we don't have sync, then assume the signal is good
      toothCurrentCount = 13;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if (toothCurrentCount != 13)
      {
        currentStatus.syncLossCounter++;
        toothCurrentCount = 13;
      }
    }
  }
  else
  {
    //Secondary signal is falling and primary signal is LOW
    if (!currentStatus.hasSync)
    {
      //If we don't have sync, then assume the signal is good
      toothCurrentCount = 5;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if (toothCurrentCount != 5)
      {
        currentStatus.syncLossCounter++;
        toothCurrentCount = 5;
      }
    }
  }
}

uint16_t getRPM_420a(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    //Possibly look at doing special handling for cranking in the future,
    //but for now just use the standard method
    tempRPM = stdGetRPM(CAM_SPEED);
  }
  else
  {
    tempRPM = stdGetRPM(CAM_SPEED);
  }

  return tempRPM;
}

int getCrankAngle_420a(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  int crankAngle;

  //Perform a lookup of the fixed toothAngles array to find what the angle of
  //the last tooth passed was.
  crankAngle = toothAngles[tempToothCurrentCount - 1] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = lastCrankAngleCalc - tempToothLastToothTime;
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += 360;
  }

  return crankAngle;
}

void triggerSetEndTeeth_420a(void)
{
  ignition_context_st &ignition1 = ignitions.ignition(ignChannel1);
  ignition_context_st &ignition2 = ignitions.ignition(ignChannel2);
  ignition_context_st &ignition3 = ignitions.ignition(ignChannel3);
  ignition_context_st &ignition4 = ignitions.ignition(ignChannel4);

  if (currentStatus.advance < 9)
  {
    ignition1.endTooth = 1;
    ignition2.endTooth = 5;
    ignition3.endTooth = 9;
    ignition4.endTooth = 13;
  }
  else
  {
    ignition1.endTooth = 16;
    ignition2.endTooth = 4;
    ignition3.endTooth = 8;
    ignition4.endTooth = 12;
  }
}

/** @} */

/** Weber-Marelli trigger setup with 2 wheels, 4 teeth 90deg apart on crank and 2 90deg apart on cam.
Uses DualWheel decoders, There can be no missing teeth on the primary wheel.
* @defgroup dec_weber_marelli Weber-Marelli
* @{
*/
void triggerPri_Webber(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++; //Increment the tooth counter
    if (checkSyncToothCount > 0)
    {
      checkSyncToothCount++;
    }
    if (triggerSecFilterTime <= curGap) //150% crank tooth
    {
      triggerSecFilterTime = curGap + (curGap >> 1);
    }
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    if (currentStatus.hasSync)
    {
      if (toothCurrentCount == 1 || toothCurrentCount > configPage4.triggerTeeth)
      {
        toothCurrentCount = 1;
        revolutionOne = !revolutionOne; //Flip sequential revolution tracker
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.startRevolutions++; //Counter
      }

      setFilter(curGap); //Recalc the new filter value
    }
    else
    {
      if (secondaryToothCount == 1 && checkSyncToothCount == 4)
      {
        toothCurrentCount = 2;
        currentStatus.hasSync = true;
        revolutionOne = 0; //Sequential revolution reset
      }
    }

    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

      if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
          && revolutionOne
          && configPage4.TrigSpeed == CRANK_SPEED)
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

void triggerSec_Webber(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  if (curGap2 >= triggerSecFilterTime)
  {
    toothLastSecToothTime = curTime2;

    if (secondaryToothCount == 2 && checkSyncToothCount == 3)
    {
      if (!currentStatus.hasSync)
      {
        toothLastToothTime = micros();
        //Fixes RPM at 10rpm until a full revolution has taken place
        toothLastMinusOneToothTime = micros() - 1500000;
        toothCurrentCount = configPage4.triggerTeeth - 1;

        currentStatus.hasSync = true;
      }
      else
      {
        if (toothCurrentCount != configPage4.triggerTeeth - 1U
            && currentStatus.startRevolutions > 2U)
        {
          //Indicates likely sync loss.
          currentStatus.syncLossCounter++;
        }
        if (configPage4.useResync == 1)
        {
          toothCurrentCount = configPage4.triggerTeeth - 1;
        }
      }
      revolutionOne = 1; //Sequential revolution reset
      triggerSecFilterTime = curGap << 2; //4 crank teeth
      secondaryToothCount = 1; //Next tooth should be first
    } //Running, on first CAM pulse restart crank teeth count, on second the counter should be 3
    else if (!currentStatus.hasSync
             && toothCurrentCount >= 3
             && secondaryToothCount == 0)
    {
      toothLastToothTime = micros();
      //Fixes RPM at 10rpm until a full revolution has taken place
      toothLastMinusOneToothTime = micros() - 1500000;
      toothCurrentCount = 1;
      revolutionOne = 1; //Sequential revolution reset

      currentStatus.hasSync = true;
    } //First start, between gaps on CAM pulses have 2 teeth, sync on first CAM pulse if seen 3 teeth or more
    else
    {
      triggerSecFilterTime = curGap + (curGap >> 1); //150% crank tooth
      secondaryToothCount++;
      checkSyncToothCount = 1; //Tooth 1 considered as already been seen
    } //First time might fall here, second CAM tooth will
  }
  else
  {
    triggerSecFilterTime = curGap + (curGap >> 1); //Noise region, using 150% of crank tooth
    checkSyncToothCount = 1; //Reset tooth counter
  } //Trigger filter
}

/** @} */

/** Ford ST170 - a dedicated decoder for 01-04 Ford Focus ST170/SVT engine.
Standard 36-1 trigger wheel running at crank speed and 8-3 trigger wheel running at cam speed.
* @defgroup dec_ford_st170 Ford ST170 (01-04 Focus)
* @{
*/
void triggerSetup_FordST170(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  //Set these as we are using the existing missing tooth primary decoder and these will never change.
  configPage4.triggerTeeth = 36;
  configPage4.triggerMissingTeeth = 1;
  configPage4.TrigSpeed = CRANK_SPEED;

  //The number of degrees that passes from tooth to tooth
  triggerToothAngle = 360 / configPage4.triggerTeeth;
  //The number of physical teeth on the wheel.
  //Doing this here saves us a calculation each time in the interrupt
  triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth;
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = (MICROS_PER_SEC / (MAX_RPM / 60U * configPage4.triggerTeeth));

  //Cam pattern is 8-3, so 2 nearest teeth are 90 deg crank angle apart.
  //Cam can be advanced by 60 deg, so going from fully retarded to fully
  //advanced closes the gap to 30 deg. Zetec cam pulleys aren't keyed from
  //factory, so I subtracted additional 10 deg to avoid filter to be too
  //aggressive. And there you have it 720/20=36.
  triggerSecFilterTime = MICROS_PER_MIN / MAX_RPM / 8U / 2U;

  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);
  checkSyncToothCount = (36) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  secondaryToothCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * (1U + 1U));
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

void triggerSec_FordST170(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if (toothLastSecToothTime == 0)
  {
    curGap2 = 0;
    toothLastSecToothTime = curTime2;
  }

  if (curGap2 >= triggerSecFilterTime)
  {
    //If the time between the current tooth and the last is greater than 1.5x
    //the time between the last tooth and the tooth before that, we make the
    //assertion that we must be at the first tooth after the gap.
    uint32_t const deltaT = toothLastSecToothTime - toothLastMinusOneSecToothTime;

    targetGap2 = deltaT + (deltaT >> 1);
    toothLastMinusOneSecToothTime = toothLastSecToothTime;
    if (curGap2 >= targetGap2 || secondaryToothCount == 5)
    {
      secondaryToothCount = 1;
      revolutionOne = 1; //Sequential revolution reset
      //This is used to prevent a condition where serious intermittent signals
      //(e.g. someone furiously plugging the sensor wire in and out) can leave
      //the filter in an unrecoverable state
      triggerSecFilterTime = 0;
    }
    else
    {
      //Set filter at 25% of the current speed. Filter can only be recalculated
      //for the regular teeth, not the missing one.
      triggerSecFilterTime = curGap2 >> 2;
      secondaryToothCount++;
    }

    toothLastSecToothTime = curTime2;

    //Record the VVT Angle
    //We use the first tooth after the long gap as our reference, this remains in the same engine
    //cycle even when the VVT is at either end of its full swing.
    if (configPage6.vvtEnabled > 0 && revolutionOne && secondaryToothCount == 1)
    {
      int16_t curAngle = getCrankAngle();

      while (curAngle > 360)
      {
        curAngle -= 360;
      }
      if (configPage6.vvtMode == VVT_MODE_CLOSED_LOOP)
      {
        curAngle = ANGLE_FILTER(curAngle << 1, configPage4.ANGLEFILTER_VVT, curAngle);
        currentStatus.vvt1Angle = 360 - curAngle - configPage10.vvtCL0DutyAng;
      }
    }
  } //Trigger filter
}

uint16_t getRPM_FordST170(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (toothCurrentCount != 1)
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes
      //the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }
  return tempRPM;
}

int getCrankAngle_FordST170(void)
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

  //Number of teeth that have passed since tooth 1, multiplied by the angle each
  //tooth represents, plus the angle that tooth 1 is ATDC.
  //This gives accuracy only to the nearest tooth.
  int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

  //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
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

static uint16_t __attribute__((noinline))
calcSetEndTeeth_FordST170(int ignitionAngle, uint8_t toothAdder)
{
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;
#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divTriggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerToothAngle;
#endif
  tempEndTooth = nudge(1, 36U + toothAdder,  tempEndTooth - 1, 36U + toothAdder);

  return clampToActualTeeth((uint16_t)tempEndTooth, toothAdder);
}

static void calcSetEndTeeth_FordST170_ignition(ignition_context_st &ignition)
{
  byte toothAdder = 0;

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
      && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = 36;
  }

  ignition.endTooth = calcSetEndTeeth_FordST170(ignition.endAngle, toothAdder);
}

void triggerSetEndTeeth_FordST170(void)
{
  calcSetEndTeeth_FordST170_ignition(ignitions.ignition(ignChannel1));
  calcSetEndTeeth_FordST170_ignition(ignitions.ignition(ignChannel2));
  calcSetEndTeeth_FordST170_ignition(ignitions.ignition(ignChannel3));
  calcSetEndTeeth_FordST170_ignition(ignitions.ignition(ignChannel4));
  // Removed ign channels >4 as an ST170 engine is a 4 cylinder
}
/** @} */

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

/** Chrysler NGC - a dedicated decoder for vehicles with 4, 6 and 8 cylinder NGC pattern.
4-cyl: 36+2-2 crank wheel and 7 tooth cam
6-cyl: 36-2+2 crank wheel and 12 tooth cam in 6 groups
8-cyl: 36-2+2 crank wheel and 15 tooth cam in 8 groups
The crank decoder uses the polarity of the missing teeth to determine position
The 4-cyl cam decoder uses the polarity of the missing teeth to determine position
The 6 and 8-cyl cam decoder uses the amount of teeth in the two previous groups of teeth to determine position
* @defgroup dec Chrysler NGC - 4, 6 and 8-cylinder
* @{
*/

void triggerSetup_NGC(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_HAS_SECONDARY);

  //Primary trigger
  configPage4.triggerTeeth = 36; //The number of teeth on the wheel incl missing teeth.
  triggerToothAngle = 10; //The number of degrees that passes from tooth to tooth
  //Trigger filter time is the shortest possible time (in uS) that there can be
  //between crank teeth (ie at max RPM).
  //Any pulses that occur faster than this time will be discarded as noise
  triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U) / (360U / triggerToothAngle);
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  toothLastMinusOneToothTime = 0;
  toothLastToothRisingTime = 0;
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle * 2U;

  //Secondary trigger
  if (configPage2.nCylinders == 4)
  {
    //Two nearest edges are 36 degrees apart. Multiply by 2 for half cam speed.
    triggerSecFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U) / (360U / 36U) * 2U;
  }
  else
  {
    //Two nearest edges are 21 degrees apart. Multiply by 2 for half cam speed.
    triggerSecFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U) / (360U / 21U) * 2U;
  }
  secondaryToothCount = 0;
  toothSystemCount = 0;
  toothLastSecToothRisingTime = 0;
  toothLastSecToothTime = 0;
  toothLastMinusOneSecToothTime = 0;

  //toothAngles is reused to store the cam pattern, only used for 6 and 8 cylinder pattern
  if (configPage2.nCylinders == 6)
  {
    toothAngles[0] = 1; // Pos 0 is required to be the same as group 6 for easier math
    toothAngles[1] = 3; // Group 1 ...
    toothAngles[2] = 1;
    toothAngles[3] = 2;
    toothAngles[4] = 3;
    toothAngles[5] = 2;
    toothAngles[6] = 1;
    toothAngles[7] = 3; // Pos 7 is required to be the same as group 1 for easier math
  }
  else if (configPage2.nCylinders == 8)
  {
    toothAngles[0] = 3; // Pos 0 is required to be the same as group 8 for easier math
    toothAngles[1] = 1; // Group 1 ...
    toothAngles[2] = 1;
    toothAngles[3] = 2;
    toothAngles[4] = 3;
    toothAngles[5] = 2;
    toothAngles[6] = 2;
    toothAngles[7] = 1;
    toothAngles[8] = 3;
    toothAngles[9] = 1; // Pos 9 is required to be the same as group 1 for easier math
  }
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

void triggerPri_NGC(void)
{
  curTime = micros();
  // We need to know the polarity of the missing tooth to determine position
  if (Trigger.read())
  {
    toothLastToothRisingTime = curTime;
    return;
  }

  curGap = curTime - toothLastToothTime;
  //Pulses should never be less than triggerFilterTime, so if they are it means
  //a false trigger.
  if (curGap >= triggerFilterTime)
  {
    toothCurrentCount++;
    //Flag this pulse as being a valid trigger (ie that it passed filters)
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
    bool isMissingTooth = false;

    //Make sure we haven't enough tooth information to calculate missing tooth length
    if (toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
    {
      //Only check for missing tooth if we expect this one to be it or if we haven't found one yet
      if (toothCurrentCount == 17
          || toothCurrentCount == 35
          || !(!currentStatus.hasSync && BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)))
      {
        //If the time between the current tooth and the last is greater than 2x
        //the time between the last tooth and the tooth before that, we make the
        //assertion that we must be at the first tooth after the gap
        if (curGap > (toothLastToothTime - toothLastMinusOneToothTime) * 2)
        {
          isMissingTooth = true; //Missing tooth detected
          //This is used to prevent a condition where serious intermittent signals
          //(e.g. someone furiously plugging the sensor wire in and out) can
          //leave the filter in an unrecoverable state
          triggerFilterTime = 0;
          //The tooth angle is double at this point
          BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);

          // Figure out the polarity of the missing tooth by comparing how far ago the last tooth rose
          if (toothLastToothRisingTime - toothLastToothTime < curTime - toothLastToothRisingTime)
          {
            //Just passed the HIGH missing tooth
            toothCurrentCount = 1;

            toothOneMinusOneTime = toothOneTime;
            toothOneTime = curTime;

            if (currentStatus.hasSync)
            {
              currentStatus.startRevolutions++;
            }
            else
            {
              currentStatus.startRevolutions = 0;
            }
          }
          else
          {
            //Just passed the first tooth after the LOW missing tooth
            toothCurrentCount = 19;
          }

          //If Sequential fuel or ignition is in use, further checks are needed before determining sync
          if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL || configPage2.injLayout == INJ_SEQUENTIAL)
          {
            // Verify the tooth counters are valid and use this to determine current revolution
            if ((configPage2.nCylinders == 4 && ((toothCurrentCount == 1 && (secondaryToothCount == 1 || secondaryToothCount == 2)) || (toothCurrentCount == 19 && secondaryToothCount == 4)))
                || (configPage2.nCylinders == 6 && ((toothCurrentCount == 1 && (toothSystemCount == 1    || toothSystemCount == 2)) || (toothCurrentCount == 19 && (toothSystemCount == 2 || toothSystemCount == 3))))
                || (configPage2.nCylinders == 8 && ((toothCurrentCount == 1 && (toothSystemCount == 1    || toothSystemCount == 2)) || (toothCurrentCount == 19 && (toothSystemCount == 3 || toothSystemCount == 4)))))
            {
              revolutionOne = false;
              currentStatus.hasSync = true;
              BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
            }
            else if ((configPage2.nCylinders == 4 && ((toothCurrentCount == 1 && secondaryToothCount == 5) || (toothCurrentCount == 19 && secondaryToothCount == 7)))
                     || (configPage2.nCylinders == 6 && ((toothCurrentCount == 1 && (toothSystemCount == 4 || toothSystemCount == 5)) || (toothCurrentCount == 19 && (toothSystemCount == 5 || toothSystemCount == 6))))
                     || (configPage2.nCylinders == 8 && ((toothCurrentCount == 1 && (toothSystemCount == 5 || toothSystemCount == 6)) || (toothCurrentCount == 19 && (toothSystemCount == 7 || toothSystemCount == 8)))))
            {
              revolutionOne = true;
              currentStatus.hasSync = true;
              //the engine is fully synced so clear the Half Sync bit
              BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
            }
            else
            {
              // If tooth counters are not valid, set half sync bit
              if (currentStatus.hasSync)
              {
                currentStatus.syncLossCounter++;
              }
              currentStatus.hasSync = false;
              //If there is primary trigger but no secondary we only have half sync.
              BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC);
            }
          }
          else //If nothing is using sequential, we have sync and also clear half sync bit
          {
            currentStatus.hasSync = true;
            BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
          }
        }
        else
        {
          // If we have found a missing tooth and don't get the next one at the
          // correct tooth we end up here -> Resync
          if (currentStatus.hasSync)
          {
            currentStatus.syncLossCounter++;
          }
          currentStatus.hasSync = false;
          BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
        }
      }

      if (!isMissingTooth)
      {
        //Regular (non-missing) tooth
        setFilter(curGap);
        BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
      }
    }

    if (isMissingTooth)
    {
      // If we have a missing tooth, copy the gap from the previous tooth as
      // that is the correct normal tooth length
      toothLastMinusOneToothTime = curTime - (toothLastToothTime - toothLastMinusOneToothTime);
    }
    else
    {
      toothLastMinusOneToothTime = toothLastToothTime;
    }
    toothLastToothTime = curTime;

    //NEW IGNITION MODE
    if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
    {
      int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;
      crankAngle = ignitionLimits(crankAngle);
      if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
          && revolutionOne
          && configPage4.TrigSpeed == CRANK_SPEED)
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, configPage4.triggerTeeth + toothCurrentCount);
      }
      else
      {
        checkPerToothTiming(crankAngle, toothCurrentCount);
      }
    }
  }
}

void triggerSec_NGC4(void)
{
  //Only check the cam wheel for sequential operation
  if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && configPage2.injLayout != INJ_SEQUENTIAL)
  {
    return;
  }

  curTime2 = micros();

  // We need to know the polarity of the missing tooth to determine position
  if (Trigger2.read())
  {
    toothLastSecToothRisingTime = curTime2;
    return;
  }

  curGap2 = curTime2 - toothLastSecToothTime;

  if (curGap2 > triggerSecFilterTime)
  {
    //Make sure we have enough tooth information to calculate tooth lengths
    if (toothLastSecToothTime > 0 && toothLastMinusOneSecToothTime > 0)
    {
      if (secondaryToothCount > 0)
      {
        secondaryToothCount++;
      }

      // Check if we have a bigger gap, that is a long tooth
      uint32_t const deltaToothTime = toothLastSecToothTime - toothLastMinusOneSecToothTime;

      if (curGap2 >= deltaToothTime + (deltaToothTime >> 1))
      {
        // Check long tooth polarity
        if (toothLastSecToothRisingTime - toothLastSecToothTime < curTime2 - toothLastSecToothRisingTime)
        {
          //Just passed the HIGH missing tooth
          if (secondaryToothCount == 0 || secondaryToothCount == 8) // synced
          {
            secondaryToothCount = 1;
          }
          else if (secondaryToothCount > 0)
          {
            //Any other number of teeth seen means we missed something or
            //something extra was seen so attempt resync.
            secondaryToothCount = 0;
          }
        }
        else
        {
          //Just passed the first tooth after the LOW missing tooth
          if (secondaryToothCount == 0 || secondaryToothCount == 5)
          {
            secondaryToothCount = 5;
          }
          else if (secondaryToothCount > 0)
          {
            secondaryToothCount = 0;
          }
        }

        //This is used to prevent a condition where serious intermitent signals
        //(e.g. someone furiously plugging the sensor wire in and out) can leave
        //the filter in an unrecoverable state
        triggerSecFilterTime = 0;
      }
      else if (secondaryToothCount > 0)
      {
        //Set filter at 25% of the current speed. Filter can only be recalc'd
        //for the regular teeth, not the missing one.
        triggerSecFilterTime = curGap2 >> 2;
      }
    }

    toothLastMinusOneSecToothTime = toothLastSecToothTime;
    toothLastSecToothTime = curTime2;
  }
}

#define secondaryToothLastCount checkSyncToothCount

void triggerSec_NGC68(void)
{
  //Only check the cam wheel for sequential operation
  if (configPage4.sparkMode != IGN_MODE_SEQUENTIAL && configPage2.injLayout != INJ_SEQUENTIAL)
  {
    return;
  }

  curTime2 = micros();

  curGap2 = curTime2 - toothLastSecToothTime;

  if (curGap2 > triggerSecFilterTime)
  {
    //Make sure we have enough tooth information to calculate tooth lengths
    if (toothLastSecToothTime > 0 && toothLastToothTime > 0 && toothLastMinusOneToothTime > 0)
    {
      /*
       * Cam wheel can have a single tooth in a group which can screw up the "targetgap" calculations
       * Instead use primary wheel tooth gap as comparison as those values are
       * always correct.
       * 2.1 primary teeth are the same duration as one secondary tooth.
       */
      // Check if we have a bigger gap, that is missing teeth
      if (curGap2 >= (3 * (toothLastToothTime - toothLastMinusOneToothTime)))
      {
        //toothSystemCount > 0 means we have cam sync and identifies which group we have synced with
        //toothAngles is reused to store the cam pattern
        // Only check for cam sync if we have actually detected two groups and can get cam sync
        if (secondaryToothCount > 0 && secondaryToothLastCount > 0)
        {
          // Do a quick check if we already have cam sync
          if (toothSystemCount > 0
              && secondaryToothCount == (unsigned int)toothAngles[toothSystemCount + 1])
          {
            toothSystemCount++;
            if (toothSystemCount > configPage2.nCylinders)
            {
              toothSystemCount = 1;
            }
          }
          else
          {
            // Check for a pair of matching groups which tells us which group we are at,
            // this should only happen when we don't have cam sync
            toothSystemCount = 0; // We either haven't got cam sync yet or we lost cam sync
            for (byte group = 1; group <= configPage2.nCylinders; group++)
            {
              // Find a matching pattern/position
              if (secondaryToothCount == (unsigned int)toothAngles[group]
                  && secondaryToothLastCount == (byte)toothAngles[group - 1])
              {
                toothSystemCount = group;
                break;
              }
            }
          }
        }

        secondaryToothLastCount = secondaryToothCount;
        //This is the first tooth in this group
        secondaryToothCount = 1;

        //This is used to prevent a condition where serious intermitent signals
        //(e.g. someone furiously plugging the sensor wire in and out) can leave
        //the filter in an unrecoverable state
        triggerSecFilterTime = 0;
      }
      else if (secondaryToothCount > 0)
      {
        //Normal tooth
        secondaryToothCount++;
        triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed
      }
    }

    toothLastSecToothTime = curTime2;
  }
}

uint16_t getRPM_NGC(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.RPM < currentStatus.crankRPM)
  {
    if (BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      tempRPM = crankingGetRPM(36, CRANK_SPEED);
    }
    else
    {
      //Can't do per tooth RPM if we're at any of the missing teeth as it messes the calculation
      tempRPM = currentStatus.RPM;
    }
  }
  else
  {
    tempRPM = stdGetRPM(CRANK_SPEED);
  }

  return tempRPM;
}

static inline uint16_t calcSetEndTeeth_NGC_SkipMissing(uint16_t toothNum)
{
  if (toothNum == 17U || toothNum == 18U)
  {
    // These are missing teeth, so set the next one before instead
    return 16U;
  }
  if (toothNum == 35U || toothNum == 36U)
  {
    // These are missing teeth, so set the next one before instead
    return 34U;
  }
  if (toothNum == 53U || toothNum == 54U)
  {
    // These are missing teeth, so set the next one before instead
    return 52U;
  }
  if (toothNum > 70U)
  {
    // These are missing teeth, so set the next one before instead
    return 70U;
  }

  return toothNum;

}

static uint16_t __attribute__((noinline))
calcSetEndTeeth_NGC(int ignitionAngle, uint8_t toothAdder)
{
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;

#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divTriggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerToothAngle;
#endif

  return calcSetEndTeeth_NGC_SkipMissing(clampToToothCount(tempEndTooth - 1, toothAdder));
}

static void calcSetEndTeeth_NGC_ignition(ignition_context_st &ignition)
{
  byte toothAdder = 0;
  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
      && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = configPage4.triggerTeeth;
  }

  ignition.endTooth = calcSetEndTeeth_NGC(ignition.endAngle, toothAdder);
}

void triggerSetEndTeeth_NGC(void)
{
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel1));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel2));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel3));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel4));
#if IGN_CHANNELS >= 6
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel5));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel6));
#endif

#if IGN_CHANNELS >= 8
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel7));
  calcSetEndTeeth_NGC_ignition(ignitions.ignition(ignChannel8));
#endif
}

/** Yamaha Vmax 1990+ with 6 uneven teeth, triggering on the wide lobe.
Within the decoder code, the sync tooth is referred to as tooth #1.*
*Derived from Harley and made to work on the Yamaha Vmax.
Trigger is based on 'CHANGE' so we get a signal on the up and downward edges of*
*the lobe. This is required to identify the wide lobe.
* @defgroup dec_vmax Yamaha Vmax
* @{
*/
void triggerSetup_Vmax(bool const initialisationComplete)
{
  triggerToothAngle = 0; // The number of degrees that passes from tooth to tooth, ev. 0. It alternates uneven
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);
  //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = ((MICROS_PER_DEG_1_RPM / minimum_rpm) * 60U);

  if (!initialisationComplete)
  {
    //Set a startup value here to avoid filter errors when starting.
    //This MUST have the initial check to prevent the fuel pump just staying on all the time.
    toothLastToothTime = micros();
  }
  triggerFilterTime = 1500;
  // We must start with a valid trigger or we cannot start measuring the lobe width.
  // We only have a false trigger on the lobe up event when it doesn't pass the filter.
  // Then, the lobe width will also not be measured.
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
  toothAngles[1] = 0;      //tooth #1, these are the absolute tooth positions
  toothAngles[2] = 40;     //tooth #2
  toothAngles[3] = 110;    //tooth #3
  toothAngles[4] = 180;    //tooth #4
  toothAngles[5] = 220;    //tooth #5
  toothAngles[6] = 290;    //tooth #6
}

//curGap = microseconds between primary triggers
//curGap2 = microseconds between secondary triggers
//toothCurrentCount = the current number for the end of a lobe
//secondaryToothCount = the current number of the beginning of a lobe
//We measure the width of a lobe so on the end of a lobe, but want to trigger on
//the beginning. Variable toothCurrentCount tracks the downward events, and
//secondaryToothCount updates on the upward events. Ideally, it should be the
//other way round but the engine stall routine resets secondaryToothCount,
//so it would not sync again after an engine stall.

void triggerPri_Vmax(void)
{
  curTime = micros();
  // Forwarded from the config page to setup the primary trigger edge (rising or falling).
  // Inverting VR-conditioners require FALLING, non-inverting VR-conditioners
  // require RISING in the Trigger edge setup.
  if (Trigger.read() == primaryTriggerEdge)
  {
    curGap2 = curTime;
    curGap = curTime - toothLastToothTime;
    if (curGap >= triggerFilterTime)
    {
      //Flag this pulse as being a valid trigger (ie that it passed filters)
      BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
      if (toothCurrentCount > 0) // We have sync based on the tooth width.
      {
        if (toothCurrentCount == 1)
        {
          secondaryToothCount = 1;
          // Has to be equal to Angle Routine, and describe the delta between two teeth.
          triggerToothAngle = 70;
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;
          currentStatus.hasSync = true;
          //Angle to this tooth is 70, next is in 40, compensating.
          setFilter((curGap * 4) / 7);
          currentStatus.startRevolutions++; //Counter
        }
        else if (toothCurrentCount == 2)
        {
          secondaryToothCount = 2;
          triggerToothAngle = 40;
          //Angle to this tooth is 40, next is in 70, compensating.
          setFilter((curGap * 7) / 4);
        }
        else if (toothCurrentCount == 3)
        {
          secondaryToothCount = 3;
          triggerToothAngle = 70;
          //Angle to this tooth is 70, next is in 70. No need to compensate.
          setFilter(curGap);
        }
        else if (toothCurrentCount == 4)
        {
          secondaryToothCount = 4;
          triggerToothAngle = 70;
          //Angle to this tooth is 70, next is in 40, compensating.
          setFilter((curGap * 4) / 7);
        }
        else if (toothCurrentCount == 5)
        {
          secondaryToothCount = 5;
          triggerToothAngle = 40;
          //Angle to this tooth is 40, next is in 70, compensating.
          setFilter((curGap * 7) / 4);
        }
        else if (toothCurrentCount == 6)
        {
          secondaryToothCount = 6;
          triggerToothAngle = 70;
          //Angle to this tooth is 70, next is in 70. No need to compensate.
          setFilter(curGap);
        }
        toothLastMinusOneToothTime = toothLastToothTime;
        toothLastToothTime = curTime;
        if (triggerFilterTime > 50000) //The first pulse seen
        {
          triggerFilterTime = 0;
        }
      }
      else
      {
        triggerFilterTime = 0;
        return; //Zero, no sync yet.
      }
    }
    else
    {
      BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being an invalid trigger
    }
  }
  else if (BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER))
  {
    // Inverted due to vr conditioner. So this is the falling lobe. We only
    // process if there was a valid trigger.
    unsigned long curGapLocal = curTime - curGap2;
    // Small lobe is 5 degrees, big lobe is 45 degrees. So this should be the wide lobe.
    if (curGapLocal > lastGap * 2)
    {
      //Wide should be seen with toothCurrentCount = 0, when there is no sync yet,
      //or toothCurrentCount = 6 when we have done a full revolution.
      if (toothCurrentCount == 0 || toothCurrentCount == 6)
      {
        currentStatus.hasSync = true;
      }
      else
      {
        //Wide lobe seen where it shouldn't, adding a sync error.
        currentStatus.syncLossCounter++;
      }
      toothCurrentCount = 1;
    }
    else if (toothCurrentCount == 6)
    {
      //The 6th lobe should be wide, adding a sync error.
      toothCurrentCount = 1;
      currentStatus.syncLossCounter++;
    }
    else
    {
      // Small lobe, just add 1 to the toothCurrentCount.
      toothCurrentCount++;
    }
    lastGap = curGapLocal;
  }
  else
  {
    //Reset this every time to ensure we only filter when needed.
    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER);
  }
}

// Needs to be enabled in main()
void triggerSec_Vmax(void)
{
  return; // No need for now. The only thing it could help to sync more quickly
          //or confirm position.
}

uint16_t getRPM_Vmax(void)
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync)
  {
    if (currentStatus.RPM < (configPage4.crankRPM * 100))
    {
      int tempToothAngle;
      unsigned long toothTime;

      if (toothLastToothTime == 0 || toothLastMinusOneToothTime == 0)
      {
        tempRPM = 0;
      }
      else
      {
        noInterrupts();

        tempToothAngle = triggerToothAngle;
        //The time in uS that one revolution would take at current speed
        //(The time tooth 1 was last seen, minus the time it was seen prior to that)
        SetRevolutionTime(toothOneTime - toothOneMinusOneTime);
        toothTime = (toothLastToothTime - toothLastMinusOneToothTime);

        interrupts();

        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * (MICROS_PER_MIN / 10U)) / toothTime;
      }
    }
    else
    {
      tempRPM = stdGetRPM(CRANK_SPEED);
    }
  }

  return tempRPM;
}


int getCrankAngle_Vmax(void)
{
  //This is the current angle ATDC the engine is at. This is the last known
  //position based on what tooth was last 'seen'. It is only accurate to the
  //resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempsecondaryToothCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.

  noInterrupts();

  tempsecondaryToothCount = secondaryToothCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe

  interrupts();

  //Check if the last tooth seen was the reference tooth (Number 3). All others
  //can be calculated, but tooth 3 has a unique angle
  int crankAngle;
  crankAngle = toothAngles[tempsecondaryToothCount] + configPage4.triggerAngle;

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
  crankAngle += timeToAngleDegPerMicroSec(elapsedTime, degreesPerMicro);

  if (crankAngle >= 720)
  {
    crankAngle -= 720;
  }
  if (crankAngle > CRANK_ANGLE_MAX)
  {
    crankAngle -= CRANK_ANGLE_MAX;
  }
  if (crankAngle < 0)
  {
    crankAngle += 360;
  }

  return crankAngle;
}

void triggerSetEndTeeth_Vmax(void)
{
}

/** @} */

/** Renix 44-2-2  and 66-2-2-2 decoder.
* Renix trigger wheel doesn't decode into 360 degrees nicely (360/44 = 8.18 degrees or 360/66 = 5.454545).
* Speeduino can't handle any teeth that have a decimal point.
* Solution is to count teeth, every 11 teeth = a proper angle.
* For 66 tooth decoder its 60 degrees per 11 teeth,
* for 44 tooth decoder it's 90 degrees per 11 teeth.
* This means the system sees 4 teeth on the 44 tooth wheel and 6 teeth on the 66 tooth wheel.
* Double missing tooth in the pattern is actually a large tooth and a large gap.
* If the trigger is set to rising you'll see the start of the large tooth
* then the gap. If its not set to rising the code won't work due to seeing two gaps
*
*
* @defgroup dec_renix Renix decoder
* @{
*/
void triggerSetup_Renix(bool const initialisationComplete)
{
  UNUSED(initialisationComplete);
  if (configPage2.nCylinders == 4)
  {
    //The number of degrees that passes from tooth to tooth (primary) this
    //changes between 41 and 49 degrees
    triggerToothAngle = 90;
    // wheel has 44 teeth but we use these to work out which tooth angle to use,
    // therefore speeduino thinks we only have 8 teeth.
    configPage4.triggerTeeth = 4;
    configPage4.triggerMissingTeeth = 0;
    //The number of teeth we're pretending physically existing on the wheel.
    triggerActualTeeth = 4;
    //Trigger filter time is the shortest possible time (in uS) that there can
    //be between crank teeth (ie at max RPM). Any pulses that occur faster than this
    //time will be disgarded as noise
    triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 44U);
  }
  else if (configPage2.nCylinders == 6)
  {
    triggerToothAngle = 60;
    // wheel has 44 teeth but we use these to work out which tooth angle to use,
    // therefore speeduino thinks we only have 6 teeth.
    configPage4.triggerTeeth = 6;
    configPage4.triggerMissingTeeth = 0;
    //The number of teeth we're pretending physically existing on the wheel.
    triggerActualTeeth = 6;
    //Trigger filter time is the shortest possible time (in uS) that there can
    //be between crank teeth (ie at max RPM).
    //Any pulses that occur faster than this time will be disgarded as noise
    triggerFilterTime = MICROS_PER_SEC / (MAX_RPM / 60U * 66U);
  }

  //Minimum 50rpm. (3333uS is the time per degree at 50rpm).
  //Largest gap between teeth is 90 or 60 degrees depending on decoder.
  unsigned const minimum_rpm = 50;

  MAX_STALL_TIME = (MICROS_PER_DEG_1_RPM / minimum_rpm) * triggerToothAngle;
  BIT_CLEAR(decoderState, BIT_DECODER_HAS_SECONDARY);

  toothSystemCount = 1;
  toothCurrentCount = 1;
  toothLastToothTime = 0;
#ifdef USE_LIBDIVIDE
  divTriggerToothAngle = libdivide::libdivide_s16_gen(triggerToothAngle);
#endif
}

// variables used to help calculate gap on the physical 44 or 66 teeth we're
// pretending don't exist in most of the speeduino code
// reusing existing variables to save storage space as these aren't used in the
// code for their original purpose.
#define renixSystemLastToothTime         toothLastToothRisingTime
#define renixSystemLastMinusOneToothTime toothLastSecToothRisingTime

void triggerPri_Renix(void)
{
  curTime = micros();
  curGap = curTime - renixSystemLastToothTime;

  if (curGap >= triggerFilterTime)
  {
    toothSystemCount++;

    if (renixSystemLastToothTime != 0 && renixSystemLastMinusOneToothTime != 0)
    {
      targetGap = 2 * (renixSystemLastToothTime - renixSystemLastMinusOneToothTime);
    }  // in real world the physical 2 tooth gap is bigger than 2 teeth - more like 2.5
    else
    {
      // random large number to stop system thinking we have a gap for the first
      // few teeth on start up
      targetGap = 100000000L;
    }

    if (curGap >= targetGap)
    {
      /* add two teeth to account for the gap we've just seen */
      toothSystemCount++;
      toothSystemCount++;

      if (toothSystemCount != 12)
      {
        // if not 12 (the first tooth after the gap) then we've lost sync
        // lost sync
        currentStatus.hasSync = false;
        currentStatus.syncLossCounter++;
        toothSystemCount = 1; // first tooth after gap is always 1
        toothCurrentCount = 1; // Reset as we've lost sync
      }
    }
    else
    {
      //Recalc the new filter value, only do this on the single gap tooth
      setFilter(curGap);
    }
    // needed for target gap calculation
    renixSystemLastMinusOneToothTime = renixSystemLastToothTime;
    renixSystemLastToothTime = curTime;

    if (toothSystemCount == 12  || toothLastToothTime == 0)
    {
      // toothLastToothTime used to ensure we set the value so the code that
      // handles the fuel pump in speeduino.ini has a value to use once the engine is running.
      toothCurrentCount++;

      // 6 Pretend teeth on the 66 tooth wheel, if get to severn rotate round back to first tooth
      // 4 Pretend teeth on the 44 tooth wheel, if get to five rotate round back to first tooth
      if ((configPage2.nCylinders == 6 && toothCurrentCount == 7)
          || (configPage2.nCylinders == 4 && toothCurrentCount == 5))
      {
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.hasSync = true;
        currentStatus.startRevolutions++; //Counter
        revolutionOne = !revolutionOne;
        toothCurrentCount = 1;
      }

      toothSystemCount = 1;
      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;


      //NEW IGNITION MODE
      if (configPage2.perToothIgn && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
      {
        int16_t crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle;

        crankAngle = ignitionLimits(crankAngle);
        if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL
            && revolutionOne
            && configPage4.TrigSpeed == CRANK_SPEED)
        {
          crankAngle += 360;
          checkPerToothTiming(crankAngle, configPage4.triggerTeeth + toothCurrentCount);
        }
        else
        {
          checkPerToothTiming(crankAngle, toothCurrentCount);
        }
      }
    }
  }
}

static uint16_t __attribute__((noinline))
calcEndTeeth_Renix(int ignitionAngle, uint8_t toothAdder)
{
  int16_t tempEndTooth = ignitionAngle - configPage4.triggerAngle;

#ifdef USE_LIBDIVIDE
  tempEndTooth = libdivide::libdivide_s16_do(tempEndTooth, &divTriggerToothAngle);
#else
  tempEndTooth = tempEndTooth / (int16_t)triggerToothAngle;
#endif
  tempEndTooth = tempEndTooth - 1;
  // Clamp to tooth count

  return clampToActualTeeth(clampToToothCount(tempEndTooth, toothAdder), toothAdder);
}

static void calcEndTeeth_Renix_ignition(ignition_context_st &ignition)
{
  byte toothAdder = 0;

  if (configPage4.sparkMode == IGN_MODE_SEQUENTIAL && configPage4.TrigSpeed == CRANK_SPEED)
  {
    toothAdder = configPage4.triggerTeeth;
  }

  ignition.endTooth = calcEndTeeth_Renix(ignition.endAngle, toothAdder);
}

void triggerSetEndTeeth_Renix(void)
{
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel1));
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel2));
  currentStatus.canin[1] = ignitions.ignition(ignChannel2).endTooth;
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel3));
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel4));
#if IGN_CHANNELS >= 5
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel5));
#endif
#if IGN_CHANNELS >= 6
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel6));
#endif
#if IGN_CHANNELS >= 7
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel7));
#endif
#if IGN_CHANNELS >= 8
  calcEndTeeth_Renix_ignition(ignitions.ignition(ignChannel8));
#endif
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

