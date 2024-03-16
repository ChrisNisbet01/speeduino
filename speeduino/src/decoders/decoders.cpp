/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
 *
 * Crank and Cam decoders
 *
 * This directory contains the various crank and cam wheel decoder functions.
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
#include "../../globals.h"
#include "decoders.h"
#include "../../ignition_control.h"
#include "../../scheduler.h"
#include "../../crankMaths.h"
#include "../../timers.h"
#include "../../schedule_calcs.h"
#include "../../ignition_contexts.h"
#include "../../bit_macros.h"
#include "triggers.h"
#include "../../utilities.h"
#include "../../crank.h"

#include "null_trigger.h"
#include "missing_tooth.h"
#include "default.h"
#include "null_trigger.h"
#include "missing_tooth.h"
#include "dual_wheel.h"
#include "basic_distributor.h"
#include "gm7x.h"
#include "4g63.h"
#include "24x.h"
#include "jeep_2000.h"
#include "audi_135.h"
#include "honda_d17.h"
#include "miata_9905.h"
#include "mazda_au.h"
#include "non_360.h"
#include "nissan_360.h"
#include "subaru_67.h"
#include "daihatsu_plus1.h"
#include "harley.h"
#include "36_minus_222.h"
#include "36_minus_21.h"
#include "420a.h"
#include "weber.h"
#include "ford_st170.h"
#include "drz400.h"
#include "chrysler_ngc.h"
#include "vmax.h"
#include "renix.h"
#include "rover_mems.h"
#include "suzuki_k6a.h"

decoder_context_st decoder;

void decoder_context_st::initialise_trigger_handler(byte const trigger_pattern)
{
  decoder_handler_st const * flash_ptr;

  switch (trigger_pattern)
  {
  case DECODER_MISSING_TOOTH:
    flash_ptr = &trigger_missing_tooth;
    break;

  case DECODER_BASIC_DISTRIBUTOR:
    flash_ptr = &trigger_basic_distributor;
    break;

  case DECODER_DUAL_WHEEL:
    flash_ptr = &trigger_dual_wheel;
    break;

  case DECODER_GM7X:
    flash_ptr = &trigger_gm7x;
    break;

  case DECODER_4G63:
    flash_ptr = &trigger_4G63;
    break;

  case DECODER_24X:
    flash_ptr = &trigger_24X;
    break;

  case DECODER_JEEP2000:
    flash_ptr = &trigger_jeep_2000;
    break;

  case DECODER_AUDI135:
    flash_ptr = &trigger_audi_135;
    break;

  case DECODER_HONDA_D17:
    flash_ptr = &trigger_honda_d17;
    break;

  case DECODER_MIATA_9905:
    flash_ptr = &trigger_miata_9905;
    break;

  case DECODER_MAZDA_AU:
    flash_ptr = &trigger_mazda_au;
    break;

  case DECODER_NON360:
    flash_ptr = &trigger_non_360;
    break;

  case DECODER_NISSAN_360:
    flash_ptr = &trigger_nissan_360;
    break;

  case DECODER_SUBARU_67:
    flash_ptr = &trigger_subaru_67;
    break;

  case DECODER_DAIHATSU_PLUS1:
    flash_ptr = &trigger_daihatsu_plus1;
    break;

  case DECODER_HARLEY:
    flash_ptr = &trigger_harley;
    break;

  case DECODER_36_2_2_2:
    flash_ptr = &trigger_36_minus_222;
    break;

  case DECODER_36_2_1:
    flash_ptr = &trigger_36_minus_21;
    break;

  case DECODER_420A:
    flash_ptr = &trigger_420a;
    break;

  case DECODER_WEBER:
    flash_ptr = &trigger_weber;
    break;

  case DECODER_ST170:
    flash_ptr = &trigger_st170;
    break;

  case DECODER_DRZ400:
    flash_ptr = &trigger_drz400;
    break;

  case DECODER_NGC:
    if (configPage2.nCylinders == 4)
    {
      flash_ptr = &trigger_ngc_4;
    }
    else
    {
      flash_ptr = &trigger_ngc_68;
    }
    break;

  case DECODER_VMAX:
    flash_ptr = &trigger_vmax;
    break;

  case DECODER_RENIX:
    flash_ptr = &trigger_renix;
    break;

  case DECODER_ROVERMEMS:
    flash_ptr = &trigger_rover_mems;
    break;

  case DECODER_SUZUKI_K6A:
    flash_ptr = &trigger_suzuki_k6a;
    break;

  default:
    flash_ptr = &trigger_default;
    break;
  }

  memcpy_P(&decoder.handler, flash_ptr, sizeof(decoder.handler));
}

static inline bool HasAnySync(const statuses &status)
{
  return status.hasSync || BIT_CHECK(status.status3, BIT_STATUS3_HALFSYNC);
}

volatile uint32_t curTime;
volatile uint32_t curGap;
volatile uint32_t curTime2;
volatile uint32_t curGap2;
volatile uint32_t curTime3;
volatile uint32_t curGap3;
volatile uint32_t lastGap;
volatile uint32_t targetGap;

//The maximum time (in uS) that the system will continue to function before the
//engine is considered stalled/stopped. This is unique to each decoder,
//depending on the number of teeth etc. 500000 (half a second) is used as the
//default value, most decoders will be much less.
uint32_t MAX_STALL_TIME = MICROS_PER_SEC / 2U;
//The current number of teeth (Once sync has been achieved, this can never actually be 0)
volatile uint16_t toothCurrentCount = 0;
//Used for decoders such as Audi 135 where not every tooth is used for
//calculating crank angle. This variable stores the actual number of teeth,
//not the number being used to calculate crank angle
volatile byte toothSystemCount = 0;
//As below, but used for decoders where not every tooth count is used for calculation
volatile uint32_t toothSystemLastToothTime = 0;
//The time (micros()) that the last tooth was registered
volatile uint32_t toothLastToothTime = 0;
//The time (micros()) that the last tooth was registered on the secondary input
volatile uint32_t toothLastSecToothTime = 0;
//The time (micros()) that the last tooth was registered on the second cam input
volatile uint32_t toothLastThirdToothTime = 0;
//The time (micros()) that the tooth before the last tooth was registered
volatile uint32_t toothLastMinusOneToothTime = 0;
//The time (micros()) that the tooth before the last tooth was registered on secondary input
volatile uint32_t toothLastMinusOneSecToothTime = 0;
//The time (micros()) that the last tooth rose (used by special decoders to
//determine missing teeth polarity)
volatile uint32_t toothLastToothRisingTime = 0;
//The time (micros()) that the last tooth rose on the secondary input (used by
//special decoders to determine missing teeth polarity)
volatile uint32_t toothLastSecToothRisingTime = 0;
volatile uint32_t targetGap2;
volatile uint32_t targetGap3;
//The time (micros()) that tooth 1 last triggered
volatile uint32_t toothOneTime = 0;
//The 2nd to last time (micros()) that tooth 1 last triggered
volatile uint32_t toothOneMinusOneTime = 0;
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
volatile uint32_t secondaryLastToothTime = 0;
//The time (micros()) that the last tooth was registered (Cam input)
volatile uint32_t secondaryLastToothTime1 = 0;

//Used for identifying the current third (Usually exhaust cam - used for VVT2) tooth for patterns with multiple secondary teeth
volatile unsigned int thirdToothCount;
//The time (micros()) that the last tooth was registered (Cam input)
volatile uint32_t thirdLastToothTime = 0;
//The time (micros()) that the last tooth was registered (Cam input)
volatile uint32_t thirdLastToothTime1 = 0;

uint16_t triggerActualTeeth;
// The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
volatile uint32_t triggerFilterTime;
// The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
// for the secondary input
volatile uint32_t triggerSecFilterTime;
// The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
// for the Third input
volatile uint32_t triggerThirdFilterTime;

volatile uint8_t decoderState = 0;

// The shortest valid time (in uS) pulse DURATION
unsigned int triggerSecFilterTime_duration;
//The number of crank degrees that elapse per tooth
volatile uint16_t triggerToothAngle;
//How many teeth must've been seen on this revolution before we try to confirm sync
//(Useful for missing tooth type decoders)
byte checkSyncToothCount;
//The time between the vvt reference pulse and the last crank pulse
uint32_t lastVVTtime;

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
addToothLogEntry(uint32_t toothTime, tooth_source_t whichTooth)
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
    decoder.handler.primaryToothHandler();
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
    decoder.handler.secondaryToothHandler();
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
    decoder.handler.tertiaryToothHandler();
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
  return status.RPM < status.crankRPM && status.startRevolutions == 0U;
}

__attribute__((noinline))
bool SetRevolutionTime(uint32_t revTime)
{
  return crank.setRevolutionTime(revTime);
}

bool UpdateRevolutionTimeFromTeeth(bool isCamTeeth)
{
  noInterrupts();

  bool haveUpdatedRevTime =
    HasAnySync(currentStatus)
    && !IsCranking(currentStatus)
    && toothOneMinusOneTime != UINT32_C(0)
    && toothOneTime > toothOneMinusOneTime
    //The time in uS that one revolution would take at current speed
    //(The time tooth 1 was last seen, minus the time it was seen prior to that)
    && SetRevolutionTime((toothOneTime - toothOneMinusOneTime) >> isCamTeeth);

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

      uint32_t const temp =
        ((toothLastToothTime - toothLastMinusOneToothTime) * totalTeeth) >> isCamTeeth;

      bool const setNewRevtime = SetRevolutionTime(temp);

      interrupts();

      if (setNewRevtime)
      {
        return RpmFromRevolutionTimeUs(crank.revolutionTime);
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
    return RpmFromRevolutionTimeUs(crank.revolutionTime);
  }

  return currentStatus.RPM;
}

