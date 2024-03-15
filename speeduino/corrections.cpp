/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
Corrections to injection pulsewidth.
The corrections functions in this file affect the fuel pulsewidth (Either increasing or decreasing)
based on factors other than the VE lookup.

These factors include:
- Temperature (Warmup Enrichment and After Start Enrichment)
- Acceleration/Deceleration
- Flood clear mode
- etc.

Most correction functions return value 100 (like 100% == 1) for no need for correction.

There are 2 top level functions that call more detailed corrections for Fuel and Ignition respectively:
- @ref correctionsFuel() - All fuel related corrections
- @ref correctionsIgn() - All ignition related corrections
*/
//************************************************************************************************************

#include "globals.h"
#include "corrections.h"
#include "speeduino.h"
#include "timers.h"
#include "maths.h"
#include "sensors.h"
#include "src/PID_v1/PID_v1.h"
#include "tps_dot.h"
#include "map_dot.h"
#include "bit_macros.h"
#include "utilities.h"

long PID_O2, PID_output, PID_AFRTarget;
/** Instance of the PID object in case that algorithm is used (Always instantiated).
* Needs to be global as it maintains state outside of each function call.
* Comes from Arduino (?) PID library.
*/
PID egoPID(&PID_O2, &PID_output, &PID_AFRTarget, configPage6.egoKP, configPage6.egoKI, configPage6.egoKD, REVERSE);

byte activateMAPDOT; //The mapDOT value seen when the MAE was activated.
byte activateTPSDOT; //The tpsDOT value seen when the MAE was activated.

bool idleAdvActive = false;
uint16_t AFRnextCycle;
uint32_t knockStartTime;
byte lastKnockCount;
int16_t knockWindowMin; //The current minimum crank angle for a knock pulse to be valid
int16_t knockWindowMax;//The current maximum crank angle for a knock pulse to be valid
uint8_t aseTaper;
uint8_t dfcoDelay;
uint8_t idleAdvTaper;
uint8_t crankingEnrichTaper;
uint8_t dfcoTaper;

/** Initialise instances and vars related to corrections (at ECU boot-up).
 */
void initialiseCorrections(void)
{
  egoPID.SetMode(AUTOMATIC); //Turn O2 PID on
  currentStatus.flexIgnCorrection = 0;
  //Default value of no adjustment must be set to avoid randomness on first
  //correction cycle after startup
  currentStatus.egoCorrection = 100;
  AFRnextCycle = 0;
  currentStatus.knockActive = false;
  //Set battery voltage to sensible value for dwell correction for "flying start"
  //(else ignition gets spurious pulses after boot)
  currentStatus.battery10 = 125;
}

/** Dispatch calculations for all fuel related corrections.
Calls all the other corrections functions and combines their results.
This is the only function that should be called from anywhere outside the file
*/
uint16_t correctionsFuel(void)
{
  static uint16_t const correction_scale_factor = 7;
  uint32_t fuel_corrections_percent = 100 << correction_scale_factor;
  uint16_t result; //temporary variable to store the result of each corrections function

  //The values returned by each of the correction functions are multiplied together
  //and then divided back to give a single 0-255 value.
  currentStatus.wueCorrection = correctionWUE();
  fuel_corrections_percent = percentage(currentStatus.wueCorrection, fuel_corrections_percent);

  currentStatus.ASEValue = correctionASE();
  fuel_corrections_percent = percentage(currentStatus.ASEValue, fuel_corrections_percent);

  result = correctionCranking();
  fuel_corrections_percent = percentage(result, fuel_corrections_percent);

  currentStatus.AEamount = correctionAccel();
  // multiply by the AE amount in case of multiplier AE mode or Decel
  if (configPage2.aeApplyMode == AE_MODE_MULTIPLIER
      || BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC))
  {
    fuel_corrections_percent = percentage(currentStatus.AEamount, fuel_corrections_percent);
  }

  result = correctionFloodClear();
  fuel_corrections_percent = percentage(result, fuel_corrections_percent);

  currentStatus.egoCorrection = correctionAFRClosedLoop();
  fuel_corrections_percent = percentage(currentStatus.egoCorrection, fuel_corrections_percent);

  currentStatus.batCorrection = correctionBatVoltage();
  if (configPage2.battVCorMode == BATTV_COR_MODE_OPENTIME)
  {
     // Apply voltage correction to injector open time.
    inj_opentime_uS = configPage2.injOpen * currentStatus.batCorrection;
    // This is to ensure that the correction is not applied twice.
    //There is no battery correction factor as we have instead changed the open time
    currentStatus.batCorrection = 100;

  }
  if (configPage2.battVCorMode == BATTV_COR_MODE_WHOLE)
  {
    fuel_corrections_percent = percentage(currentStatus.batCorrection, fuel_corrections_percent);
  }

  currentStatus.iatCorrection = correctionIATDensity();
  fuel_corrections_percent = percentage(currentStatus.iatCorrection, fuel_corrections_percent);

  currentStatus.baroCorrection = correctionBaro();
  fuel_corrections_percent = percentage(currentStatus.baroCorrection, fuel_corrections_percent);

  currentStatus.flexCorrection = correctionFlex();
  fuel_corrections_percent = percentage(currentStatus.flexCorrection, fuel_corrections_percent);

  currentStatus.fuelTempCorrection = correctionFuelTemp();
  fuel_corrections_percent = percentage(currentStatus.fuelTempCorrection, fuel_corrections_percent);

  currentStatus.launchCorrection = correctionLaunch();
  fuel_corrections_percent = percentage(currentStatus.launchCorrection, fuel_corrections_percent);

  bitWrite(currentStatus.status1, BIT_STATUS1_DFCO, correctionDFCO());
  byte dfcoTaperCorrection = correctionDFCOfuel();
  fuel_corrections_percent = percentage(dfcoTaperCorrection, fuel_corrections_percent);

  /* Now scale the correction back to normal percentage. */
  fuel_corrections_percent >>= correction_scale_factor;

  //This is the maximum allowable increase during cranking
  uint32_t const max_fuel_corrections = 1500;

  if (fuel_corrections_percent > max_fuel_corrections)
  {
    fuel_corrections_percent = max_fuel_corrections;
  }

  return fuel_corrections_percent;
}

/** Warm Up Enrichment (WUE) corrections.
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add
*/
byte correctionWUE(void)
{
  byte WUEValue;

  //Possibly reduce the frequency this runs at (Costs about 50 loops per second)
  if (currentStatus.coolant > table2D_getAxisValue(&WUETable, 9) - CALIBRATION_TEMPERATURE_OFFSET)
  {
    //This avoids doing the 2D lookup if we're already up to temp
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = table2D_getRawValue(&WUETable, 9);
  }
  else
  {
    BIT_SET(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = table2D_getValue(&WUETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  }

  return WUEValue;
}

/** Cranking Enrichment corrections.
Additional fuel % to be added when the engine is cranking
*/
uint16_t correctionCranking(void)
{
  uint16_t crankingValue = 100;

  //Check if we are actually cranking
  if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
  {
    crankingValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    crankingValue = (uint16_t) crankingValue * 5; //multiplied by 5 to get range from 0% to 1275%
    crankingEnrichTaper = 0;
  }

  //If we're not cranking, check if if cranking enrichment tapering to ASE should be done
  else if (crankingEnrichTaper < configPage10.crankingEnrichTaper)
  {
    crankingValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    crankingValue = (uint16_t) crankingValue * 5; //multiplied by 5 to get range from 0% to 1275%
    //Taper start value needs to account for ASE that is now running, so total correction does not increase when taper begins
    uint32_t taperStart = (uint32_t) crankingValue * 100 / currentStatus.ASEValue;
    crankingValue = (uint16_t) map(crankingEnrichTaper, 0, configPage10.crankingEnrichTaper, taperStart, 100); //Taper from start value to 100%
    if (crankingValue < 100) { crankingValue = 100; } //Sanity check
    if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { crankingEnrichTaper++; }
  }
  return crankingValue;
}

/** After Start Enrichment calculation.
 * This is a short period (Usually <20 seconds) immediately after the engine first fires (But not when cranking)
 * where an additional amount of fuel is added (Over and above the WUE amount).
 *
 * @return uint8_t The After Start Enrichment modifier as a %. 100% = No modification.
 */
byte correctionASE(void)
{
  int16_t ASEValue = currentStatus.ASEValue;
  //Two checks are required:
  //1) Is the engine run time less than the configured ase time
  //2) Make sure we're not still cranking
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) != true )
  {
    if ( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) || (currentStatus.ASEValue == 0) )
    {
      if (currentStatus.runSecs < (table2D_getValue(&ASECountTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET))
          && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))
      {
        BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
        ASEValue = 100 + table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
        aseTaper = 0;
      }
      else
      {
        if (aseTaper < configPage2.aseTaperTime) //Check if we've reached the end of the taper time
        {
          BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
          ASEValue = 100 + map(aseTaper, 0, configPage2.aseTaperTime, table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET), 0);
          aseTaper++;
        }
        else
        {
          BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
          ASEValue = 100;
        }
      }

      //Safety checks
      if(ASEValue > 255) { ASEValue = 255; }
      if(ASEValue < 0) { ASEValue = 0; }
      ASEValue = (byte)ASEValue;
    }
  }
  else
  {
    //Engine is cranking, ASE disabled
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
    ASEValue = 100;
  }
  return ASEValue;
}

static int16_t AEColdAdjustment(void)
{
  int16_t accelValue;

  //Apply AE cold coolant modifier, if CLT is less than taper end temperature
  if (currentStatus.coolant >= (int)(configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET))
  {
    accelValue = 100;
    goto done;
  }

  //If CLT is less than taper min temp, apply full modifier
  if (currentStatus.coolant <= (int)(configPage2.aeColdTaperMin - CALIBRATION_TEMPERATURE_OFFSET))
  {
    accelValue = configPage2.aeColdPct;
    goto done;
  }

  /*
   * CLT must be somewhere between taper min and taper max.
   * Avoid compiler warnings about variable initialisation by putting positive
   * DOT code into its own scope.
   */
  {
    //If CLT is between taper min and max, taper the modifier value
    int16_t taperRange = (int16_t)configPage2.aeColdTaperMax - configPage2.aeColdTaperMin;
    int16_t taperPercent =
      (int)((currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET - configPage2.aeColdTaperMin) * 100) / taperRange;

    accelValue = (int16_t)100 + percentage(100 - taperPercent, configPage2.aeColdPct - 100);
    //Potential overflow (if AE is large) without using uint16_t (percentage() may overflow)
  }

done:
  return accelValue;
}

static int16_t doAECalculation(
  int16_t const DOT, byte const threshold, struct table2D &table, byte &activateDOT)
{
  int16_t accelValue;

  if (abs(DOT) <= threshold)
  {
    accelValue = 0;
    goto done;
  }

  //If AE isn't currently turned on, need to check whether it needs to be turned on
  activateDOT = abs(DOT);
  //Set the time in the future where the enrichment will be turned off.
  //aeTime is stored as mS / 10, so multiply it by 10000 to get it in uS
  currentStatus.AEEndTime = micros_safe() + ((uint32_t)configPage2.aeTime * 10000);

  //Check if the rate of change is negative or positive. Negative means deceleration.
  if (DOT < 0)
  {
    //Deceleration
    BIT_SET(currentStatus.engine, BIT_ENGINE_DCC);
    accelValue = configPage2.decelAmount; //In decel, use the decel fuel amount as accelValue
    /* Hmm, should the AEEndtime also apply to deceleration? */
    goto done;
  }

  /*
   * Must be positive rate of change.
   * Avoid compiler warnings about variable initialisation by putting positive
   * DOT code into its own scope.
   */
  {
    //Positive rate of change is acceleration.
    BIT_SET(currentStatus.engine, BIT_ENGINE_ACC);
    //The x-axis of ae table is divided by 10 to fit values into a byte.
    accelValue = table2D_getValue(&table, DOT / 10);

    //Apply the RPM taper to the above
    //The RPM settings are stored divided by 100:
    uint16_t trueTaperMin = configPage2.aeTaperMin * 100;
    uint16_t trueTaperMax = configPage2.aeTaperMax * 100;

    if (currentStatus.RPM > trueTaperMax)
    {
      //RPM is beyond taper max limit, so accel enrich is turned off
      // No need to check for cold adjustment as any percentage of 0 is still 0.
      accelValue = 0;
      goto done;
    }

    if (currentStatus.RPM > trueTaperMin)
    {
      int16_t const taperRange = trueTaperMax - trueTaperMin;
      //The percentage of the way through the RPM taper range
      int16_t const taperPercent =
        ((currentStatus.RPM - trueTaperMin) * 100UL) / taperRange;

      //Calculate the above percentage of the calculated accel amount.
      accelValue = percentage(100 - taperPercent, accelValue);
    }

    accelValue = percentage(AEColdAdjustment(), accelValue);
  }

done:
  return accelValue + 100; //Add the 100 normalisation to the calculated amount;
}

/** Acceleration enrichment correction calculation.
 *
 * Calculates the % change of the throttle over time (%/second) and performs a lookup based on this
 * Coolant-based modifier is applied on the top of this.
 * When the enrichment is turned on, it runs at that amount for a fixed period of time (taeTime)
 *
 * @return uint16_t The Acceleration enrichment modifier as a %. 100% = No modification.
 *
 * As the maximum enrichment amount is +255% and maximum cold adjustment for this is 255%, the overall return value
 * from this function can be 100+(255*255/100)=750. Hence this function returns a uint16_t rather than byte.
 */
uint16_t correctionAccel(void)
{
  int16_t accelValue = 100;

  //First, check whether the accel. enrichment is already running
  if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)
      || BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC))
  {
    //If it is currently running, check whether it should still be running or
    //whether it's reached it's end time
    if ((long)(micros_safe() - currentStatus.AEEndTime) > 0)
    {
      //Time to turn enrichment off
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC);
      currentStatus.AEamount = 0;
      accelValue = 100;
    }
    else
    {
      //Enrichment still needs to keep running.
      //Simply return the total TAE amount
      accelValue = currentStatus.AEamount;

      //Need to check whether the accel amount has increased from when AE was turned on
      //If the accel amount HAS increased, we clear the current enrich phase and a new one will be started below
      if ((configPage2.aeMode == AE_MODE_MAP && abs(currentStatus.mapDOT) > activateMAPDOT)
          || (configPage2.aeMode == AE_MODE_TPS && abs(currentStatus.tpsDOT) > activateTPSDOT))
      {
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC);
      }
    }
  }

  //Need to check this again as it may have been changed in the above section
  //(Both ACC and DCC are off if this has changed)
  if (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)
      && !BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC))
  {
    if(configPage2.aeMode == AE_MODE_MAP)
    {
      accelValue =
        doAECalculation(currentStatus.mapDOT, configPage2.maeThresh, maeTable, activateMAPDOT);
    }
    else if(configPage2.aeMode == AE_MODE_TPS)
    {
      accelValue =
        doAECalculation(currentStatus.tpsDOT, configPage2.taeThresh, taeTable, activateTPSDOT);
    }
    else
    {
      accelValue = 100;
    }
  } //AE active

  return accelValue;
}

/** Simple check to see whether we are cranking with the TPS above the flood clear threshold.
@return 100 (not cranking and thus no need for flood-clear) or 0 (Engine cranking and TPS above @ref config4.floodClear limit).
*/
byte correctionFloodClear(void)
{
  byte floodValue = 100;
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    //Engine is currently cranking, check what the TPS is
    if(currentStatus.TPS >= configPage4.floodClear)
    {
      //Engine is cranking and TPS is above threshold. Cut all fuel
      floodValue = 0;
    }
  }
  return floodValue;
}

/** Battery Voltage correction.
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add.
*/
byte correctionBatVoltage(void)
{
  byte batValue = 100;
  batValue = table2D_getValue(&injectorVCorrectionTable, currentStatus.battery10);
  return batValue;
}

/** Simple temperature based corrections lookup based on the inlet air temperature (IAT).
This corrects for changes in air density from movement of the temperature.
*/
byte correctionIATDensity(void)
{
  byte IATValue = 100;
  IATValue = table2D_getValue(&IATDensityCorrectionTable, currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //currentStatus.IAT is the actual temperature, values in IATDensityCorrectionTable.axisX are temp+offset

  return IATValue;
}

/** Correction for current barometric / ambient pressure.
 * @returns A percentage value indicating the amount the fuelling should be changed based on the barometric reading. 100 = No change. 110 = 10% increase. 90 = 10% decrease
 */
byte correctionBaro(void)
{
  byte baroValue = 100;
  baroValue = table2D_getValue(&baroFuelTable, currentStatus.baro);

  return baroValue;
}

/** Launch control has a setting to increase the fuel load to assist in bringing up boost.
This simple check applies the extra fuel if we're currently launching
*/
byte correctionLaunch(void)
{
  byte launchValue = 100;
  if(currentStatus.launchingHard || currentStatus.launchingSoft) { launchValue = (100 + configPage6.lnchFuelAdd); }

  return launchValue;
}

/**
*/
byte correctionDFCOfuel(void)
{
  byte scaleValue = 100;
  if ( BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) )
  {
    if ( (configPage9.dfcoTaperEnable == 1) && (dfcoTaper != 0) )
    {
      //Do a check if the user reduced the duration while active to avoid overflow
      if (dfcoTaper > configPage9.dfcoTaperTime) { dfcoTaper = configPage9.dfcoTaperTime; }
      scaleValue = map(dfcoTaper, configPage9.dfcoTaperTime, 0, 100, configPage9.dfcoTaperFuel);
      if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { dfcoTaper--; }
    }
    else { scaleValue = 0; } //Taper ended or disabled, disable fuel
  }
  else { dfcoTaper = configPage9.dfcoTaperTime; } //Keep updating the duration until DFCO is active

  return scaleValue;
}

/*
 * Returns true if deceleration fuel cutoff should be on, false if its off
 */
bool correctionDFCO(void)
{
  bool DFCOValue = false;

  if (configPage2.dfcoEnabled == 1)
  {
    if (BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO))
    {
      DFCOValue = currentStatus.RPM > (uint16_t)configPage4.dfcoRPM * 10
                  && currentStatus.TPS < configPage4.dfcoTPSThresh;

      if (!DFCOValue)
      {
        dfcoDelay = 0;
      }
    }
    else
    {
      if (currentStatus.TPS < configPage4.dfcoTPSThresh
          && currentStatus.coolant >= (int)configPage2.dfcoMinCLT - CALIBRATION_TEMPERATURE_OFFSET
          && currentStatus.RPM > (unsigned int)configPage4.dfcoRPM * 10 + (unsigned int)configPage4.dfcoHyster * 2)
      {
        if (dfcoDelay < configPage2.dfcoDelay)
        {
          if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ))
          {
            dfcoDelay++;
          }
        }
        else
        {
          DFCOValue = true;
        }
      }
      else
      {
        //Prevent future activation right away if previous time wasn't activated
        dfcoDelay = 0;
      }
    } // DFCO active check
  } // DFCO enabled check

  return DFCOValue;
}

/** Flex fuel adjustment to vary fuel based on ethanol content.
 * The amount of extra fuel required is a linear relationship based on the % of ethanol.
*/
byte correctionFlex(void)
{
  byte flexValue = 100;

  if (configPage2.flexEnabled == 1)
  {
    flexValue = table2D_getValue(&flexFuelTable, currentStatus.ethanolPct);
  }
  return flexValue;
}

/*
 * Fuel temperature adjustment to vary fuel based on fuel temperature reading
*/
byte correctionFuelTemp(void)
{
  byte fuelTempValue = 100;

  if (configPage2.flexEnabled == 1)
  {
    fuelTempValue = table2D_getValue(&fuelTempTable, currentStatus.fuelTemp + CALIBRATION_TEMPERATURE_OFFSET);
  }
  return fuelTempValue;
}

/** Lookup the AFR target table and perform either a simple or PID adjustment based on this.

Simple (Best suited to narrowband sensors):
If the O2 sensor reports that the mixture is lean/rich compared to the desired AFR target, it will make a 1% adjustment
It then waits egoDelta number of ignition events and compares O2 against the target table again. If it is still lean/rich then the adjustment is increased to 2%.

This continues until either:
- the O2 reading flips from lean to rich, at which point the adjustment cycle starts again at 1% or
- the adjustment amount increases to egoLimit at which point it stays at this level until the O2 state (rich/lean) changes

PID (Best suited to wideband sensors):

*/
byte correctionAFRClosedLoop(void)
{
  byte AFRValue = 100;

  //afrTarget value lookup must be done if O2 sensor is enabled,
  //and always if incorporateAFR is enabled
  if (configPage6.egoType > 0 || configPage2.incorporateAFR)
  {
    currentStatus.afrTarget = currentStatus.O2; //Catch all in case the below doesn't run. This prevents the Include AFR option from doing crazy things if the AFR target conditions aren't met. This value is changed again below if all conditions are met.

    //Determine whether the Y axis of the AFR target table tshould be MAP (Speed-Density) or TPS (Alpha-N)
    //Note that this should only run after the sensor warmup delay when using Include AFR option, but on Incorporate AFR option it needs to be done at all times
    if (currentStatus.runSecs > configPage6.ego_sdelay || configPage2.incorporateAFR)
    {
      //Perform the target lookup
      currentStatus.afrTarget = get3DTableValue(&afrTable, currentStatus.fuelLoad, currentStatus.RPM);
    }
  }

  //egoType of 0 means no O2 sensor.
  //If DFCO is active do not run the ego controllers to prevent integrator wind-up.
  if (configPage6.egoType > 0 && !BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO))
  {
    //Need to record this here, just to make sure the correction stays 'on'
    //even if the nextCycle count isn't ready
    AFRValue = currentStatus.egoCorrection;

    if (ignitionCount >= AFRnextCycle || ignitionCount < AFRnextCycle - configPage6.egoCount)
    {
      //Set the target ignition event for the next calculation
      AFRnextCycle = ignitionCount + configPage6.egoCount;

      //Check all other requirements for closed loop adjustments
      if (currentStatus.coolant > (int)(configPage6.egoTemp - CALIBRATION_TEMPERATURE_OFFSET)
          && currentStatus.RPM > (uint16_t)configPage6.egoRPM * 100
          && currentStatus.TPS <= configPage6.egoTPSMax
          && currentStatus.O2 < configPage6.ego_max
          && currentStatus.O2 > configPage6.ego_min
          && currentStatus.runSecs > configPage6.ego_sdelay
          &&  !BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO)
          && currentStatus.MAP <= configPage9.egoMAPMax * 2U
          && currentStatus.MAP >= configPage9.egoMAPMin * 2U)
      {

        //Check which algorithm is used, simple or PID
        if (configPage6.egoAlgorithm == EGO_ALGORITHM_SIMPLE)
        {
          //*************************************************************************************************************************************
          //Simple algorithm
          if(currentStatus.O2 > currentStatus.afrTarget)
          {
            //Running lean
            if(currentStatus.egoCorrection < (100 + configPage6.egoLimit) ) //Fuelling adjustment must be at most the egoLimit amount (up or down)
            {
              AFRValue = (currentStatus.egoCorrection + 1); //Increase the fuelling by 1%
            }
            else { AFRValue = currentStatus.egoCorrection; } //Means we're at the maximum adjustment amount, so simply return that again
          }
          else if(currentStatus.O2 < currentStatus.afrTarget)
          {
            //Running Rich
            if(currentStatus.egoCorrection > (100 - configPage6.egoLimit) ) //Fuelling adjustment must be at most the egoLimit amount (up or down)
            {
              AFRValue = (currentStatus.egoCorrection - 1); //Decrease the fuelling by 1%
            }
            else { AFRValue = currentStatus.egoCorrection; } //Means we're at the maximum adjustment amount, so simply return that again
          }
          else { AFRValue = currentStatus.egoCorrection; } //Means we're already right on target

        }
        else if(configPage6.egoAlgorithm == EGO_ALGORITHM_PID)
        {
          //*************************************************************************************************************************************
          //PID algorithm
          egoPID.SetOutputLimits((long)(-configPage6.egoLimit), (long)(configPage6.egoLimit)); //Set the limits again, just in case the user has changed them since the last loop. Note that these are sent to the PID library as (Eg:) -15 and +15
          egoPID.SetTunings(configPage6.egoKP, configPage6.egoKI, configPage6.egoKD); //Set the PID values again, just in case the user has changed them since the last loop
          PID_O2 = (long)(currentStatus.O2);
          PID_AFRTarget = (long)(currentStatus.afrTarget);

          bool PID_compute = egoPID.Compute();
          //currentStatus.egoCorrection = 100 + PID_output;
          if (PID_compute)
          {
            AFRValue = 100 + PID_output;
          }

        }
        else { AFRValue = 100; } // Occurs if the egoAlgorithm is set to 0 (No Correction)
      } //Multi variable check
      else { AFRValue = 100; } // If multivariable check fails disable correction
    } //Ignition count check
  } //egoType

  return AFRValue; //Catch all (Includes when AFR target = current AFR
}

//******************************** IGNITION ADVANCE CORRECTIONS ********************************
/** Correct ignition timing to configured fixed value.
 */
static bool correctionFixedTiming(int8_t &fixed_advance)
{
  bool const is_fixed = configPage2.fixAngEnable == 1;

  if (is_fixed)
  {
    fixed_advance = configPage4.FixAng;
  }

  return is_fixed;
}

/** Correct ignition timing to configured fixed value to use during cranking.
 */
static bool correctionCrankingFixedTiming(int8_t &fixed_advance)
{
  bool const is_fixed = BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK);

  if (is_fixed)
  {
    if (configPage2.crkngAddCLTAdv != 0)
    {
      fixed_advance = correctionCLTadvance(configPage4.CrankAng);
    }
    else //Use the CLT compensated cranking ignition angle
    {
      fixed_advance = configPage4.CrankAng;
    }
  }

  return is_fixed;
}

/** Dispatch calculations for all ignition related corrections.
 * @param base_advance - Base ignition advance (deg. ?)
 * @return Advance considering all (~12) individual corrections
 */
int8_t correctionsIgn(int8_t base_advance)
{
  int8_t advance;

  if (correctionCrankingFixedTiming(advance))
  {
    goto done;
  }

  if (correctionFixedTiming(advance))
  {
    goto done;
  }

  advance = correctionFlexTiming(base_advance);
  advance = correctionWMITiming(advance);
  advance = correctionIATretard(advance);
  advance = correctionCLTadvance(advance);
  advance = correctionIdleAdvance(advance);
  advance = correctionSoftRevLimit(advance);
  advance = correctionNitrous(advance);
  advance = correctionSoftLaunch(advance);
  advance = correctionSoftFlatShift(advance);
  advance = correctionKnock(advance);
  advance = correctionDFCOignition(advance);

done:
  return advance;
}

int8_t correctionFlexTiming(int8_t advance)
{
  int16_t ignFlexValue = advance;

  if (configPage2.flexEnabled == 1) //Check for flex being enabled
  {
    //Negative values are achieved with offset
    ignFlexValue = (int16_t)table2D_getValue(&flexAdvTable, currentStatus.ethanolPct) - OFFSET_IGNITION;
    //This gets cast to a signed 8 bit value to allow for negative advance (ie retard).
    currentStatus.flexIgnCorrection = (int8_t)ignFlexValue;
    ignFlexValue = (int8_t)advance + currentStatus.flexIgnCorrection;
  }

  return ignFlexValue;
}

int8_t correctionWMITiming(int8_t advance)
{
  if (configPage10.wmiEnabled >= 1
      && configPage10.wmiAdvEnabled == 1
      && !BIT_CHECK(currentStatus.status4, BIT_STATUS4_WMI_EMPTY))
  {
    if (currentStatus.TPS >= configPage10.wmiTPS
        && currentStatus.RPM >= configPage10.wmiRPM
        && currentStatus.MAP / 2 >= configPage10.wmiMAP
        && currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET >= configPage10.wmiIAT)
    {
      //Negative values are achieved with offset
      return (int16_t)advance + table2D_getValue(&wmiAdvTable, currentStatus.MAP / 2) - OFFSET_IGNITION;
    }
  }

  return advance;
}

/** Ignition correction for inlet air temperature (IAT).
 */
int8_t correctionIATretard(int8_t advance)
{
  int8_t advanceIATadjust = table2D_getValue(&IATRetardTable, currentStatus.IAT);

  return advance - advanceIATadjust;
}

/** Ignition correction for coolant temperature (CLT).
 */
int8_t correctionCLTadvance(int8_t advance)
{
  //Adjust the advance based on CLT.
  int8_t advanceCLTadjust =
     (int16_t)(table2D_getValue(&CLTAdvanceTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) - 15;
  int8_t ignCLTValue = advance + advanceCLTadjust;

  return ignCLTValue;
}

/** Ignition Idle advance correction.
 */
int8_t correctionIdleAdvance(int8_t advance)
{
  int8_t ignIdleValue = advance;

  //Adjust the advance based on idle target rpm.
  if (configPage2.idleAdvEnabled >= 1
      && runSecsX10 >= configPage2.idleAdvDelay * 5
      && idleAdvActive)
  {
    int idleRPMdelta = (currentStatus.CLIdleTarget - (currentStatus.RPM / 10)) + 50;

    // Limit idle rpm delta between -500rpm - 500rpm
    if (idleRPMdelta > 100)
    {
      idleRPMdelta = 100;
    }
    if (idleRPMdelta < 0)
    {
      idleRPMdelta = 0;
    }

    if (currentStatus.RPM < configPage2.idleAdvRPM * 100
        && (configPage2.vssMode == 0 || currentStatus.vss < configPage2.idleAdvVss)
        && ((configPage2.idleAdvAlgorithm == 0 && currentStatus.TPS < configPage2.idleAdvTPS)
            || (configPage2.idleAdvAlgorithm == 1 && currentStatus.CTPSActive == 1))) // closed throttle position sensor (CTPS) based idle state
    {
      if (idleAdvTaper < configPage9.idleAdvStartDelay)
      {
        if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ))
        {
          idleAdvTaper++;
        }
      }
      else
      {
        int8_t advanceIdleAdjust = (int16_t)(table2D_getValue(&idleAdvanceTable, idleRPMdelta)) - 15;
        if (configPage2.idleAdvEnabled == 1)
        {
          ignIdleValue = (advance + advanceIdleAdjust);
        }
        else if (configPage2.idleAdvEnabled == 2)
        {
          ignIdleValue = advanceIdleAdjust;
        }
      }
    }
    else
    {
      idleAdvTaper = 0;
    }
  }

  /*
   * When Idle advance is the only idle speed control mechanism, activate as soon as not cranking.
   * When some other mechanism is also present, wait until the engine is no more
   * than 200 RPM below idle target speed on first time.
   */

  if (!idleAdvActive)
  {
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN)
        && (configPage6.iacAlgorithm == 0
            || currentStatus.RPM > (((uint16_t)currentStatus.CLIdleTarget * 10) - (uint16_t)IGN_IDLE_THRESHOLD)))
    {
      idleAdvActive = true;
    }
  }
  else if (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN)) //Clear flag if engine isn't running anymore
  {
    idleAdvActive = false;
  }

  return ignIdleValue;
}

/** Ignition soft revlimit correction.
 */
int8_t correctionSoftRevLimit(int8_t advance)
{
  byte ignSoftRevValue = advance;

  BIT_CLEAR(currentStatus.spark, BIT_SPARK_SFTLIM);

  if (configPage6.engineProtectType == PROTECT_CUT_IGN || configPage6.engineProtectType == PROTECT_CUT_BOTH)
  {
    if (currentStatus.RPMdiv100 >= configPage4.SoftRevLim) //Softcut RPM limit
    {
      BIT_SET(currentStatus.spark, BIT_SPARK_SFTLIM);
      if (softLimitTime < configPage4.SoftLimMax)
      {
        if (configPage2.SoftLimitMode == SOFT_LIMIT_RELATIVE) //delay timing by configured number of degrees in relative mode
        {
          ignSoftRevValue = ignSoftRevValue - configPage4.SoftLimRetard;
        }
        else if (configPage2.SoftLimitMode == SOFT_LIMIT_FIXED) //delay timing to configured number of degrees in fixed mode
        {
          ignSoftRevValue = configPage4.SoftLimRetard;
        }

        if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ))
        {
          softLimitTime++;
        }
      }
    }
    else if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) //Only reset time at runSecsX10 update rate
    {
      softLimitTime = 0;
    }
  }

  return ignSoftRevValue;
}

/** Ignition Nitrous oxide correction.
 */
int8_t correctionNitrous(int8_t advance)
{
  byte ignNitrous = advance;

  //Check if nitrous is currently active
  if (configPage10.n2o_enable > 0)
  {
    //Check which stage is running (if any)
    if ((currentStatus.nitrous_status == NITROUS_STAGE1) || (currentStatus.nitrous_status == NITROUS_BOTH))
    {
      ignNitrous -= configPage10.n2o_stage1_retard;
    }
    if ((currentStatus.nitrous_status == NITROUS_STAGE2) || (currentStatus.nitrous_status == NITROUS_BOTH))
    {
      ignNitrous -= configPage10.n2o_stage2_retard;
    }
  }

  return ignNitrous;
}

/** Ignition soft launch correction.
 */
int8_t correctionSoftLaunch(int8_t advance)
{
  byte ignSoftLaunchValue = advance;

  //SoftCut rev limit for 2-step launch control.
  if (configPage6.launchEnabled
      && currentStatus.clutchTrigger
      && currentStatus.clutchEngagedRPM < (unsigned int)configPage6.flatSArm * 100
      && currentStatus.RPM > (unsigned int)configPage6.lnchSoftLim * 100
      && currentStatus.TPS >= configPage10.lnchCtrlTPS)
  {
    currentStatus.launchingSoft = true;
    BIT_SET(currentStatus.spark, BIT_SPARK_SLAUNCH);
    ignSoftLaunchValue = configPage6.lnchRetard;
  }
  else
  {
    currentStatus.launchingSoft = false;
    BIT_CLEAR(currentStatus.spark, BIT_SPARK_SLAUNCH);
  }

  return ignSoftLaunchValue;
}
/** Ignition correction for soft flat shift.
 */
int8_t correctionSoftFlatShift(int8_t advance)
{
  int8_t ignSoftFlatValue = advance;

  if (configPage6.flatSEnable
      && currentStatus.clutchTrigger
      && currentStatus.clutchEngagedRPM > ((unsigned int)configPage6.flatSArm * 100)
      && currentStatus.RPM > currentStatus.clutchEngagedRPM - (configPage6.flatSSoftWin * 100))
  {
    BIT_SET(currentStatus.spark2, BIT_SPARK2_FLATSS);
    ignSoftFlatValue = configPage6.flatSRetard;
  }
  else
  {
    BIT_CLEAR(currentStatus.spark2, BIT_SPARK2_FLATSS);
  }

  return ignSoftFlatValue;
}
/** Ignition knock (retard) correction.
 */
int8_t correctionKnock(int8_t advance)
{
  byte knockRetard = 0;

  //First check is to do the window calculations (Assuming knock is enabled)
  if (configPage10.knock_mode != KNOCK_MODE_OFF)
  {
    knockWindowMin = table2D_getValue(&knockWindowStartTable, currentStatus.RPMdiv100);
    knockWindowMax = knockWindowMin + table2D_getValue(&knockWindowDurationTable, currentStatus.RPMdiv100);
  }

  if ((configPage10.knock_mode == KNOCK_MODE_DIGITAL))
  {
    //
    if (knockCounter > configPage10.knock_count)
    {
      if (currentStatus.knockActive)
      {
        //Knock retard is currently active.
      }
      else
      {
        //Knock needs to be activated
        lastKnockCount = knockCounter;
        knockStartTime = micros();
        knockRetard = configPage10.knock_firstStep;
      }
    }

  }

  return advance - knockRetard;
}

/** Ignition DFCO taper correction.
 */
int8_t correctionDFCOignition(int8_t advance)
{
  int8_t dfcoRetard = advance;

  if (configPage9.dfcoTaperEnable == 1 && BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO))
  {
    if (dfcoTaper != 0)
    {
      dfcoRetard -= map(dfcoTaper, configPage9.dfcoTaperTime, 0, 0, configPage9.dfcoTaperAdvance);
    }
    else //Taper ended, use full value
    {
      dfcoRetard -= configPage9.dfcoTaperAdvance;
    }
  }
  else //Keep updating the duration until DFCO is active
  {
    dfcoTaper = configPage9.dfcoTaperTime;
  }

  return dfcoRetard;
}

/** Ignition Dwell Correction.
 */
uint16_t correctionsDwell(uint16_t dwell)
{
  uint16_t tempDwell = dwell;
  //Spark duration config is in mS*10.
  uint16_t sparkDur_uS = MS_TIMES_10_TO_US(configPage4.sparkDur);

  if(currentStatus.actualDwell == 0)
  {
    //Initialise the actualDwell value if this is the first time being called.
    currentStatus.actualDwell = tempDwell;
  }

  //**************************************************************************************************************************
  //Pull battery voltage based dwell correction and apply if needed
  currentStatus.dwellCorrection = table2D_getValue(&dwellVCorrectionTable, currentStatus.battery10);
  tempDwell = percentage(currentStatus.dwellCorrection, dwell);

  //**************************************************************************************************************************
  //Dwell error correction is a basic closed loop to keep the dwell time
  //consistent even when adjusting its end time for the per tooth timing.
  //This is mostly of benefit to low resolution triggers at low rpm (<1500)
  if (configPage2.perToothIgn && configPage4.dwellErrCorrect == 1)
  {
    int16_t error = tempDwell - currentStatus.actualDwell;

    //Prevent overflow when casting to signed int
    if (tempDwell > INT16_MAX)
    {

      tempDwell = INT16_MAX;
    }
    //Double correction amount if actual dwell is less than 50% of the requested dwell
    if (error > ((int16_t)tempDwell / 2))
    {
      error += error;
    }

    if (error > 0)
    {
      tempDwell += error;
    }
  }

  //**************************************************************************************************************************
  /*
  Dwell limiter - If the total required dwell time per revolution is longer than
  the maximum time available at the current RPM, reduce dwell.
  This can occur if there are multiple sparks per revolution
  This only times this can occur are:
  1. Single channel spark mode where there will be nCylinders/2 sparks per revolution
  2. Rotary ignition in wasted spark configuration (FC/FD), results in 2 pulses per rev. R
  X-8 is fully sequential resulting in 1 pulse, so not required
  */
  uint16_t dwellPerRevolution = tempDwell + sparkDur_uS;
  int8_t pulsesPerRevolution = 1;

  if ((configPage4.sparkMode == IGN_MODE_SINGLE
       || (configPage4.sparkMode == IGN_MODE_ROTARY && configPage10.rotaryType != ROTARY_IGN_RX8))
      && configPage2.nCylinders > 1
      ) //No point in running this for 1 cylinder engines
  {
    pulsesPerRevolution = (configPage2.nCylinders >> 1);
    dwellPerRevolution = dwellPerRevolution * pulsesPerRevolution;
  }

  if (dwellPerRevolution > revolutionTime)
  {
    //Possibly need some method of reducing spark duration here as well, but this is a start
    uint16_t adjustedSparkDur = udiv_32_16(sparkDur_uS * revolutionTime, dwellPerRevolution);
    tempDwell = udiv_32_16(revolutionTime, (uint16_t)pulsesPerRevolution) - adjustedSparkDur;
  }

  return tempDwell;
}
