#include "globals.h"
#include "secondaryTables.h"
#include "corrections.h"
#include "engine_load_calcs.h"
#include "auxiliary_pins.h"
#include "bit_macros.h"
#include "maths.h"

void calculateSecondaryFuel(void)
{
  //If the secondary fuel table is in use, also get the VE value from there
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE);
  bool fuel2_is_active = false;
  bool use_base_ve2 = false;

  if (configPage10.fuel2Mode == FUEL2_MODE_MULTIPLY)
  {
    fuel2_is_active = true;
    currentStatus.VE2 = getVE2();
    //Fuel 2 table is treated as a % value.
    uint16_t const combinedVE = percentage(currentStatus.VE2, currentStatus.VE1);

    if (combinedVE <= 255)
    {
      currentStatus.VE = combinedVE;
    }
    else
    {
      currentStatus.VE = 255;
    }
  }
  else if (configPage10.fuel2Mode == FUEL2_MODE_ADD)
  {
    fuel2_is_active = true;
    currentStatus.VE2 = getVE2();
    //Fuel tables are added together, but a check is made to make sure this won't overflow the 8-bit VE value
    uint16_t combinedVE = (uint16_t)currentStatus.VE1 + (uint16_t)currentStatus.VE2;
    if (combinedVE <= 255)
    {
      currentStatus.VE = combinedVE;
    }
    else
    {
      currentStatus.VE = 255;
    }
  }
  else if (configPage10.fuel2Mode == FUEL2_MODE_CONDITIONAL_SWITCH )
  {
    if (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_RPM)
    {
      if (currentStatus.RPM > configPage10.fuel2SwitchValue)
      {
        use_base_ve2 = true;
      }
    }
    else if (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_MAP)
    {
      if (currentStatus.MAP > configPage10.fuel2SwitchValue)
      {
        use_base_ve2 = true;
      }
    }
    else if (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_TPS)
    {
      if (currentStatus.TPS > configPage10.fuel2SwitchValue)
      {
        use_base_ve2 = true;
      }
    }
    else if (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_ETH)
    {
      if (currentStatus.ethanolPct > configPage10.fuel2SwitchValue)
      {
        use_base_ve2 = true;
      }
    }
  }
  else if (Fuel2Input.is_configured())
  {
    if (Fuel2Input.read() == configPage10.fuel2InputPolarity)
    {
      use_base_ve2 = true;
    }
  }
  else
  {
    /* Do nothing. */
  }

  if (fuel2_is_active || use_base_ve2)
  {
    BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE);
    if (use_base_ve2)
    {
      currentStatus.VE2 = getVE2();
      currentStatus.VE = currentStatus.VE2;
    }
  }
}


void calculateSecondarySpark(void)
{
  //Same as above but for the secondary ignition table
  //Clear the bit indicating that the 2nd spark table is in use.
  bool spark2_is_active = false;

  if (configPage10.spark2Mode > 0)
  {
    int8_t advance = 0;
    bool spark2_advance_required = false;

    if (configPage10.spark2Mode == SPARK2_MODE_MULTIPLY)
    {
      currentStatus.advance2 = getAdvance2();
      //make sure we don't have a negative value in the multiplier table
      //(sharing a signed 8 bit table)
      if(currentStatus.advance2 < 0)
      {
        currentStatus.advance2 = 0;
      }
      //Spark 2 table is treated as a % value.
      int16_t const combinedAdvance = percentage(currentStatus.advance2, currentStatus.advance1);

      //make sure we don't overflow and accidentally set negative timing,
      //currentStatus.advance can only hold a signed 8 bit value
      if (combinedAdvance <= INT8_MAX)
      {
        advance = combinedAdvance;
      }
      else
      {
        advance = INT8_MAX;
      }
      spark2_is_active = true;
    }
    else if (configPage10.spark2Mode == SPARK2_MODE_ADD)
    {
      currentStatus.advance2 = getAdvance2();
      //Spark tables are added together, but a check is made to make sure this
      //won't overflow the 8-bit VE value
      int16_t const combinedAdvance = (int16_t)currentStatus.advance1 + (int16_t)currentStatus.advance2;
      //make sure we don't overflow and accidentally set negative timing,
      //currentStatus.advance can only hold a signed 8 bit value
      if (combinedAdvance <= INT8_MAX)
      {
        advance = combinedAdvance;
      }
      else
      {
        advance = INT8_MAX;
      }
      spark2_is_active = true;
    }
    else if (configPage10.spark2Mode == SPARK2_MODE_CONDITIONAL_SWITCH)
    {
      if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_RPM)
      {
        if (currentStatus.RPM > configPage10.spark2SwitchValue)
        {
          spark2_advance_required = true;
        }
      }
      else if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_MAP)
      {
        if (currentStatus.MAP > configPage10.spark2SwitchValue)
        {
          spark2_advance_required = true;
        }
      }
      else if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_TPS)
      {
        if (currentStatus.TPS > configPage10.spark2SwitchValue)
        {
          spark2_advance_required = true;
        }
      }
      else if (configPage10.spark2SwitchVariable == SPARK2_CONDITION_ETH)
      {
        if (currentStatus.ethanolPct > configPage10.spark2SwitchValue)
        {
          spark2_advance_required = true;
        }
      }
    }
    else if (Spark2Input.is_configured()
             && Spark2Input.read() == configPage10.spark2InputPolarity)
    {
      spark2_advance_required = true;
    }

    if (spark2_advance_required)
    {
      currentStatus.advance2 = getAdvance2();
      advance = currentStatus.advance2;
      spark2_is_active = true;
    }

    if (spark2_is_active)
    {
      BIT_SET(currentStatus.spark2, BIT_SPARK2_SPARK2_ACTIVE);

      bool const timing_is_fixed =
        configPage2.fixAngEnable == 1 || BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK);

      if (!timing_is_fixed)
      {
        currentStatus.advance = advance;
      }
      /*
       * Else currentStatus.advance should remain at the previously calculated
       * (by getAdvance()) fixed value.
       */
    }
  }

  if (!spark2_is_active)
  {
    BIT_CLEAR(currentStatus.spark2, BIT_SPARK2_SPARK2_ACTIVE);
  }
}

/**
 * @brief Looks up and returns the VE value from the secondary fuel table
 *
 * This performs largely the same operations as getVE() however the lookup is of
 * the secondary fuel table and uses the secondary load source
 * @return byte
 */
byte getVE2(void)
{
  currentStatus.fuelLoad2 =
    calculate_engine_load((load_source_t)configPage10.fuel2Algorithm, currentStatus);
  //Perform lookup into fuel map for RPM vs MAP value
  byte tempVE = get3DTableValue(&fuelTable2, currentStatus.fuelLoad2, currentStatus.RPM);

  return tempVE;
}

/**
 * @brief Performs a lookup of the second ignition advance table.
 * @brief The values used to look this up will be RPM and whatever load source the user has configured
 *
 * @return byte The current target advance value in degrees
 */
byte getAdvance2(void)
{
  currentStatus.ignLoad2 =
    calculate_engine_load((load_source_t)configPage10.spark2Algorithm, currentStatus);
  //As for VE2, but for ignition advance
  byte spark2_ignition_advance =
    get3DTableValue(&ignitionTable2, currentStatus.ignLoad2, currentStatus.RPM) - OFFSET_IGNITION;

  //Perform the corrections calculation on the secondary advance value,
  //only if it uses a switched mode
  if (configPage10.spark2SwitchVariable == SPARK2_MODE_CONDITIONAL_SWITCH
      || configPage10.spark2SwitchVariable == SPARK2_MODE_INPUT_SWITCH)
  {
    spark2_ignition_advance = correctionsIgn(spark2_ignition_advance);
  }

  return spark2_ignition_advance;
}
