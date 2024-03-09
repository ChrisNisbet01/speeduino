#include "calculateInjectorStaging.h"
#include "globals.h"
#include "maths.h"
#include "injector_contexts.h"
#include "bit_macros.h"

extern uint16_t inj_opentime_uS;
extern uint16_t staged_req_fuel_mult_pri;
extern uint16_t staged_req_fuel_mult_sec;

typedef struct staged_PW_st
{
  uint32_t primary_PW_us;
  uint32_t secondary_PW_us;
} staged_PW_st;

static struct staged_PW_st
calculateStagedInjectorPulsewidths(uint32_t const desiredPW, uint32_t const pwLimit)
{
  // Exclude the opening time from primary pulsewidth as it needs to be
  // multiplied out again by the pri/sec req_fuel values below.
  // It is added on again after the calculations.

  staged_PW_st staged_PW = {
      .primary_PW_us = desiredPW - inj_opentime_uS,
      .secondary_PW_us = 0,
  };
  bool staging_is_active = false;

  // Scale the 'full' pulsewidth by each of the injector capacities
  uint32_t tempPW1 = div100((uint32_t)staged_PW.primary_PW_us * staged_req_fuel_mult_pri);

  switch ((staging_mode_t)configPage10.stagingMode)
  {
  case STAGING_MODE_TABLE:
  {
    uint8_t const stagingSplit =
        get3DTableValue(&stagingTable, currentStatus.fuelLoad, currentStatus.RPM);

    if (stagingSplit > 0)
    {
      // This is ONLY needed in in table mode.
      // Auto mode only calculates the difference.
      uint32_t const tempPW3 =
          div100((uint32_t)staged_PW.primary_PW_us * staged_req_fuel_mult_sec);

      staging_is_active = true;
      staged_PW.secondary_PW_us = div100(stagingSplit * tempPW3);
    }
    staged_PW.primary_PW_us = div100((100U - stagingSplit) * tempPW1);

    break;
  }

  case STAGING_MODE_AUTO:
  {
    // If automatic mode, the primary injectors are used all the way up to their limit
    //(Configured by the pulsewidth limit setting)
    // If they exceed their limit, the extra duty is passed to the secondaries
    staged_PW.primary_PW_us = tempPW1;

    if (tempPW1 > pwLimit)
    {
      staging_is_active = true;
      staged_PW.primary_PW_us = pwLimit;

      uint32_t extraPW = tempPW1 - pwLimit;
      // Convert the 'left over' fuel amount from primary injector scaling to secondary
      staged_PW.secondary_PW_us =
          udiv_32_16(extraPW * staged_req_fuel_mult_sec, staged_req_fuel_mult_pri);
    }
    // If tempPW1 <= pwLImit it means that the entire fuel load can be handled
    // by the primaries and staging is inactive.
    break;
  }

  default:
    break;
  }

  if (staging_is_active)
  {
    BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  }
  else
  {
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  }

  /* Include the injector opening times. */
  staged_PW.primary_PW_us += inj_opentime_uS;
  if (staged_PW.secondary_PW_us > 0)
  {
    staged_PW.secondary_PW_us += inj_opentime_uS;
  }

  return staged_PW;
}

static void
assignStagedInjectorPulsewidths(staged_PW_st const &staged_PW)
{
  // Allocate the primary and secondary pulse widths based on the fuel configuration
  switch (configPage2.nCylinders)
  {
  case 1:
    // Primary pulsewidth on channel 1, secondary on channel 2
    injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel2].PW = staged_PW.secondary_PW_us;
    break;

  case 2:
    // Primary pulsewidth on channels 1 and 2, secondary on channels 3 and 4
    injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel3].PW = staged_PW.secondary_PW_us;
    injector_contexts[injChannel4].PW = staged_PW.secondary_PW_us;
    break;

  case 3:
    // 6 channels required for 'normal' 3 cylinder staging support
    injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel3].PW = staged_PW.primary_PW_us;
#if INJ_CHANNELS >= 6
    // Primary pulsewidth on channels 1, 2 and 3, secondary on channels 4, 5 and 6
    injector_contexts[injChannel4].PW = staged_PW.secondary_PW_us;
    injector_contexts[injChannel5].PW = staged_PW.secondary_PW_us;
    injector_contexts[injChannel6].PW = staged_PW.secondary_PW_us;
#else
    // If there are not enough channels, then primary pulsewidth is on
    // channels 1, 2 and 3, secondary on channel 4
    injector_contexts[injChannel4].PW = staged_PW.secondary_PW_us;
#endif
    break;

  case 4:
    if (configPage2.injLayout == INJ_SEQUENTIAL || configPage2.injLayout == INJ_SEMISEQUENTIAL)
    {
      // Staging with 4 cylinders semi/sequential requires 8 total channels
#if INJ_CHANNELS >= 8
      injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel3].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel4].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel5].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel6].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel7].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel8].PW = staged_PW.secondary_PW_us;
#elif INJ_CHANNELS >= 5
      // This is an invalid config as there are not enough outputs to support sequential + staging
      // Put the staging output to the non-existent channel 5
      injector_contexts[injChannel5].PW = staged_PW.secondary_PW_us;
#endif
    }
    else
    {
      injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel3].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel4].PW = staged_PW.secondary_PW_us;
    }
    break;

  case 5:
    // No easily supportable 5 cylinder staging option unless there are at least 5 channels
    injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel3].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel4].PW = staged_PW.primary_PW_us;
#if INJ_CHANNELS >= 5
    injector_contexts[injChannel5].PW = staged_PW.primary_PW_us;
#endif
#if INJ_CHANNELS >= 6
    injector_contexts[injChannel6].PW = staged_PW.secondary_PW_us;
#endif
    break;

  case 6:
    injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel3].PW = staged_PW.primary_PW_us;
#if INJ_CHANNELS >= 6
    // 6 cylinder staging only if not sequential
    if (configPage2.injLayout != INJ_SEQUENTIAL)
    {
      injector_contexts[injChannel4].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel5].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel6].PW = staged_PW.secondary_PW_us;
    }
#if INJ_CHANNELS >= 8
    else
    {
      injector_contexts[injChannel4].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel5].PW = staged_PW.primary_PW_us;
      injector_contexts[injChannel6].PW = staged_PW.primary_PW_us;
      // If there are 8 channels, then the 6 cylinder sequential option is
      // available by using channels 7 + 8 for staging
      injector_contexts[injChannel7].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel8].PW = staged_PW.secondary_PW_us;
    }
#endif
#endif
    break;

  case 8:
    injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel3].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel4].PW = staged_PW.primary_PW_us;
#if INJ_CHANNELS >= 8
    // 8 cylinder staging only if not sequential
    if (configPage2.injLayout != INJ_SEQUENTIAL)
    {
      injector_contexts[injChannel5].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel6].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel7].PW = staged_PW.secondary_PW_us;
      injector_contexts[injChannel8].PW = staged_PW.secondary_PW_us;
    }
#endif
    break;

  default:
    // Assume 4 cylinder non-seq for default
    injector_contexts[injChannel1].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel2].PW = staged_PW.primary_PW_us;
    injector_contexts[injChannel3].PW = staged_PW.secondary_PW_us;
    injector_contexts[injChannel4].PW = staged_PW.secondary_PW_us;
    break;
  }
}

void calculateInjectorStaging(unsigned int const desiredPW, uint32_t const pwLimit)
{
  // Calculate staging pulsewidths if used
  // To run staged injection, the number of cylinders must be less than or equal
  // to the injector channels (ie Assuming you're running paired injection,
  // you need at least as many injector channels as you have cylinders, half for
  // the primaries and half for the secondaries)

  // Final check is to ensure that DFCO isn't active, which would cause an
  // overflow below (See #267)
  if (configPage10.stagingEnabled
      && (configPage2.nCylinders <= INJ_CHANNELS || configPage2.injType == INJ_TYPE_TBODY)
      && desiredPW > inj_opentime_uS)
  {
    staged_PW_st const staged_PW = calculateStagedInjectorPulsewidths(desiredPW, pwLimit);

    assignStagedInjectorPulsewidths(staged_PW);
  }
  else
  {
    for (size_t i = injChannel1; i < injChannelCount; i++)
    {
      unsigned int const PW = (i < injectors.maxOutputs) ? desiredPW : 0;

      injector_contexts[i].PW = PW;
    }

    // Clear the staging active flag
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  }
}
