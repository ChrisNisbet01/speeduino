#pragma once

#include "auxiliary_pins.h"
#include "globals.h"
#include "decoders.h"
#include "bit_macros.h"

void initialiseAll(void);
void initialiseTriggers(void);
void setPinMapping(byte boardID);
void changeHalfToFullSync(void);
void changeFullToHalfSync(void);

static inline bool
VSS_USES_RPM2(void)
{
  // VSS is on the same pin as RPM2 and RPM2 is not used as part of the decoder
  return VSS.is_configured()
         && VSS.pin == Trigger2.pin
         && !BIT_CHECK(decoderState, BIT_DECODER_HAS_SECONDARY);
}

static inline bool
FLEX_USES_RPM2(void)
{
  // Same as for VSS, but for Flex sensor
  return Flex.is_configured()
         && Flex.pin == Trigger2.pin
         && !BIT_CHECK(decoderState, BIT_DECODER_HAS_SECONDARY);
}
