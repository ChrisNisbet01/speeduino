#include "ignition_pins.h"

//These are for the direct port manipulation of the ignition pins
IOPortMaskOutputPin ign1;
IOPortMaskOutputPin ign2;
IOPortMaskOutputPin ign3;
IOPortMaskOutputPin ign4;
#if IGN_CHANNELS >= 5
IOPortMaskOutputPin ign5;
#endif
#if IGN_CHANNELS >= 6
IOPortMaskOutputPin ign6;
#endif
#if IGN_CHANNELS >= 7
IOPortMaskOutputPin ign7;
#endif
#if IGN_CHANNELS >= 8
IOPortMaskOutputPin ign8;
#endif

static inline bool pinIsIgnition(byte const pin)
{
  return pin == ign1.pin || pin == ign2.pin || pin == ign3.pin || pin == ign4.pin
#if IGN_CHANNELS >= 5
         || pin == ign5.pin
#endif
#if IGN_CHANNELS >= 6
         || pin == ign6.pin
#endif
#if IGN_CHANNELS >= 7
         || pin == ign7.pin
#endif
#if IGN_CHANNELS >= 8
         || pin == ign8.pin
#endif
    ;
}

