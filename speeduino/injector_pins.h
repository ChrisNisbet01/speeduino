#pragma once

#include "pin.h"

//These are for the direct port manipulation of the injectors
extern IOPortMaskOutputPin inj1;
extern IOPortMaskOutputPin inj2;
extern IOPortMaskOutputPin inj3;
extern IOPortMaskOutputPin inj4;
#if (INJ_CHANNELS >= 5)
extern IOPortMaskOutputPin inj5;
#endif
#if (INJ_CHANNELS >= 6)
extern IOPortMaskOutputPin inj6;
#endif
#if (INJ_CHANNELS >= 7)
extern IOPortMaskOutputPin inj7;
#endif
#if (INJ_CHANNELS >= 8)
extern IOPortMaskOutputPin inj8;
#endif

static inline bool pinIsInjector(byte const pin)
{
  return pin == inj1.pin || pin == inj2.pin || pin == inj3.pin || pin == inj4.pin
#if INJ_CHANNELS >= 5
    || pin == inj5.pin
#endif
#if INJ_CHANNELS >= 6
    || pin == inj6.pin
#endif
#if INJ_CHANNELS >= 7
    || pin == inj7.pin
#endif
#if INJ_CHANNELS >= 8
    || pin == inj8.pin
#endif
  ;
}

