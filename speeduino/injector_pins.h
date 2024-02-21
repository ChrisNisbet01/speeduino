#pragma once

#include "pin.h"

//These are for the direct port manipulation of the injectors.
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

