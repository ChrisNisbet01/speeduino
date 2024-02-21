#pragma once

#include "pin.h"

//These are for the direct port manipulation of the ignition coils.
extern IOPortMaskOutputPin ign1;
extern IOPortMaskOutputPin ign2;
extern IOPortMaskOutputPin ign3;
extern IOPortMaskOutputPin ign4;
#if IGN_CHANNELS >= 5
extern IOPortMaskOutputPin ign5;
#endif
#if IGN_CHANNELS >= 6
extern IOPortMaskOutputPin ign6;
#endif
#if IGN_CHANNELS >= 7
extern IOPortMaskOutputPin ign7;
#endif
#if IGN_CHANNELS >= 8
extern IOPortMaskOutputPin ign8;
#endif

