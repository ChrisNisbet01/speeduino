#pragma once

#include "pin.h"

//These are for the direct port manipulation of the ignition pins
extern IOPin ign1;
extern IOPin ign2;
extern IOPin ign3;
extern IOPin ign4;
#if IGN_CHANNELS >= 5
extern IOPin ign5;
#endif
#if IGN_CHANNELS >= 6
extern IOPin ign6;
#endif
#if IGN_CHANNELS >= 7
extern IOPin ign7;
#endif
#if IGN_CHANNELS >= 8
extern IOPin ign8;
#endif

