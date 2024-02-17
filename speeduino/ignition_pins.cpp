#include "ignition_pins.h"

//These are for the direct port manipulation of the ignition pins
IOPin ign1;
IOPin ign2;
IOPin ign3;
IOPin ign4;
#if IGN_CHANNELS >= 5
IOPin ign5;
#endif
#if IGN_CHANNELS >= 6
IOPin ign6;
#endif
#if IGN_CHANNELS >= 7
IOPin ign7;
#endif
#if IGN_CHANNELS >= 8
IOPin ign8;
#endif

