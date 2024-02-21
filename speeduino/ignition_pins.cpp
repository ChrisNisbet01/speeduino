#include "ignition_pins.h"

//These are for the direct port manipulation of the ignition coils.
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

