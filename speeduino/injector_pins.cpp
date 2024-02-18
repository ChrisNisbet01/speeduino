#include "injector_pins.h"

/// volatile inj*_pin_port and  inj*_pin_mask vars are for the direct port manipulation of the injectors, coils and aux outputs.
IOPortMaskOutputPin inj1;
IOPortMaskOutputPin inj2;
IOPortMaskOutputPin inj3;
IOPortMaskOutputPin inj4;
#if (INJ_CHANNELS >= 5)
IOPortMaskOutputPin inj5;
#endif
#if (INJ_CHANNELS >= 6)
IOPortMaskOutputPin inj6;
#endif
#if (INJ_CHANNELS >= 7)
IOPortMaskOutputPin inj7;
#endif
#if (INJ_CHANNELS >= 8)
IOPortMaskOutputPin inj8;
#endif

