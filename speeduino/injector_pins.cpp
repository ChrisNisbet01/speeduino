#include "injector_pins.h"

/// volatile inj*_pin_port and  inj*_pin_mask vars are for the direct port manipulation of the injectors, coils and aux outputs.
IOPin inj1;
IOPin inj2;
IOPin inj3;
IOPin inj4;
#if (INJ_CHANNELS >= 5)
IOPin inj5;
#endif
#if (INJ_CHANNELS >= 6)
IOPin inj6;
#endif
#if (INJ_CHANNELS >= 7)
IOPin inj7;
#endif
#if (INJ_CHANNELS >= 8)
IOPin inj8;
#endif
