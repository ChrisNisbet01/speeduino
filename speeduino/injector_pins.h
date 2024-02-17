#pragma once

#include "pin.h"

//These are for the direct port manipulation of the injectors
extern IOPin inj1;
extern IOPin inj2;
extern IOPin inj3;
extern IOPin inj4;
#if (INJ_CHANNELS >= 5)
extern IOPin inj5;
#endif
#if (INJ_CHANNELS >= 6)
extern IOPin inj6;
#endif
#if (INJ_CHANNELS >= 7)
extern IOPin inj7;
#endif
#if (INJ_CHANNELS >= 8)
extern IOPin inj8;
#endif

