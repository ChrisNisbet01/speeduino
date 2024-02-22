#pragma once

#include "globals.h"

typedef enum injector_id_t
{
  injector_id_1 = 0,
  injector_id_2,
  injector_id_3,
  injector_id_4,
#if INJ_CHANNELS >= 5
  injector_id_5,
#endif
#if INJ_CHANNELS >= 6
  injector_id_6,
#endif
#if INJ_CHANNELS >= 7
  injector_id_7,
#endif
#if INJ_CHANNELS >= 8
  injector_id_8,
#endif
  injector_id_COUNT,
} injector_id_t;
