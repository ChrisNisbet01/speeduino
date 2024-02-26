#pragma once
#include "globals.h"

typedef enum ignition_id_t
{
  ignition_id_1 = 0,
  ignition_id_2,
  ignition_id_3,
  ignition_id_4,
#if IGN_CHANNELS >= 5
  ignition_id_5,
#endif
#if IGN_CHANNELS >= 6
  ignition_id_6,
#endif
#if IGN_CHANNELS >= 7
  ignition_id_7,
#endif
#if IGN_CHANNELS >= 8
  ignition_id_8,
#endif
  ignition_id_COUNT,
} ignition_id_t;

