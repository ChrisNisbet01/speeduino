#include "engine_load_calcs.h"

int16_t calculate_engine_load(load_source_t const load_source, struct statuses const &status)
{
  int16_t load;

  switch (load_source)
  {
  //Alpha-N
  case LOAD_SOURCE_TPS:
    load = status.TPS * 2;
    break;

  //IMAP / EMAP
  case LOAD_SOURCE_IMAPEMAP:
    load = ((int16_t)status.MAP * 100U) / status.EMAP;
    break;

  //Speed Density
  case LOAD_SOURCE_MAP:
  default: /* Default to Speed Density for unknown/unsupported load sources. */
    load = status.MAP;
    break;
  }

  return load;
}

