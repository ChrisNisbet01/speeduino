#include "engine_load_calcs.h"

int16_t calculate_engine_load(load_source_t const load_source, struct statuses const &status, int16_t const fallback_value)
{
  int16_t load;

  switch (load_source)
  {
  //Speed Density
  case LOAD_SOURCE_MAP:
    load = status.MAP;
    break;

  //Alpha-N
  case LOAD_SOURCE_TPS:
    load = status.TPS * 2;
    break;

  //IMAP / EMAP
  case LOAD_SOURCE_IMAPEMAP:
    load = ((int16_t)status.MAP * 100U) / status.EMAP;
    break;

  default:
    load = fallback_value;
    break;
  }

  return load;
}
