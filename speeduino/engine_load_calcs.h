#ifndef ENGINE_LOAD_CALCS_H__
#define ENGINE_LOAD_CALCS_H__

#include "globals.h"

int16_t calculate_engine_load(load_source_t load_source, struct statuses const &status, int16_t fallback_value);

#endif /* ENGINE_LOAD_CALCS_H__ */
