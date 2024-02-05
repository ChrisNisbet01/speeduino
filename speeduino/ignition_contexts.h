#pragma once

#include "scheduler.h"

typedef enum
{
  ignChannel1,
  ignChannel2,
  ignChannel3,
  ignChannel4,
  ignChannel5,
#if IGN_CHANNELS >= 6
  ignChannel6,
#endif
#if IGN_CHANNELS >= 6
  ignChannel7,
#endif
#if IGN_CHANNELS >= 6
  ignChannel8,
#endif
  ignChannelCount,
} ignitionChannelID_t;

typedef struct ignition_context_st
{
public:
  IgnitionSchedule * ignitionSchedule;
} ignition_context_st;


typedef struct ignition_contexts_st
{
public:

ignition_context_st& ignition(ignitionChannelID_t ign)
  {
    return ignitions[ign];
  }


private:
  ignition_context_st ignitions[ignChannelCount];

} ignition_contexts_st;

extern ignition_contexts_st ignitions;

