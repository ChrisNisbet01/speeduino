#pragma once

#include <stdint.h>
#include "scheduler.h"

extern int ignition1EndAngle;

extern int ignition2EndAngle;

extern int ignition3EndAngle;

extern int ignition4EndAngle;

#if (IGN_CHANNELS >= 5)
extern int ignition5EndAngle;
#endif
#if (IGN_CHANNELS >= 6)
extern int ignition6EndAngle;
#endif
#if (IGN_CHANNELS >= 7)
extern int ignition7EndAngle;
#endif
#if (IGN_CHANNELS >= 8)
extern int ignition8EndAngle;
#endif

static inline uint16_t __attribute__((always_inline))
calculateInjectorStartAngle(uint16_t PWdivTimerPerDegree, int16_t injChannelDegrees, uint16_t injAngle);

static inline uint32_t __attribute__((always_inline))
calculateInjectorTimeout(const FuelSchedule &schedule, int channelInjDegrees, int injectorStartAngle, int crankAngle);

static inline void __attribute__((always_inline))
calculateIgnitionAngle(const int dwellAngle, const uint16_t channelIgnDegrees, int8_t advance, int *pEndAngle, int *pStartAngle);

// Ignition for rotary.
static inline void __attribute__((always_inline))
calculateIgnitionTrailingRotary(int dwellAngle, int rotarySplitDegrees, int leadIgnitionAngle, int *pEndAngle, int *pStartAngle);

static inline uint32_t __attribute__((always_inline))
calculateIgnitionTimeout(const IgnitionSchedule &schedule, int startAngle, int channelIgnDegrees, int crankAngle);

#include "schedule_calcs.hpp"
