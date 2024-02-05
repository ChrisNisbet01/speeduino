#pragma once

#include <stdint.h>
#include "scheduler.h"

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
