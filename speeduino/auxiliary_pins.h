#pragma once

#include "pin.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)
extern IODigitalWriteOutputPin boost;
#else
extern IOAtomicWriteOutputPin boost;
#endif

extern IOPortMaskOutputPin TachOut;

