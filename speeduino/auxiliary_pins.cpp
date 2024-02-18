#include "auxiliary_pins.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)
IODigitalWriteOutputPin boost;
#else
IOAtomicWriteOutputPin boost;
#endif

IOPortMaskOutputPin TachOut;

