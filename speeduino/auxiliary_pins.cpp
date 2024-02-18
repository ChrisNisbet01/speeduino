#include "auxiliary_pins.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)

IODigitalWriteOutputPin boost;
IODigitalWriteOutputPin FuelPump;
IODigitalWriteOutputPin Fan;
IODigitalWriteOutputPin VVT_1;
IODigitalWriteOutputPin VVT_2;

#else

IOAtomicWriteOutputPin boost;
IOAtomicWriteOutputPin FuelPump;
IOAtomicWriteOutputPin Fan;
IOAtomicWriteOutputPin VVT_1;
IOAtomicWriteOutputPin VVT_2;

#endif

IOPortMaskOutputPin TachOut;

IOPortMaskOutputPin Idle1;
IOPortMaskOutputPin Idle2;
IOPortMaskOutputPin IdleUpOutput;

IODigitalWriteOutputPin StepperDir;
IODigitalWriteOutputPin StepperStep;
IODigitalWriteOutputPin StepperEnable;
IODigitalWriteOutputPin IgnBypass;

