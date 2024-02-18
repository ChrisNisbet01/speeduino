#include "auxiliary_pins.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)
IODigitalWriteOutputPin boost;
IODigitalWriteOutputPin FuelPump;
#else
IOAtomicWriteOutputPin boost;
IOAtomicWriteOutputPin FuelPump;
#endif

IOPortMaskOutputPin TachOut;

IOPortMaskOutputPin Idle1;
IOPortMaskOutputPin Idle2;
IOPortMaskOutputPin IdleUpOutput;
IOPortMaskOutputPin Fan;
IOPortMaskOutputPin StepperDir;
IOPortMaskOutputPin StepperStep;
IOPortMaskOutputPin StepperEnable;
IOPortMaskOutputPin VVT_1;
IOPortMaskOutputPin VVT_2;
IOPortMaskOutputPin IgnBypass;

