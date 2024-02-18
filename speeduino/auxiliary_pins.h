#pragma once

#include "pin.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)

extern IODigitalWriteOutputPin boost;
extern IODigitalWriteOutputPin FuelPump;

#else

extern IOAtomicWriteOutputPin boost;
extern IOAtomicWriteOutputPin FuelPump;

#endif

extern IOPortMaskOutputPin TachOut;
extern IOPortMaskOutputPin Idle1;
extern IOPortMaskOutputPin Idle2;
extern IOPortMaskOutputPin IdleUpOutput;
extern IOPortMaskOutputPin Fan;
extern IOPortMaskOutputPin StepperDir;
extern IOPortMaskOutputPin StepperStep;
extern IOPortMaskOutputPin StepperEnable;
extern IOPortMaskOutputPin VVT_1;
extern IOPortMaskOutputPin VVT_2;
extern IOPortMaskOutputPin IgnBypass;

