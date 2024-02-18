#pragma once

#include "pin.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)

extern IODigitalWriteOutputPin boost;
extern IODigitalWriteOutputPin FuelPump;
extern IODigitalWriteOutputPin Fan;
extern IODigitalWriteOutputPin VVT_1;
extern IODigitalWriteOutputPin VVT_2;

#else

extern IOAtomicWriteOutputPin boost;
extern IOAtomicWriteOutputPin FuelPump;
extern IOAtomicWriteOutputPin Fan;
extern IOAtomicWriteOutputPin VVT_1;
extern IOAtomicWriteOutputPin VVT_2;

#endif

extern IOPortMaskOutputPin TachOut;
extern IOPortMaskOutputPin Idle1;
extern IOPortMaskOutputPin Idle2;
extern IOPortMaskOutputPin IdleUpOutput;

extern IODigitalWriteOutputPin StepperDir;
extern IODigitalWriteOutputPin StepperStep;
extern IODigitalWriteOutputPin StepperEnable;

extern IODigitalWriteOutputPin IgnBypass;

