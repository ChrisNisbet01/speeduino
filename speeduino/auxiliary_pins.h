#pragma once

#include "pin.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)

extern IODigitalWriteOutputPin boost;
extern IODigitalWriteOutputPin FuelPump;
extern IODigitalWriteOutputPin Fan;
extern IODigitalWriteOutputPin VVT_1;
extern IODigitalWriteOutputPin VVT_2;
extern IODigitalWriteOutputPin AirConComp;
extern IODigitalWriteOutputPin AirConFan;

extern IODigitalWriteOutputPin Trigger;
extern IODigitalWriteOutputPin Trigger2;
extern IODigitalWriteOutputPin Trigger3;

extern IODigitalWriteOutputPin Flex;

extern IODigitalWriteOutputPin NitrousStage1;
extern IODigitalWriteOutputPin NitrousStage2;

#else

extern IOAtomicWriteOutputPin boost;
extern IOAtomicWriteOutputPin FuelPump;
extern IOAtomicWriteOutputPin Fan;
extern IOAtomicWriteOutputPin VVT_1;
extern IOAtomicWriteOutputPin VVT_2;
extern IOAtomicWriteOutputPin AirConComp;
extern IOAtomicWriteOutputPin AirConFan;

extern IOPortMaskOutputPin Trigger;
extern IOPortMaskOutputPin Trigger2;
extern IOPortMaskOutputPin Trigger3;

extern IOPortMaskOutputPin Flex;

extern IOPortMaskOutputPin NitrousStage1;
extern IOPortMaskOutputPin NitrousStage2;

#endif

extern IOPortMaskOutputPin TachOut;
extern IOPortMaskOutputPin Idle1;
extern IOPortMaskOutputPin Idle2;
extern IOPortMaskOutputPin IdleUpOutput;

extern IODigitalWriteOutputPin StepperDir;
extern IODigitalWriteOutputPin StepperStep;
extern IODigitalWriteOutputPin StepperEnable;

extern IODigitalWriteOutputPin IgnBypass;

extern IODigitalWriteOutputPin WMIEnabled;
extern IODigitalWriteOutputPin WMIIndicator;

extern IOPortMaskOutputPin NitrousArming;

extern IOPortMaskOutputPin AirConRequest;

