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

extern IODigitalWriteInputPin Trigger;
extern IODigitalWriteInputPin Trigger2;
extern IODigitalWriteInputPin Trigger3;

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

extern IOPortMaskInputPin Trigger;
extern IOPortMaskInputPin Trigger2;
extern IOPortMaskInputPin Trigger3;

extern IOPortMaskInputPin Flex;

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

extern IOPortMaskInputPin NitrousArming;

extern IOPortMaskInputPin AirConRequest;

