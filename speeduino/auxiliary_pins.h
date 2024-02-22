#pragma once

#include "pin.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)

extern IODigitalWriteOutputPin Boost;
extern IODigitalWriteOutputPin FuelPump;
extern IODigitalWriteOutputPin Fan;
extern IODigitalWriteOutputPin VVT_1;
extern IODigitalWriteOutputPin VVT_2;
extern IODigitalWriteOutputPin AirConComp;
extern IODigitalWriteOutputPin AirConFan;

extern IODigitalReadInputPin Trigger;
extern IODigitalReadInputPin Trigger2;
extern IODigitalReadInputPin Trigger3;

extern IODigitalReadInputPin Flex;

extern IODigitalWriteOutputPin NitrousStage1;
extern IODigitalWriteOutputPin NitrousStage2;

#else

extern IOAtomicWriteOutputPin Boost;
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

extern IODigitalReadInputPin IdleUp;   //Input for triggering Idle Up
extern IODigitalReadInputPin CTPS;     //Input for triggering closed throttle state
extern IODigitalReadInputPin Fuel2Input;  //Input for switching to the 2nd fuel table
extern IODigitalReadInputPin Spark2Input; //Input for switching to the 2nd ignition table
extern IODigitalReadInputPin Launch;
extern IODigitalReadInputPin VSS;  // VSS (Vehicle speed sensor) Pin
extern IODigitalReadInputPin WMIEmpty; // Water tank empty sensor
extern IODigitalReadInputPin SDEnable;

extern IODigitalWriteOutputPin ResetControl; // Output pin used control resetting the Arduino

