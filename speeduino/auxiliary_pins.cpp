#include "auxiliary_pins.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)

IODigitalWriteOutputPin boost;
IODigitalWriteOutputPin FuelPump;
IODigitalWriteOutputPin Fan;
IODigitalWriteOutputPin VVT_1;
IODigitalWriteOutputPin VVT_2;
IODigitalWriteOutputPin AirConComp;
IODigitalWriteOutputPin AirConFan;

IODigitalWriteOutputPin Trigger;
IODigitalWriteOutputPin Trigger2;
IODigitalWriteOutputPin Trigger3;

IODigitalWriteOutputPin Flex;

#else

IOAtomicWriteOutputPin boost;
IOAtomicWriteOutputPin FuelPump;
IOAtomicWriteOutputPin Fan;
IOAtomicWriteOutputPin VVT_1;
IOAtomicWriteOutputPin VVT_2;
IOAtomicWriteOutputPin AirConComp;
IOAtomicWriteOutputPin AirConFan;

IOPortMaskOutputPin Trigger;
IOPortMaskOutputPin Trigger2;
IOPortMaskOutputPin Trigger3;

IOPortMaskOutputPin Flex;

#endif

IOPortMaskOutputPin TachOut;

IOPortMaskOutputPin Idle1;
IOPortMaskOutputPin Idle2;
IOPortMaskOutputPin IdleUpOutput;

IODigitalWriteOutputPin StepperDir;
IODigitalWriteOutputPin StepperStep;
IODigitalWriteOutputPin StepperEnable;
IODigitalWriteOutputPin IgnBypass;

IODigitalWriteOutputPin WMIEnabled;
IODigitalWriteOutputPin WMIIndicator;

