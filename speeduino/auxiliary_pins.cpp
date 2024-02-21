#include "auxiliary_pins.h"

#if defined(CORE_TEENSY) || defined(CORE_STM32)

IODigitalWriteOutputPin Boost;
IODigitalWriteOutputPin FuelPump;
IODigitalWriteOutputPin Fan;
IODigitalWriteOutputPin VVT_1;
IODigitalWriteOutputPin VVT_2;
IODigitalWriteOutputPin AirConComp;
IODigitalWriteOutputPin AirConFan;

IODigitalReadInputPin Trigger;
IODigitalReadInputPin Trigger2;
IODigitalReadInputPin Trigger3;

IODigitalWriteOutputPin Flex;

IODigitalWriteOutputPin NitrousStage1;
IODigitalWriteOutputPin NitrousStage2;

#else

IOAtomicWriteOutputPin Boost;
IOAtomicWriteOutputPin FuelPump;
IOAtomicWriteOutputPin Fan;
IOAtomicWriteOutputPin VVT_1;
IOAtomicWriteOutputPin VVT_2;
IOAtomicWriteOutputPin AirConComp;
IOAtomicWriteOutputPin AirConFan;

IOPortMaskInputPin Trigger;
IOPortMaskInputPin Trigger2;
IOPortMaskInputPin Trigger3;

IOPortMaskInputPin Flex;

IOPortMaskOutputPin NitrousStage1;
IOPortMaskOutputPin NitrousStage2;

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

IOPortMaskInputPin NitrousArming;

IOPortMaskInputPin AirConRequest;

IODigitalReadInputPin IdleUp;   //Input for triggering Idle Up
IODigitalReadInputPin CTPS;     //Input for triggering closed throttle state
IODigitalReadInputPin Fuel2Input;  //Input for switching to the 2nd fuel table
IODigitalReadInputPin Spark2Input; //Input for switching to the 2nd ignition table
IODigitalReadInputPin Launch;
IODigitalReadInputPin VSS;  // VSS (Vehicle speed sensor) Pin
IODigitalReadInputPin Baro; //Pin that an external barometric pressure sensor is attached to (If used)
IODigitalReadInputPin FuelPressure;
IODigitalReadInputPin OilPressure;
IODigitalReadInputPin WMIEmpty; // Water tank empty sensor
IODigitalReadInputPin SDEnable;

IODigitalWriteOutputPin ResetControl; // Output pin used control resetting the Arduino

