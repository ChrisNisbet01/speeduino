#pragma once

#include <stdint.h>

void triggerSetup_RoverMEMS(bool initialisationComplete = false);

void triggerPri_RoverMEMS(void);

void triggerSec_RoverMEMS(void);

uint16_t getRPM_RoverMEMS(void);

int getCrankAngle_RoverMEMS(void);

void triggerSetEndTeeth_RoverMEMS(void);

