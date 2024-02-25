#pragma once

#include <stdint.h>

void triggerSetup_DualWheel(bool initialisationComplete = false);

void triggerPri_DualWheel(void);

void triggerSec_DualWheel(void);

uint16_t getRPM_DualWheel(void);

int getCrankAngle_DualWheel(void);

void triggerSetEndTeeth_DualWheel(void);

