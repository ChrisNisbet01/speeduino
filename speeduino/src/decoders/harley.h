#pragma once

#include <stdint.h>

void triggerSetup_Harley(bool initialisationComplete);

void triggerPri_Harley(void);

void triggerSec_Harley(void);

uint16_t getRPM_Harley(void);

int getCrankAngle_Harley(void);

