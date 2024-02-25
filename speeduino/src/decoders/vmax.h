#pragma once

#include <stdint.h>

void triggerSetup_Vmax(bool initialisationComplete);

void triggerPri_Vmax(void);

void triggerSec_Vmax(void);

uint16_t getRPM_Vmax(void);

int getCrankAngle_Vmax(void);

