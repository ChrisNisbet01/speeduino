#pragma once

#include <stdint.h>

void triggerSetup_Daihatsu(bool initialisationComplete = false);

void triggerPri_Daihatsu(void);

void triggerSec_Daihatsu(void);

uint16_t getRPM_Daihatsu(void);

int getCrankAngle_Daihatsu(void);

