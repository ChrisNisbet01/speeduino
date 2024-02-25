#pragma once

#include <stdint.h>

void triggerSetup_Jeep2000(bool initialisationComplete);

void triggerPri_Jeep2000(void);

void triggerSec_Jeep2000(void);

uint16_t getRPM_Jeep2000(void);

int getCrankAngle_Jeep2000(void);

