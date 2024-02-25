#pragma once

#include <stdint.h>

void triggerSetup_MazdaAU(bool initialisationComplete = false);

void triggerPri_MazdaAU(void);

void triggerSec_MazdaAU(void);

uint16_t getRPM_MazdaAU(void);

int getCrankAngle_MazdaAU(void);

void triggerSetEndTeeth_MazdaAU(void);

