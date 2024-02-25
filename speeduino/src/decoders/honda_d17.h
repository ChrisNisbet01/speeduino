#pragma once

#include <stdint.h>

void triggerSetup_HondaD17(bool initialisationComplete = false);

void triggerPri_HondaD17(void);

void triggerSec_HondaD17(void);

uint16_t getRPM_HondaD17(void);

int getCrankAngle_HondaD17(void);

void triggerSetEndTeeth_HondaD17(void);

