#pragma once

#include <stdint.h>

void triggerSetup_420a(bool initialisationComplete = false);

void triggerPri_420a(void);

void triggerSec_420a(void);

uint16_t getRPM_420a(void);

int getCrankAngle_420a(void);

void triggerSetEndTeeth_420a(void);

