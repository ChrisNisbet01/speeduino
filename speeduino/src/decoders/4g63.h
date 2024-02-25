#pragma once

#include <stdint.h>

void triggerSetup_4G63(bool initialisationComplete);

void triggerPri_4G63(void);

void triggerSec_4G63(void);

uint16_t getRPM_4G63(void);

int getCrankAngle_4G63(void);

void triggerSetEndTeeth_4G63(void);

