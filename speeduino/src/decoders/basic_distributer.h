#pragma once

#include <stdint.h>

void triggerSetup_BasicDistributor(bool initialisationComplete = false);

void triggerPri_BasicDistributor(void);

uint16_t getRPM_BasicDistributor(void);

int getCrankAngle_BasicDistributor(void);

void triggerSetEndTeeth_BasicDistributor(void);

