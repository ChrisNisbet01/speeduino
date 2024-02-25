#pragma once

#include <stdint.h>

void triggerSetup_NGC(bool initialisationComplete = false);

void triggerPri_NGC(void);

void triggerSec_NGC4(void);

void triggerSec_NGC68(void);

uint16_t getRPM_NGC(void);

void triggerSetEndTeeth_NGC(void);

