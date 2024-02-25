#pragma once

#include <stdint.h>

void triggerSetup_24X(bool initialisationComplete = false);

void triggerPri_24X(void);

void triggerSec_24X(void);

uint16_t getRPM_24X(void);

int getCrankAngle_24X(void);

