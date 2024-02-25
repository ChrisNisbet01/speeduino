#pragma once

#include <stdint.h>

void triggerSetup_Subaru67(bool initialisationComplete = false);

void triggerPri_Subaru67(void);

void triggerSec_Subaru67(void);

uint16_t getRPM_Subaru67(void);

int getCrankAngle_Subaru67(void);

void triggerSetEndTeeth_Subaru67(void);

