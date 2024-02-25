#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_harley;

void triggerSetup_Harley(bool initialisationComplete);

void triggerPri_Harley(void);

void triggerSec_Harley(void);

uint16_t getRPM_Harley(void);

int getCrankAngle_Harley(void);

