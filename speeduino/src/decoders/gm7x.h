#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_gm7x;

void triggerSetup_GM7X(bool initialisationComplete = false);

void triggerPri_GM7X(void);

void triggerSec_GM7X(void);

uint16_t getRPM_GM7X(void);

int getCrankAngle_GM7X(void);

void triggerSetEndTeeth_GM7X(void);

