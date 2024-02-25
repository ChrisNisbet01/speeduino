#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_dual_wheel;

void triggerSetup_DualWheel(bool initialisationComplete = false);

void triggerPri_DualWheel(void);

void triggerSec_DualWheel(void);

uint16_t getRPM_DualWheel(void);

int getCrankAngle_DualWheel(void);

void triggerSetEndTeeth_DualWheel(void);

