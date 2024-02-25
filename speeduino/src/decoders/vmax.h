#pragma once

#include "decoder_structs.h"

#include <stdint.h>

extern decoder_handler_st const trigger_vmax;

void triggerSetup_Vmax(bool initialisationComplete);

void triggerPri_Vmax(void);

void triggerSec_Vmax(void);

uint16_t getRPM_Vmax(void);

int getCrankAngle_Vmax(void);

